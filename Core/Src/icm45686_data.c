/*
 * ICM-45686 FIFO HIRES parser and averaged output.
 *
 * ODR датчика: 3200 Гц.
 * TIM6 запускает чтение FIFO на 400 Гц.
 * За батч считываются восемь HIRES-пакетов по 20 байт.
 * Длина FIFO payload: 160 байт.
 *
 * ICM_ParseAllFIFO() сохраняет raw samples.
 * ICM_AverageAllBatches() вызывается отдельно из main loop
 * и формирует один output sample на каждый датчик.
 *
 * Big Endian layout, build20() и разбор raw-полей сохранены.
 */

#include <string.h>
#include <limits.h>

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"

#define FIFO_HDR_MSG_BIT    (1U << 7)
#define FIFO_HDR_ACCEL_BIT  (1U << 6)
#define FIFO_HDR_GYRO_BIT   (1U << 5)
#define FIFO_HDR_HIRES_BIT  (1U << 4)
#define FIFO_HDR_TMST_BIT   (1U << 3)

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

ICM_Sample_t g_sensor_averaged[ICM_TOTAL_SENSORS];
uint8_t g_sensor_average_status[ICM_TOTAL_SENSORS];

volatile uint32_t g_icm_parse_cyc_last = 0U;
volatile uint32_t g_icm_parse_cyc_max = 0U;
volatile uint32_t g_icm_parse_us_last = 0U;
volatile uint32_t g_icm_parse_us_max = 0U;

/*
 * Сборка знакового 20-битного значения.
 * Реализация сохранена из исходного проекта.
 */
static inline int32_t build20(uint8_t msb,
                             uint8_t lsb,
                             uint8_t nibble)
{
    int32_t raw = ((int32_t)(uint32_t)msb << 12) |
                  ((int32_t)(uint32_t)lsb << 4) |
                  (int32_t)(uint32_t)(nibble & 0x0FU);

    return (raw << 12) >> 12;
}

/*
 * raw_buf указывает на первый байт FIFO payload, после SPI command.
 * buf_len при штатном чтении равен ICM_FIFO_PAYLOAD_BYTES.
 *
 * samples[] остаётся raw-батчем; усреднение здесь не выполняется.
 */
void ICM_ParseFIFOBuffer(const uint8_t *raw_buf,
                         uint16_t buf_len,
                         ICM_SensorBatch_t *batch)
{
    uint16_t offset = 0U;
    uint8_t n = 0U;

    const uint8_t *pkt;
    uint8_t hdr;

    batch->count = 0U;

    while ((uint16_t)(offset + ICM_FIFO_PACKET_BYTES) <= buf_len)
    {
        pkt = &raw_buf[offset];
        hdr = pkt[0];

        /* Служебный MSG-пакет: данные не разбираем. */
        if ((hdr & FIFO_HDR_MSG_BIT) != 0U)
        {
            offset = (uint16_t)(offset + ICM_FIFO_PACKET_BYTES);
            continue;
        }

        if (((hdr & FIFO_HDR_HIRES_BIT) != 0U) &&
            (n < ICM_FIFO_POLL_PACKETS))
        {
            ICM_Sample_t *s = &batch->samples[n];

            /* Accel: Big Endian main words + HIRES nibbles. */
            s->accel_x = build20(pkt[1],
                                pkt[2],
                                (uint8_t)(pkt[17] >> 4U));

            s->accel_y = build20(pkt[3],
                                pkt[4],
                                (uint8_t)(pkt[18] >> 4U));

            s->accel_z = build20(pkt[5],
                                pkt[6],
                                (uint8_t)(pkt[19] >> 4U));

            /* Gyro: Big Endian main words + HIRES nibbles. */
            s->gyro_x = build20(pkt[7],
                               pkt[8],
                               (uint8_t)(pkt[17] & 0x0FU));

            s->gyro_y = build20(pkt[9],
                               pkt[10],
                               (uint8_t)(pkt[18] & 0x0FU));

            s->gyro_z = build20(pkt[11],
                               pkt[12],
                               (uint8_t)(pkt[19] & 0x0FU));

            /* Temperature: Big Endian int16_t. */
            s->temp_raw = (int16_t)(((uint16_t)pkt[13] << 8U) |
                                   (uint16_t)pkt[14]);

            /*
             * Timestamp сохраняется как исходное поле FIFO.
             * Для output Fs оно не используется.
             */
            if ((hdr & FIFO_HDR_TMST_BIT) != 0U)
            {
                s->timestamp = (uint16_t)(
                    ((uint16_t)pkt[15] << 8U) |
                    (uint16_t)pkt[16]);
            }
            else
            {
                s->timestamp = 0U;
            }

            n++;
        }

        offset = (uint16_t)(offset + ICM_FIFO_PACKET_BYTES);
    }

    batch->count = n;
}

/*
 * Только парсинг всех датчиков.
 * SPI6 получает payload из отдельного D3/SRAM4-буфера.
 */
void ICM_ParseAllFIFO(void)
{
    uint32_t start_cyc = DWT->CYCCNT;

    uint8_t b;
    uint8_t s;
    uint8_t id;

    for (b = 0U; b < ICM_SPI_BUS_COUNT; b++)
    {
        for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
        {
            id = (uint8_t)(b * ICM_SENSORS_PER_BUS + s);

            g_sensor_batches[id].sensor_id = id;

            if ((g_sensor_fault_mask & (1ULL << id)) != 0U)
            {
                memset(g_sensor_batches[id].samples,
                       0x00,
                       sizeof(g_sensor_batches[id].samples));

                g_sensor_batches[id].count = 0U;
            }
            else
            {
                const uint8_t *src = (b == 5U)
                    ? &g_fifo_data_spi6[s][1U]
                    : &g_fifo_data[b][s][1U];

                ICM_ParseFIFOBuffer(
                    src,
                    (uint16_t)(ICM_FIFO_DMA_BUF_SIZE - 1U),
                    &g_sensor_batches[id]);
            }
        }
    }

    uint32_t delta_cyc = DWT->CYCCNT - start_cyc;

    g_icm_parse_cyc_last = delta_cyc;

    if (delta_cyc > g_icm_parse_cyc_max)
    {
        g_icm_parse_cyc_max = delta_cyc;
    }

    uint32_t us = delta_cyc / (SystemCoreClock / 1000000UL);

    g_icm_parse_us_last = us;

    if (us > g_icm_parse_us_max)
    {
        g_icm_parse_us_max = us;
    }
}

/*
 * Симметричное округление:
 * ближайшее целое, точная половина округляется от нуля.
 *
 * В C знаковое целочисленное деление усекается к нулю.
 */
static int32_t div_round_s64(int64_t sum, uint32_t count)
{
    const int64_t half = (int64_t)count / 2;

    if (count == 0U)
    {
        return 0;
    }

    if (sum >= 0)
    {
        return (int32_t)((sum + half) / (int64_t)count);
    }

    return (int32_t)((sum - half) / (int64_t)count);
}

static int16_t div_round_temp(int32_t sum, uint32_t count)
{
    int32_t result = div_round_s64((int64_t)sum, count);

    if (result > INT16_MAX)
    {
        result = INT16_MAX;
    }

    if (result < INT16_MIN)
    {
        result = INT16_MIN;
    }

    return (int16_t)result;
}

/*
 * Один output sample на датчик.
 *
 * Валиден только полный батч размером ICM_DECIMATION_FACTOR.
 * Raw samples[] и batch->count не изменяются.
 *
 * Неполный/пустой/fault-батч даёт нулевой output и отдельный статус.
 * Timestamp результата берётся из последнего raw sample окна.
 */
void ICM_AverageAllBatches(void)
{
    uint8_t id;

    for (id = 0U; id < ICM_TOTAL_SENSORS; id++)
    {
        ICM_SensorBatch_t *batch = &g_sensor_batches[id];
        ICM_Sample_t *out = &g_sensor_averaged[id];

        uint8_t n;

        int64_t sum_ax = 0;
        int64_t sum_ay = 0;
        int64_t sum_az = 0;

        int64_t sum_gx = 0;
        int64_t sum_gy = 0;
        int64_t sum_gz = 0;

        int32_t sum_temp = 0;

        memset(out, 0, sizeof(*out));

        if ((g_sensor_fault_mask & (1ULL << id)) != 0U)
        {
            g_sensor_average_status[id] = ICM_AVG_STATUS_FAULT;
            continue;
        }

        if (batch->count == 0U)
        {
            g_sensor_average_status[id] = ICM_AVG_STATUS_NO_DATA;
            continue;
        }

        if (batch->count != ICM_DECIMATION_FACTOR)
        {
            g_sensor_average_status[id] = ICM_AVG_STATUS_SHORT_BATCH;
            continue;
        }

        for (n = 0U; n < batch->count; n++)
        {
            const ICM_Sample_t *sample = &batch->samples[n];

            sum_ax += sample->accel_x;
            sum_ay += sample->accel_y;
            sum_az += sample->accel_z;

            sum_gx += sample->gyro_x;
            sum_gy += sample->gyro_y;
            sum_gz += sample->gyro_z;

            sum_temp += sample->temp_raw;
        }

        out->accel_x = div_round_s64(sum_ax, batch->count);
        out->accel_y = div_round_s64(sum_ay, batch->count);
        out->accel_z = div_round_s64(sum_az, batch->count);

        out->gyro_x = div_round_s64(sum_gx, batch->count);
        out->gyro_y = div_round_s64(sum_gy, batch->count);
        out->gyro_z = div_round_s64(sum_gz, batch->count);

        out->temp_raw = div_round_temp(sum_temp, batch->count);

        /*
         * Timestamp последнего raw sample окна.
         * Delta timestamp не усредняется арифметически.
         */
        out->timestamp = batch->samples[batch->count - 1U].timestamp;

        g_sensor_average_status[id] = ICM_AVG_STATUS_OK;
    }
}
