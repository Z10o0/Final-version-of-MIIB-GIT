/**
 * @file icm45686_data.c
 *
 * ICM-45686 FIFO 20-byte HIRES parser.
 * Разбор соответствует DS-000577 §6.1 и SREGDATAENDIANSEL=1
 * (Big Endian).
 *
 * ВАЖНО: датчик настроен в Big Endian
 * (IREG 0xA267 bit1 = 1).
 * В Big Endian MSB-байт идёт первым в паре.
 *
 * Layout 20-byte пакета в Big Endian:
 *   Byte 0:  Header
 *   Byte 1:  Ax[19:12] MSB
 *   Byte 2:  Ax[11:4]  LSB
 *   Byte 3:  Ay[19:12] MSB
 *   Byte 4:  Ay[11:4]  LSB
 *   Byte 5:  Az[19:12] MSB
 *   Byte 6:  Az[11:4]  LSB
 *   Byte 7:  Gx[19:12] MSB
 *   Byte 8:  Gx[11:4]  LSB
 *   Byte 9:  Gy[19:12] MSB
 *   Byte 10: Gy[11:4]  LSB
 *   Byte 11: Gz[19:12] MSB
 *   Byte 12: Gz[11:4]  LSB
 *   Byte 13: Temp[15:8]      MSB
 *   Byte 14: Temp[7:0]       LSB
 *   Byte 15: Timestamp[15:8] MSB
 *   Byte 16: Timestamp[7:0]  LSB
 *   Byte 17: Ax[3:0](hi nibble) | Gx[3:0](lo nibble)
 *   Byte 18: Ay[3:0](hi nibble) | Gy[3:0](lo nibble)
 *   Byte 19: Az[3:0](hi nibble) | Gz[3:0](lo nibble)
 *
 * Сборка 20-bit знакового значения:
 *   raw = (MSB_byte << 12) | (LSB_byte << 4) | nibble
 *   sign-extend: (raw << 12) >> 12
 *
 * ODR датчика 3200 Гц.
 * TIM6 запускает чтение FIFO на 400 Гц.
 * За батч считываются 8 HIRES-пакетов по 20 байт.
 * Парсер сохраняет восемь raw samples, после чего
 * ICM_AverageAllBatches() формирует один output sample 400 Гц
 * на каждый датчик.
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

ICM_SensorBatch_t
    g_sensor_batches[ICM_TOTAL_SENSORS];

ICM_Sample_t
    g_sensor_averaged[ICM_TOTAL_SENSORS];

uint8_t
    g_sensor_average_status[ICM_TOTAL_SENSORS];

/**
 * build20 — сборка 20-bit знакового числа из трёх компонентов.
 *
 * @param msb
 * Байт [19:12] — старший байт, первый в BE-пакете.
 *
 * @param lsb
 * Байт [11:4] — младший байт, второй в BE-пакете.
 *
 * @param nibble
 * Биты [3:0] из byte17/18/19, уже сдвинутые или замаскированные.
 */
static inline int32_t build20(uint8_t msb,
                              uint8_t lsb,
                              uint8_t nibble)
{
    int32_t raw;

    raw = ((int32_t)(uint32_t)msb << 12) |
          ((int32_t)(uint32_t)lsb << 4)  |
          (int32_t)(uint32_t)(nibble & 0x0FU);

    /* Sign extension с позиции bit19. */
    return (raw << 12) >> 12;
}

/**
 * ICM_ParseFIFOBuffer — разобрать все валидные HIRES-пакеты
 * из непрерывного FIFO-буфера одного датчика.
 *
 * @param raw_buf
 * Указатель на первый байт payload после cmd/addr байта.
 *
 * @param buf_len
 * Длина payload в байтах.
 * Для полного батча: 8 пакетов × 20 байт = 160 байт.
 *
 * @param batch
 * Структура назначения: samples[] и count.
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

        /*
         * MSG-пакет: маркер конца FIFO или служебный пакет.
         * Данные такого пакета не сохраняются.
         */
        if ((hdr & FIFO_HDR_MSG_BIT) != 0U)
        {
            offset =
                (uint16_t)(offset + ICM_FIFO_PACKET_BYTES);
            continue;
        }

        /*
         * Сохраняются только HIRES-пакеты.
         * Количество результатов ограничено размером batch->samples[].
         */
        if (((hdr & FIFO_HDR_HIRES_BIT) != 0U) &&
            (n < ICM_FIFO_POLL_PACKETS))
        {
            ICM_Sample_t *sample = &batch->samples[n];

            /*
             * Big Endian: MSB-байт идёт первым.
             *
             * Акселерометр:
             *   pkt[1] = Ax MSB, pkt[2]  = Ax LSB
             *   pkt[3] = Ay MSB, pkt[4]  = Ay LSB
             *   pkt[5] = Az MSB, pkt[6]  = Az LSB
             *
             * Гироскоп:
             *   pkt[7]  = Gx MSB, pkt[8]  = Gx LSB
             *   pkt[9]  = Gy MSB, pkt[10] = Gy LSB
             *   pkt[11] = Gz MSB, pkt[12] = Gz LSB
             *
             * Nibbles:
             *   pkt[17]: hi=Ax[3:0], lo=Gx[3:0]
             *   pkt[18]: hi=Ay[3:0], lo=Gy[3:0]
             *   pkt[19]: hi=Az[3:0], lo=Gz[3:0]
             */
            sample->accel_x =
                build20(pkt[1],
                        pkt[2],
                        (uint8_t)(pkt[17] >> 4U));

            sample->accel_y =
                build20(pkt[3],
                        pkt[4],
                        (uint8_t)(pkt[18] >> 4U));

            sample->accel_z =
                build20(pkt[5],
                        pkt[6],
                        (uint8_t)(pkt[19] >> 4U));

            sample->gyro_x =
                build20(pkt[7],
                        pkt[8],
                        (uint8_t)(pkt[17] & 0x0FU));

            sample->gyro_y =
                build20(pkt[9],
                        pkt[10],
                        (uint8_t)(pkt[18] & 0x0FU));

            sample->gyro_z =
                build20(pkt[11],
                        pkt[12],
                        (uint8_t)(pkt[19] & 0x0FU));

            /*
             * Температура: Big Endian, 2 байта.
             * T[°C] = temp_raw / 128.0f + 25.0f.
             */
            sample->temp_raw =
                (int16_t)(((uint16_t)pkt[13] << 8U) |
                           (uint16_t)pkt[14]);

            /*
             * Timestamp: Big Endian.
             * Разрешение 1 мкс/LSB при TMST_RESOL=0.
             */
            if ((hdr & FIFO_HDR_TMST_BIT) != 0U)
            {
                sample->timestamp =
                    (uint16_t)(((uint16_t)pkt[15] << 8U) |
                               (uint16_t)pkt[16]);
            }
            else
            {
                sample->timestamp = 0U;
            }

            n++;
        }

        offset =
            (uint16_t)(offset + ICM_FIFO_PACKET_BYTES);
    }

    batch->count = n;
}

/**
 * ICM_ParseAllFIFO — разобрать FIFO всех 36 датчиков.
 *
 * Если датчик помечен неисправным в g_sensor_fault_mask, его raw batch
 * обнуляется и count устанавливается в ноль.
 *
 * Для исправного датчика разбирается payload длиной
 * ICM_FIFO_DMA_BUF_SIZE - 1 байт, начиная с элемента [1].
 * Элемент [0] является командным байтом SPI-транзакции.
 */
volatile uint32_t g_icm_parse_cyc_last = 0U;
volatile uint32_t g_icm_parse_cyc_max  = 0U;
volatile uint32_t g_icm_parse_us_last  = 0U;
volatile uint32_t g_icm_parse_us_max   = 0U;

void ICM_ParseAllFIFO(void)
{
    uint32_t start_cyc = DWT->CYCCNT;

    uint8_t bus_idx;
    uint8_t sensor_idx;
    uint8_t id;

    for (bus_idx = 0U;
         bus_idx < ICM_SPI_BUS_COUNT;
         bus_idx++)
    {
        for (sensor_idx = 0U;
             sensor_idx < ICM_SENSORS_PER_BUS;
             sensor_idx++)
        {
            const uint8_t *src;

            id = (uint8_t)(
                bus_idx * ICM_SENSORS_PER_BUS + sensor_idx);

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
                /*
                 * SPI6 использует отдельные буферы в D3/SRAM4,
                 * доступной BDMA.
                 */
                src = (bus_idx == 5U)
                    ? &g_fifo_data_spi6[sensor_idx][1U]
                    : &g_fifo_data[bus_idx][sensor_idx][1U];

                ICM_ParseFIFOBuffer(
                    src,
                    (uint16_t)(ICM_FIFO_DMA_BUF_SIZE - 1U),
                    &g_sensor_batches[id]);
            }
        }
    }

    {
        uint32_t delta_cyc =
            DWT->CYCCNT - start_cyc;

        uint32_t us =
            delta_cyc / (SystemCoreClock / 1000000UL);

        g_icm_parse_cyc_last = delta_cyc;

        if (delta_cyc > g_icm_parse_cyc_max)
        {
            g_icm_parse_cyc_max = delta_cyc;
        }

        g_icm_parse_us_last = us;

        if (us > g_icm_parse_us_max)
        {
            g_icm_parse_us_max = us;
        }
    }
}

/**
 * Деление знаковой 64-битной суммы с симметричным округлением.
 *
 * Для положительного значения прибавляется count / 2.
 * Для отрицательного значения count / 2 вычитается.
 */
static int32_t div_round_s64(int64_t sum,
                             uint32_t count)
{
    const int64_t half = (int64_t)count / 2;

    if (count == 0U)
    {
        return 0;
    }

    if (sum >= 0)
    {
        return (int32_t)(
            (sum + half) / (int64_t)count);
    }

    return (int32_t)(
        (sum - half) / (int64_t)count);
}

/**
 * Усреднение температуры с симметричным округлением
 * и насыщением до диапазона int16_t.
 */
static int16_t div_round_temp(int32_t sum,
                              uint32_t count)
{
    int32_t result =
        div_round_s64((int64_t)sum, count);

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

/**
 * ICM_AverageAllBatches — сформировать один output sample 400 Гц
 * для каждого датчика.
 *
 * Усредняются accel, gyro и temperature.
 *
 * Timestamp не усредняется: в output записывается timestamp последнего
 * raw sample полного окна.
 *
 * Неполные батчи не усредняются. Результат остаётся нулевым, а статус
 * устанавливается в ICM_AVG_STATUS_SHORT_BATCH.
 */
void ICM_AverageAllBatches(void)
{
    uint8_t id;

    for (id = 0U;
         id < ICM_TOTAL_SENSORS;
         id++)
    {
        ICM_SensorBatch_t *batch =
            &g_sensor_batches[id];

        ICM_Sample_t *out =
            &g_sensor_averaged[id];

        uint8_t sample_idx;

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
            g_sensor_average_status[id] =
                ICM_AVG_STATUS_FAULT;

            continue;
        }

        if (batch->count == 0U)
        {
            g_sensor_average_status[id] =
                ICM_AVG_STATUS_NO_DATA;

            continue;
        }

        if (batch->count != ICM_DECIMATION_FACTOR)
        {
            g_sensor_average_status[id] =
                ICM_AVG_STATUS_SHORT_BATCH;

            continue;
        }

        for (sample_idx = 0U;
             sample_idx < batch->count;
             sample_idx++)
        {
            const ICM_Sample_t *sample =
                &batch->samples[sample_idx];

            sum_ax += sample->accel_x;
            sum_ay += sample->accel_y;
            sum_az += sample->accel_z;

            sum_gx += sample->gyro_x;
            sum_gy += sample->gyro_y;
            sum_gz += sample->gyro_z;

            sum_temp += sample->temp_raw;
        }

        out->accel_x =
            div_round_s64(sum_ax, batch->count);

        out->accel_y =
            div_round_s64(sum_ay, batch->count);

        out->accel_z =
            div_round_s64(sum_az, batch->count);

        out->gyro_x =
            div_round_s64(sum_gx, batch->count);

        out->gyro_y =
            div_round_s64(sum_gy, batch->count);

        out->gyro_z =
            div_round_s64(sum_gz, batch->count);

        out->temp_raw =
            div_round_temp(sum_temp, batch->count);

        /*
         * Timestamp результата относится к последнему
         * raw sample окна.
         */
        out->timestamp =
            batch->samples[batch->count - 1U].timestamp;

        g_sensor_average_status[id] =
            ICM_AVG_STATUS_OK;
    }
}
