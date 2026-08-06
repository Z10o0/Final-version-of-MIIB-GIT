/**
 * @file  icm45686_data.c
 *
 * 20-байтный HIRES FIFO пакет ICM-45686
 * Строго по даташиту DS-000577, таблица из User Guide AN-000478.
 *
 * Layout (Big-Endian — Byte H первый):
 *  Byte  0:  Header
 *  Byte  1:  Ax H  [19:12]
 *  Byte  2:  Ax L  [11:4]
 *  Byte  3:  Ay H
 *  Byte  4:  Ay L
 *  Byte  5:  Az H
 *  Byte  6:  Az L
 *  Byte  7:  Gx H
 *  Byte  8:  Gx L
 *  Byte  9:  Gy H
 *  Byte 10:  Gy L
 *  Byte 11:  Gz H
 *  Byte 12:  Gz L
 *  Byte 13:  Temp H
 *  Byte 14:  Temp L
 *  Byte 15:  Timestamp H
 *  Byte 16:  Timestamp L
 *  Byte 17:  Ax LSB [3:0] hi-nibble | Gx LSB [3:0] lo-nibble
 *  Byte 18:  Ay LSB [3:0] hi-nibble | Gy LSB [3:0] lo-nibble
 *  Byte 19:  Az LSB [3:0] hi-nibble | Gz LSB [3:0] lo-nibble
 *
 * Сборка 20-bit знакового числа:
 *   raw20 = (H << 12) | (L << 4) | nibble[3:0]
 *   sign-extend через арифметический сдвиг: (raw20 << 12) >> 12
 */

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

#ifndef ICM45686_FIFO_HEADER_MSG_BIT
#define ICM45686_FIFO_HEADER_MSG_BIT    (1U << 7)
#endif
#ifndef ICM45686_FIFO_HEADER_ACCEL_BIT
#define ICM45686_FIFO_HEADER_ACCEL_BIT  (1U << 6)
#endif
#ifndef ICM45686_FIFO_HEADER_GYRO_BIT
#define ICM45686_FIFO_HEADER_GYRO_BIT   (1U << 5)
#endif
#ifndef ICM45686_FIFO_HEADER_TMST_BIT
#define ICM45686_FIFO_HEADER_TMST_BIT   (1U << 3)
#endif

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/**
 * Сборка 20-битного знакового значения из трёх байт:
 *   hi    = байт H [19:12]
 *   lo    = байт L [11:4]
 *   nibble = [3:0] из nibble-байта (уже замаскированный, 0x0F)
 */
static inline int32_t icm_build20(uint8_t hi, uint8_t lo, uint8_t nibble)
{
    /* Собираем 20-битное число в биты [19:0] */
    int32_t raw = ((int32_t)(uint32_t)hi    << 12) |
                  ((int32_t)(uint32_t)lo    <<  4) |
                   (int32_t)(uint32_t)(nibble & 0x0FU);
    /* Арифметический sign-extend: растягиваем бит 19 на биты [31:20] */
    return (raw << 12) >> 12;
}

void ICM_ParseFIFOBuffer(const uint8_t    *raw_buf,
                         uint16_t          buf_len,
                         ICM_SensorBatch_t *batch)
{
    uint16_t       offset  = 0U;
    uint8_t        pkt_cnt = 0U;
    const uint8_t *pkt;
    uint8_t        header;

    batch->count = 0U;

    while ((offset + 20U) <= buf_len)
    {
        pkt    = &raw_buf[offset];
        header = pkt[0];

        /* Пропускаем MSG-пакеты (пустые маркеры конца FIFO) */
        if ((header & ICM45686_FIFO_HEADER_MSG_BIT) != 0U)
        {
            offset += 20U;
            continue;
        }

        if (((header & ICM45686_FIFO_HEADER_ACCEL_BIT) != 0U) ||
            ((header & ICM45686_FIFO_HEADER_GYRO_BIT)  != 0U))
        {
            if (pkt_cnt < ICM_FIFO_POLL_PACKETS)
            {
                ICM_Sample_t *smp = &batch->samples[pkt_cnt];

                /*
                 * ACCEL: Byte1=AxH, Byte2=AxL, ..., Byte6=AzL
                 * Nibble-байты 17,18,19:
                 *   Byte 17 hi-nibble = Ax[3:0]
                 *   Byte 18 hi-nibble = Ay[3:0]
                 *   Byte 19 hi-nibble = Az[3:0]
                 */
                smp->accel_x = icm_build20(pkt[1],  pkt[2],  pkt[17] >> 4U);
                smp->accel_y = icm_build20(pkt[3],  pkt[4],  pkt[18] >> 4U);
                smp->accel_z = icm_build20(pkt[5],  pkt[6],  pkt[19] >> 4U);

                /*
                 * GYRO: Byte7=GxH, Byte8=GxL, ..., Byte12=GzL
                 * Nibble-байты 17,18,19:
                 *   Byte 17 lo-nibble = Gx[3:0]
                 *   Byte 18 lo-nibble = Gy[3:0]
                 *   Byte 19 lo-nibble = Gz[3:0]
                 */
                smp->gyro_x  = icm_build20(pkt[7],  pkt[8],  pkt[17] & 0x0FU);
                smp->gyro_y  = icm_build20(pkt[9],  pkt[10], pkt[18] & 0x0FU);
                smp->gyro_z  = icm_build20(pkt[11], pkt[12], pkt[19] & 0x0FU);

                /*
                 * TEMP: Byte13=H, Byte14=L (Big-Endian)
                 * Формула перевода: °C = temp_raw / 128.0f + 25.0f
                 */
                smp->temp_raw = (int16_t)(((uint16_t)pkt[13] << 8U) |
                                           (uint16_t)pkt[14]);

                /*
                 * TIMESTAMP: Byte15=H, Byte16=L (Big-Endian)
                 */
                if ((header & ICM45686_FIFO_HEADER_TMST_BIT) != 0U)
                {
                    smp->timestamp = (uint16_t)(((uint16_t)pkt[15] << 8U) |
                                                 (uint16_t)pkt[16]);
                }
                else
                {
                    smp->timestamp = 0U;
                }

                pkt_cnt++;
            }
        }

        offset += 20U;
    }

    batch->count = pkt_cnt;
}

void ICM_ParseAllFIFO(void)
{
    uint8_t  b, s, sensor_id;
    uint32_t fault_mask = g_sensor_fault_mask;

    const uint16_t data_offset = 1U;
    const uint16_t data_len    = (uint16_t)(ICM_FIFO_DMA_BUF_SIZE - data_offset);

    for (b = 0U; b < ICM_SPI_BUS_COUNT; b++)
    {
        for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
        {
            sensor_id = (uint8_t)(b * ICM_SENSORS_PER_BUS + s);
            g_sensor_batches[sensor_id].sensor_id = sensor_id;

            if ((fault_mask & (1UL << sensor_id)) != 0U)
            {
                memset(g_sensor_batches[sensor_id].samples, 0x00,
                       sizeof(g_sensor_batches[sensor_id].samples));
                g_sensor_batches[sensor_id].count = 0U;
            }
            else
            {
                ICM_ParseFIFOBuffer(
                    &g_fifo_data[b][s][data_offset],
                    data_len,
                    &g_sensor_batches[sensor_id]);
            }
        }
    }
}
