/**
 * @file    icm45686_data.c
 * @brief   Разбор 20-байтных HIRES FIFO-пакетов ICM-45686.
 *
 * ТОЧНЫЙ формат (Big-Endian, из DS-000577 + SlimeVR reference impl):
 *
 *  Byte  0      Header
 *  Byte  1      Accel X [19:12]  MSB
 *  Byte  2      Accel X [11:4]   LSB
 *  Byte  3      Accel Y [19:12]
 *  Byte  4      Accel Y [11:4]
 *  Byte  5      Accel Z [19:12]
 *  Byte  6      Accel Z [11:4]
 *  Byte  7      Gyro  X [19:12]
 *  Byte  8      Gyro  X [11:4]
 *  Byte  9      Gyro  Y [19:12]
 *  Byte 10      Gyro  Y [11:4]
 *  Byte 11      Gyro  Z [19:12]
 *  Byte 12      Gyro  Z [11:4]
 *  Byte 13      Temp  (1 byte, MSB)
 *  Byte 14      Timestamp LSB
 *  Byte 15      Timestamp MSB
 *  Byte 16      (reserved/ext)
 *  Byte 17      Accel X[3:0] hi-nibble | Gyro X[3:0] lo-nibble
 *  Byte 18      Accel Y[3:0] hi-nibble | Gyro Y[3:0] lo-nibble
 *  Byte 19      Accel Z[3:0] hi-nibble | Gyro Z[3:0] lo-nibble
 *
 * Сборка 20-bit знакового int32_t (через сдвиг в старшие биты + приведение):
 *   accel_x = (int32_t)( (pkt[1]<<24) | (pkt[2]<<16) | ((pkt[17]&0xF0)<<8) )
 *   gyro_x  = (int32_t)( (pkt[7]<<24) | (pkt[8]<<16) | ((pkt[17]&0x0F)<<12) )
 *   Sign-extend происходит автоматически через (int32_t) — бит 31 = знаковый.
 *   Для получения "нормального" 20-бит числа сдвинь вправо на 12:
 *   val20 = accel_x >> 12
 */

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

#ifndef ICM45686_FIFO_HEADER_TMST_BIT
#define ICM45686_FIFO_HEADER_TMST_BIT   (1U << 3)
#endif

/* Маска невалидных данных (датчик шлёт 0x8000 если данные недоступны) */
static const uint8_t ICM_INVALID_DATA[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

void ICM_ParseFIFOBuffer(const uint8_t    *raw_buf,
                         uint16_t          buf_len,
                         ICM_SensorBatch_t *batch)
{
    uint16_t       offset  = 0U;
    uint8_t        pkt_cnt = 0U;
    uint8_t        header;
    const uint8_t *pkt;

    batch->count = 0U;

    while ((offset + 20U) <= buf_len)
    {
        pkt    = &raw_buf[offset];
        header = pkt[0];

        /* Пропускаем пустые MSG-пакеты */
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
                memset(smp, 0x00, sizeof(ICM_Sample_t));

                /* ============================================================
                 * ACCEL: Big-Endian, 20-bit
                 * Сборка: сдвиг MSB в бит31, LSB в бит23, nibble в бит[15:12]
                 * Байты 17,18,19: hi-nibble = accel X/Y/Z [3:0]
                 * Проверка валидности: байты 1-6 не должны быть 0x80,0x00,...
                 * ============================================================ */
                if (memcmp(&pkt[1], ICM_INVALID_DATA, 6U) != 0)
                {
                    smp->accel_x = (int32_t)(
                        ((uint32_t)pkt[1]  << 24U) |
                        ((uint32_t)pkt[2]  << 16U) |
                        (((uint32_t)pkt[17] & 0xF0U) << 8U)
                    );
                    smp->accel_y = (int32_t)(
                        ((uint32_t)pkt[3]  << 24U) |
                        ((uint32_t)pkt[4]  << 16U) |
                        (((uint32_t)pkt[18] & 0xF0U) << 8U)
                    );
                    smp->accel_z = (int32_t)(
                        ((uint32_t)pkt[5]  << 24U) |
                        ((uint32_t)pkt[6]  << 16U) |
                        (((uint32_t)pkt[19] & 0xF0U) << 8U)
                    );
                }

                /* ============================================================
                 * GYRO: Big-Endian, 20-bit
                 * Байты 17,18,19: lo-nibble = gyro X/Y/Z [3:0]
                 * lo-nibble сдвигается на 12 (бит[15:12] в 32-бит слове)
                 * ============================================================ */
                if (memcmp(&pkt[7], ICM_INVALID_DATA, 6U) != 0)
                {
                    smp->gyro_x = (int32_t)(
                        ((uint32_t)pkt[7]  << 24U) |
                        ((uint32_t)pkt[8]  << 16U) |
                        (((uint32_t)pkt[17] & 0x0FU) << 12U)
                    );
                    smp->gyro_y = (int32_t)(
                        ((uint32_t)pkt[9]  << 24U) |
                        ((uint32_t)pkt[10] << 16U) |
                        (((uint32_t)pkt[18] & 0x0FU) << 12U)
                    );
                    smp->gyro_z = (int32_t)(
                        ((uint32_t)pkt[11] << 24U) |
                        ((uint32_t)pkt[12] << 16U) |
                        (((uint32_t)pkt[19] & 0x0FU) << 12U)
                    );
                }

                /* ============================================================
                 * ТЕМПЕРАТУРА: байт 13, 1 байт MSB
                 * Формула: temp_c = (int8_t)pkt[13] / 2.0f + 25.0f
                 * Сохраняем сырой байт как int8_t
                 * ============================================================ */
                smp->temp_raw = (int16_t)(int8_t)pkt[13];

                /* ============================================================
                 * TIMESTAMP: Little-Endian, байты 14 (LSB) и 15 (MSB)
                 * ============================================================ */
                if ((header & ICM45686_FIFO_HEADER_TMST_BIT) != 0U)
                {
                    smp->timestamp = (uint16_t)(
                        ((uint16_t)pkt[15] << 8U) | (uint16_t)pkt[14]
                    );
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
    uint8_t  b, s;
    uint8_t  sensor_id;
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
