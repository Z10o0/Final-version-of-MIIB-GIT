/**
 * @file  icm45686_data.c
 *
 * ICM-45686 FIFO 20-byte HIRES parser
 * Строго по даташиту DS-000577 + официальному SDK inv_imu_regmap_le.h
 *
 * Датчик работает в LITTLE ENDIAN (SREGDATAENDIANSEL=0, default).
 * inv_imu_regmap_le.h в репозитории подтверждает: LE-режим.
 *
 * Layout 20-byte пакета в Little Endian:
 *  Byte  0:  Header
 *  Byte  1:  Ax[11:4]   LSB
 *  Byte  2:  Ax[19:12]  MSB
 *  Byte  3:  Ay[11:4]   LSB
 *  Byte  4:  Ay[19:12]  MSB
 *  Byte  5:  Az[11:4]   LSB
 *  Byte  6:  Az[19:12]  MSB
 *  Byte  7:  Gx[11:4]   LSB
 *  Byte  8:  Gx[19:12]  MSB
 *  Byte  9:  Gy[11:4]   LSB
 *  Byte 10:  Gy[19:12]  MSB
 *  Byte 11:  Gz[11:4]   LSB
 *  Byte 12:  Gz[19:12]  MSB
 *  Byte 13:  Temp[7:0]  LSB
 *  Byte 14:  Temp[15:8] MSB
 *  Byte 15:  Timestamp[7:0]  LSB
 *  Byte 16:  Timestamp[15:8] MSB
 *  Byte 17:  Ax[3:0](hi nibble) | Gx[3:0](lo nibble)
 *  Byte 18:  Ay[3:0](hi nibble) | Gy[3:0](lo nibble)
 *  Byte 19:  Az[3:0](hi nibble) | Gz[3:0](lo nibble)
 *
 * Сборка 20-bit знакового:
 *   raw = (MSB_byte << 12) | (LSB_byte << 4) | nibble
 *   sign-extend: (raw << 12) >> 12
 */

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

#define FIFO_HDR_MSG_BIT    (1U << 7)
#define FIFO_HDR_ACCEL_BIT  (1U << 6)
#define FIFO_HDR_GYRO_BIT   (1U << 5)
#define FIFO_HDR_HIRES_BIT  (1U << 4)
#define FIFO_HDR_TMST_BIT   (1U << 3)

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/**
 * 20-bit sign-extend из трёх компонентов.
 * msb    = байт [19:12]  (старший байт, ВТОРОЙ в пакете LE)
 * lsb    = байт [11:4]   (младший байт, ПЕРВЫЙ в пакете LE)
 * nibble = биты [3:0]    (из byte17/18/19, уже замаскирован 0x0F)
 */
static inline int32_t build20(uint8_t msb, uint8_t lsb, uint8_t nibble)
{
    int32_t raw = ((int32_t)(uint32_t)msb    << 12) |
                  ((int32_t)(uint32_t)lsb    <<  4) |
                   (int32_t)(uint32_t)(nibble & 0x0FU);
    return (raw << 12) >> 12;
}

void ICM_ParseFIFOBuffer(const uint8_t    *raw_buf,
                         uint16_t          buf_len,
                         ICM_SensorBatch_t *batch)
{
    uint16_t       offset = 0U;
    uint8_t        n      = 0U;
    const uint8_t *pkt;
    uint8_t        hdr;

    batch->count = 0U;

    while ((uint16_t)(offset + 20U) <= buf_len)
    {
        pkt = &raw_buf[offset];
        hdr = pkt[0];

        /* MSG-пакет: маркер конца FIFO, пропускаем */
        if ((hdr & FIFO_HDR_MSG_BIT) != 0U)
        {
            offset += 20U;
            continue;
        }

        /* HIRES пакет */
        if (((hdr & FIFO_HDR_HIRES_BIT) != 0U) && (n < ICM_FIFO_POLL_PACKETS))
        {
            ICM_Sample_t *s = &batch->samples[n];

            /*
             * Little Endian: LSB-байт идёт ПЕРВЫМ, MSB ВТОРЫМ.
             *
             * Акселерометр:
             *   pkt[1] = Ax[11:4] (LSB), pkt[2] = Ax[19:12] (MSB)
             *   pkt[3] = Ay[11:4] (LSB), pkt[4] = Ay[19:12] (MSB)
             *   pkt[5] = Az[11:4] (LSB), pkt[6] = Az[19:12] (MSB)
             *
             * Гироскоп:
             *   pkt[7]  = Gx[11:4] (LSB), pkt[8]  = Gx[19:12] (MSB)
             *   pkt[9]  = Gy[11:4] (LSB), pkt[10] = Gy[19:12] (MSB)
             *   pkt[11] = Gz[11:4] (LSB), pkt[12] = Gz[19:12] (MSB)
             *
             * Nibbles (не зависят от эндианности):
             *   pkt[17] hi-nibble = Ax[3:0], lo-nibble = Gx[3:0]
             *   pkt[18] hi-nibble = Ay[3:0], lo-nibble = Gy[3:0]
             *   pkt[19] hi-nibble = Az[3:0], lo-nibble = Gz[3:0]
             */
            s->accel_x = build20(pkt[2],  pkt[1],  pkt[17] >> 4U);
            s->accel_y = build20(pkt[4],  pkt[3],  pkt[18] >> 4U);
            s->accel_z = build20(pkt[6],  pkt[5],  pkt[19] >> 4U);

            s->gyro_x  = build20(pkt[8],  pkt[7],  pkt[17] & 0x0FU);
            s->gyro_y  = build20(pkt[10], pkt[9],  pkt[18] & 0x0FU);
            s->gyro_z  = build20(pkt[12], pkt[11], pkt[19] & 0x0FU);

            /*
             * Температура: Little Endian, 2 байта
             *   pkt[13] = Temp[7:0]  (LSB)
             *   pkt[14] = Temp[15:8] (MSB)
             * Формула: T[°C] = temp_raw / 128.0f + 25.0f
             */
            s->temp_raw = (int16_t)(((uint16_t)pkt[14] << 8U) |
                                     (uint16_t)pkt[13]);

            /*
             * Timestamp: Little Endian
             *   pkt[15] = Timestamp[7:0]  (LSB)
             *   pkt[16] = Timestamp[15:8] (MSB)
             */
            if ((hdr & FIFO_HDR_TMST_BIT) != 0U)
            {
                s->timestamp = (uint16_t)(((uint16_t)pkt[16] << 8U) |
                                           (uint16_t)pkt[15]);
            }
            else
            {
                s->timestamp = 0U;
            }

            n++;
        }

        offset += 20U;
    }

    batch->count = n;
}

void ICM_ParseAllFIFO(void)
{
    uint8_t b, s, id;

    for (b = 0U; b < ICM_SPI_BUS_COUNT; b++)
    {
        for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
        {
            id = (uint8_t)(b * ICM_SENSORS_PER_BUS + s);
            g_sensor_batches[id].sensor_id = id;

            if ((g_sensor_fault_mask & (1UL << id)) != 0U)
            {
                memset(g_sensor_batches[id].samples, 0x00,
                       sizeof(g_sensor_batches[id].samples));
                g_sensor_batches[id].count = 0U;
            }
            else
            {
                /* Byte[0] = SPI address byte, данные с byte[1] */
                ICM_ParseFIFOBuffer(
                    &g_fifo_data[b][s][1U],
                    (uint16_t)(ICM_FIFO_DMA_BUF_SIZE - 1U),
                    &g_sensor_batches[id]);
            }
        }
    }
}
