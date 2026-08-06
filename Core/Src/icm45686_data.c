/**
 * @file  icm45686_data.c
 *
 * ICM-45686 FIFO 20-byte High-Resolution Packet Parser
 * Строго по даташиту DS-000577 Rev 1.0
 *
 * ВАЖНО: Устройство работает в Little Endian режиме по умолчанию
 * (SREGDATAENDIANSEL=0 после reset). Либо нужно включить Big Endian
 * записью SREGDATAENDIANSEL=1 в IREG IPREGTOP1 (addr 0xA267).
 *
 * 20-byte packet layout (Big Endian mode, как в даташите):
 *  Byte  0:  Header
 *  Byte  1:  Ax[19:12]  (MSB)
 *  Byte  2:  Ax[11:4]
 *  Byte  3:  Ay[19:12]
 *  Byte  4:  Ay[11:4]
 *  Byte  5:  Az[19:12]
 *  Byte  6:  Az[11:4]
 *  Byte  7:  Gx[19:12]
 *  Byte  8:  Gx[11:4]
 *  Byte  9:  Gy[19:12]
 *  Byte 10:  Gy[11:4]
 *  Byte 11:  Gz[19:12]
 *  Byte 12:  Gz[11:4]
 *  Byte 13:  Temp[15:8]  (MSB)
 *  Byte 14:  Temp[7:0]
 *  Byte 15:  Timestamp[15:8]  (MSB)
 *  Byte 16:  Timestamp[7:0]
 *  Byte 17:  Ax[3:0](hi) | Gx[3:0](lo)
 *  Byte 18:  Ay[3:0](hi) | Gy[3:0](lo)
 *  Byte 19:  Az[3:0](hi) | Gz[3:0](lo)
 *
 * 20-bit sign-extend: raw = (H<<12)|(L<<4)|nibble → (raw<<12)>>12
 *
 * Температура в FIFO (20-byte пакет): int16_t, формула:
 *   T[°C] = FIFO_TEMP / 128.0f + 25.0f   <- для 16-bit FIFO temp
 *   (NB: 8-bit FIFO temp в 8/16-byte пакетах: T = TEMP/2 + 25)
 */

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

/* Header bits */
#define FIFO_HDR_MSG_BIT     (1U << 7)
#define FIFO_HDR_ACCEL_BIT   (1U << 6)
#define FIFO_HDR_GYRO_BIT    (1U << 5)
#define FIFO_HDR_HIRES_BIT   (1U << 4)
#define FIFO_HDR_TMST_BIT    (1U << 3)

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/**
 * Собирает 20-битное знаковое число и делает sign-extend до int32_t.
 * hi = байт [19:12], lo = байт [11:4], nibble = [3:0] (уже замаскирован)
 */
static inline int32_t build20(uint8_t hi, uint8_t lo, uint8_t nibble)
{
    int32_t raw = ((int32_t)(uint32_t)hi   << 12) |
                  ((int32_t)(uint32_t)lo   <<  4) |
                   (int32_t)(uint32_t)(nibble & 0x0FU);
    return (raw << 12) >> 12;   /* арифметический сдвиг = sign-extend */
}

void ICM_ParseFIFOBuffer(const uint8_t    *raw_buf,
                         uint16_t          buf_len,
                         ICM_SensorBatch_t *batch)
{
    uint16_t       offset  = 0U;
    uint8_t        n       = 0U;
    const uint8_t *pkt;
    uint8_t        hdr;

    batch->count = 0U;

    while ((uint16_t)(offset + 20U) <= buf_len)
    {
        pkt = &raw_buf[offset];
        hdr = pkt[0];

        /* MSG-пакет — маркер конца данных, пропускаем */
        if ((hdr & FIFO_HDR_MSG_BIT) != 0U)
        {
            offset += 20U;
            continue;
        }

        /* Только 20-байтный HIRES пакет: бит HIRESEN в хедере */
        if (((hdr & FIFO_HDR_HIRES_BIT) != 0U) && (n < ICM_FIFO_POLL_PACKETS))
        {
            ICM_Sample_t *s = &batch->samples[n];

            /*
             * Акселерометр:
             *   Byte17 [7:4] = Ax[3:0]
             *   Byte18 [7:4] = Ay[3:0]
             *   Byte19 [7:4] = Az[3:0]
             * Гироскоп:
             *   Byte17 [3:0] = Gx[3:0]
             *   Byte18 [3:0] = Gy[3:0]
             *   Byte19 [3:0] = Gz[3:0]
             */
            s->accel_x = build20(pkt[1],  pkt[2],  pkt[17] >> 4U);
            s->accel_y = build20(pkt[3],  pkt[4],  pkt[18] >> 4U);
            s->accel_z = build20(pkt[5],  pkt[6],  pkt[19] >> 4U);

            s->gyro_x  = build20(pkt[7],  pkt[8],  pkt[17] & 0x0FU);
            s->gyro_y  = build20(pkt[9],  pkt[10], pkt[18] & 0x0FU);
            s->gyro_z  = build20(pkt[11], pkt[12], pkt[19] & 0x0FU);

            /*
             * Температура в 20-byte пакете: 2 байта (int16_t), Big Endian
             * Формула: T[°C] = temp_raw / 128.0f + 25.0f
             */
            s->temp_raw = (int16_t)(((uint16_t)pkt[13] << 8U) |
                                     (uint16_t)pkt[14]);

            /* Timestamp: только если бит TMSTFIELDEN установлен */
            if ((hdr & FIFO_HDR_TMST_BIT) != 0U)
            {
                s->timestamp = (uint16_t)(((uint16_t)pkt[15] << 8U) |
                                           (uint16_t)pkt[16]);
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
    uint8_t  b, s, id;

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
                /* Байт [0] — адресный байт SPI, данные начинаются с [1] */
                ICM_ParseFIFOBuffer(
                    &g_fifo_data[b][s][1U],
                    (uint16_t)(ICM_FIFO_DMA_BUF_SIZE - 1U),
                    &g_sensor_batches[id]);
            }
        }
    }
}
