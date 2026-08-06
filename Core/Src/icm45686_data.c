#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

#ifndef ICM45686_FIFO_HEADER_TMST_BIT
#define ICM45686_FIFO_HEADER_TMST_BIT  (1U << 3)
#endif
#ifndef ICM45686_FIFO_HEADER_MSG_BIT
#define ICM45686_FIFO_HEADER_MSG_BIT   (1U << 7)
#endif

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/*
 * Сборка 20-битного знакового значения:
 *   hi  = байт H [19:12]  (MSB)
 *   lo  = байт L [11:4]
 *   lsb = nibble [3:0]
 *
 * Алгоритм: складываем в int32_t, делаем sign-extend через сдвиг.
 *   raw = (hi << 12) | (lo << 4) | lsb   → 20-битное число в [19:0]
 *   sign-extend: (raw << 12) >> 12         → знак из бита 19 на биты [31:20]
 */
static inline int32_t icm_build20(uint8_t hi, uint8_t lo, uint8_t lsb)
{
    int32_t raw = ((uint32_t)hi  << 12U) |
                  ((uint32_t)lo  <<  4U) |
                  ((uint32_t)lsb & 0x0FU);
    /* Арифметический sign-extend с бита 19 */
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
                 * ACCEL — Byte 1=Ax H, 2=Ax L, ..., 6=Az L
                 * Byte 17: hi-nibble = Ax[3:0], lo-nibble = Gx[3:0]
                 * Byte 18: hi-nibble = Ay[3:0], lo-nibble = Gy[3:0]
                 * Byte 19: hi-nibble = Az[3:0], lo-nibble = Gz[3:0]
                 */
                smp->accel_x = icm_build20(pkt[1],  pkt[2],  pkt[17] >> 4U);
                smp->accel_y = icm_build20(pkt[3],  pkt[4],  pkt[18] >> 4U);
                smp->accel_z = icm_build20(pkt[5],  pkt[6],  pkt[19] >> 4U);

                /*
                 * GYRO — Byte 7=Gx H, 8=Gx L, ..., 12=Gz L
                 * lo-nibble байт 17,18,19
                 */
                smp->gyro_x  = icm_build20(pkt[7],  pkt[8],  pkt[17] & 0x0FU);
                smp->gyro_y  = icm_build20(pkt[9],  pkt[10], pkt[18] & 0x0FU);
                smp->gyro_z  = icm_build20(pkt[11], pkt[12], pkt[19] & 0x0FU);

                /*
                 * TEMP — Big-Endian: Byte 13=H, Byte 14=L
                 * Формула: °C = temp_raw / 128.0f + 25.0f
                 */
                smp->temp_raw = (int16_t)(((uint16_t)pkt[13] << 8U) |
                                           (uint16_t)pkt[14]);

                /*
                 * TIMESTAMP — Big-Endian: Byte 15=H, Byte 16=L
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
