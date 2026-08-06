/**
 * @file    icm45686_data.c
 * @brief   Разбор FIFO-пакетов ICM-45686 (20-бит HIRES режим, Little-Endian).
 *
 *          Формат пакета (ICM45686_FIFO_PACKET_SIZE_16BIT переопределено на 20 байт):
 *          Байт  Содержимое
 *           0    Header (Ожидаем 0x78: Accel+Gyro+Hires+Tmst)
 *           1-2  Accel X (LSB, MSB)
 *           3-4  Accel Y
 *           5-6  Accel Z
 *           7-8  Gyro X
 *           9-10 Gyro Y
 *          11-12 Gyro Z
 *          13-14 Temperature (LSB, MSB)
 *          15-17 Hires / Ext Data (Игнорируем)
 *          18-19 Timestamp (LSB, MSB)
 */

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

#ifndef ICM45686_FIFO_HEADER_TMST_BIT
#define ICM45686_FIFO_HEADER_TMST_BIT   (1U << 3)
#endif

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

    /* Внимание: ICM45686_FIFO_PACKET_SIZE_16BIT должно быть 20 в конфигурационном файле! */
    while ((offset + 20U) <= buf_len)
    {
        pkt    = &raw_buf[offset];
        header = pkt[0];

        /* Проверяем что пакет валидный (ACCEL или GYRO) */
        if (((header & ICM45686_FIFO_HEADER_ACCEL_BIT) != 0U) ||
            ((header & ICM45686_FIFO_HEADER_GYRO_BIT)  != 0U))
        {
            if (pkt_cnt < ICM_FIFO_POLL_PACKETS)
            {
                ICM_Sample_t *smp = &batch->samples[pkt_cnt];
                memset(smp, 0x00, sizeof(ICM_Sample_t));

                /* Little-Endian Accel & Gyro */
                smp->accel_x = (int16_t)(((uint16_t)pkt[2] << 8U) | (uint16_t)pkt[1]);
                smp->accel_y = (int16_t)(((uint16_t)pkt[4] << 8U) | (uint16_t)pkt[3]);
                smp->accel_z = (int16_t)(((uint16_t)pkt[6] << 8U) | (uint16_t)pkt[5]);

                smp->gyro_x  = (int16_t)(((uint16_t)pkt[8]  << 8U) | (uint16_t)pkt[7]);
                smp->gyro_y  = (int16_t)(((uint16_t)pkt[10] << 8U) | (uint16_t)pkt[9]);
                smp->gyro_z  = (int16_t)(((uint16_t)pkt[12] << 8U) | (uint16_t)pkt[11]);

                /* Температура в HIRES занимает 2 байта! Берем старший как грубое значение или оба если нужно */
                smp->temp = (int8_t)pkt[14]; // MSB температуры

                /* Timestamp сместился на байты 18 (LSB) и 19 (MSB) */
                if ((header & ICM45686_FIFO_HEADER_TMST_BIT) != 0U)
                {
                    smp->timestamp = (uint16_t)(((uint16_t)pkt[19] << 8U) | (uint16_t)pkt[18]);
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
    uint8_t  b, s;
    uint8_t  sensor_id;
    uint32_t fault_mask = g_sensor_fault_mask;

    const uint16_t data_offset = 1U;
    const uint16_t data_len    = (uint16_t)(ICM_FIFO_DMA_BUF_SIZE - data_offset);

    for (b = 0U; b < 3U; b++)
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
