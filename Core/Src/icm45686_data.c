/**
 * @file    icm45686_data.c
 * @brief   Разбор FIFO-пакетов ICM-45686 (16-бит режим, Little-Endian).
 *
 *          Формат пакета (ICM45686_FIFO_PACKET_SIZE_16BIT = 16 байт):
 *          Байт  Содержимое
 *           0    Header
 *           1    Accel X LSB  ← Little-Endian!
 *           2    Accel X MSB
 *           3    Accel Y LSB
 *           4    Accel Y MSB
 *           5    Accel Z LSB
 *           6    Accel Z MSB
 *           7    Gyro X LSB
 *           8    Gyro X MSB
 *           9    Gyro Y LSB
 *          10    Gyro Y MSB
 *          11    Gyro Z LSB
 *          12    Gyro Z MSB
 *          13    Temperature
 *          14    Timestamp LSB
 *          15    Timestamp MSB
 *
 *          Первый байт RX-буфера DMA — адрес команды (отброшен),
 *          данные начинаются с offset +1 от начала g_fifo_data[b][s].
 */

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

/* ================================================================
 * Глобальный массив результатов: по одному ICM_SensorBatch_t на датчик
 * ================================================================ */
ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/* ================================================================
 * ICM_ParseFIFOBuffer — разбор буфера одного исправного датчика
 * ================================================================ */
void ICM_ParseFIFOBuffer(const uint8_t    *raw_buf,
                         uint16_t          buf_len,
                         ICM_SensorBatch_t *batch)
{
    uint16_t       offset  = 0U;
    uint8_t        pkt_cnt = 0U;
    uint8_t        header;
    const uint8_t *pkt;

    batch->count = 0U;

    while ((offset + (uint16_t)ICM45686_FIFO_PACKET_SIZE_16BIT) <= buf_len)
    {
        pkt    = &raw_buf[offset];
        header = pkt[0];

        /*
         * FIFO Header (16-bit packet):
         *   bit7 = MSG   — пустой пакет-заглушка, пропустить
         *   bit6 = ACCEL — данные акселерометра валидны
         *   bit5 = GYRO  — данные гироскопа валидны
         */
        if (((header & ICM45686_FIFO_HEADER_ACCEL_BIT) != 0U) ||
            ((header & ICM45686_FIFO_HEADER_GYRO_BIT)  != 0U))
        {
            if (pkt_cnt < ICM_FIFO_POLL_PACKETS)
            {
                ICM_Sample_t *smp = &batch->samples[pkt_cnt];

                /* ✅ Little-Endian: LSB по меньшему адресу */
                smp->accel_x = (int16_t)(((uint16_t)pkt[2] << 8U) | (uint16_t)pkt[1]);
                smp->accel_y = (int16_t)(((uint16_t)pkt[4] << 8U) | (uint16_t)pkt[3]);
                smp->accel_z = (int16_t)(((uint16_t)pkt[6] << 8U) | (uint16_t)pkt[5]);

                smp->gyro_x  = (int16_t)(((uint16_t)pkt[8]  << 8U) | (uint16_t)pkt[7]);
                smp->gyro_y  = (int16_t)(((uint16_t)pkt[10] << 8U) | (uint16_t)pkt[9]);
                smp->gyro_z  = (int16_t)(((uint16_t)pkt[12] << 8U) | (uint16_t)pkt[11]);

                smp->temp = (int8_t)pkt[13];

                /* ✅ Little-Endian timestamp */
                smp->timestamp = (uint16_t)(((uint16_t)pkt[15] << 8U) | (uint16_t)pkt[14]);

                pkt_cnt++;
            }
        }
        /* header & 0x80 — MSG-пакет, пропускаем */

        offset += (uint16_t)ICM45686_FIFO_PACKET_SIZE_16BIT;
    }

    batch->count = pkt_cnt;
}

/* ================================================================
 * ICM_ParseAllFIFO — разбор всех 18 датчиков.
 *
 * Для датчиков с fault=1: samples обнуляются, count = 0.
 * ================================================================ */
void ICM_ParseAllFIFO(void)
{
    uint8_t  b, s;
    uint8_t  sensor_id;
    uint32_t fault_mask = g_sensor_fault_mask;

    /* Первый байт RX-буфера — адрес команды, данные с байта 1 */
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
