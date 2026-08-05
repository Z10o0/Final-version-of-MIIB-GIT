/**
 * @file    icm45686_data.c
 * @brief   Разбор FIFO-пакетов ICM-45686 (16-бит режим).
 *
 *          Формат пакета (ICM45686_FIFO_PACKET_SIZE_16BIT = 16 байт):
 *          Байт  Содержимое
 *           0    Header
 *           1    Accel X MSB
 *           2    Accel X LSB
 *           3    Accel Y MSB
 *           4    Accel Y LSB
 *           5    Accel Z MSB
 *           6    Accel Z LSB
 *           7    Gyro X MSB
 *           8    Gyro X LSB
 *           9    Gyro Y MSB
 *          10    Gyro Y LSB
 *          11    Gyro Z MSB
 *          12    Gyro Z LSB
 *          13    Temperature
 *          14    Timestamp LSB      ⚠ — в коде указано LSB, но даташит говорит H (MSB)!
 *          15    Timestamp MSB      ⚠
 *
 *          Первые 2 байта RX-буфера DMA — служебные (адрес+dummy),
 *          поэтому данные начинаются с offset +1 от начала g_fifo_data[b][s].
 *
 *          Неисправные датчики (g_sensor_fault_mask):
 *          ICM_ParseAllFIFO() записывает нули во все поля g_sensor_batches
 *          для таких датчиков и выставляет count = 0.
 *          ПК-сторона видит нулевые данные и может определить fault
 *          по count == 0 или по маске в заголовке пакета.
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
 * ICM_ParseFIFOBuffer — разбор буфера одного ИСПРАВНОГО датчика
 * ================================================================ */
void ICM_ParseFIFOBuffer(const uint8_t *raw_buf,
                         uint16_t       buf_len,
                         ICM_SensorBatch_t *batch)
{
    uint16_t offset  = 0U;
    uint8_t  pkt_cnt = 0U;
    uint8_t  header;
    const uint8_t *pkt;

    batch->count = 0U;

    while ((offset + (uint16_t)ICM45686_FIFO_PACKET_SIZE_16BIT) <= buf_len)
    {
        pkt    = &raw_buf[offset];
        header = pkt[0];

        /* Проверка валидности: хотя бы гироскоп или акселерометр */
        if (((header & ICM45686_FIFO_HEADER_ACCEL_BIT) != 0U) ||
            ((header & ICM45686_FIFO_HEADER_GYRO_BIT)  != 0U))
        {
            if (pkt_cnt < ICM_FIFO_POLL_PACKETS)
            {
                ICM_Sample_t *smp = &batch->samples[pkt_cnt];

                /* Акселерометр (big-endian в FIFO) */
                smp->accel_x = (int16_t)(((uint16_t)pkt[1] << 8U) | (uint16_t)pkt[2]);
                smp->accel_y = (int16_t)(((uint16_t)pkt[3] << 8U) | (uint16_t)pkt[4]);
                smp->accel_z = (int16_t)(((uint16_t)pkt[5] << 8U) | (uint16_t)pkt[6]);

                /* Гироскоп */
                smp->gyro_x  = (int16_t)(((uint16_t)pkt[7]  << 8U) | (uint16_t)pkt[8]);
                smp->gyro_y  = (int16_t)(((uint16_t)pkt[9]  << 8U) | (uint16_t)pkt[10]);
                smp->gyro_z  = (int16_t)(((uint16_t)pkt[11] << 8U) | (uint16_t)pkt[12]);

                /* Температура */
                smp->temp = (int8_t)pkt[13];

                smp->timestamp = (uint16_t)(((uint16_t)pkt[14] << 8U) | (uint16_t)pkt[15]);

                pkt_cnt++;
            }
        }
        /* Header == 0x80 — пустой пакет-заглушка, пропускаем */

        offset += (uint16_t)ICM45686_FIFO_PACKET_SIZE_16BIT;
    }

    batch->count = pkt_cnt;
}

/* ================================================================
 * ICM_ParseAllFIFO — разбор всех 18 датчиков.
 *
 * Для датчиков с fault=1 (бит установлен в g_sensor_fault_mask):
 *   - все samples обнуляются через memset
 *   - count = 0
 * Это гарантирует, что в UART-пакете неисправный датчик
 * всегда виден как нулевые данные.
 * ================================================================ */
void ICM_ParseAllFIFO(void)
{
    uint8_t  b, s;
    uint8_t  sensor_id;
    uint32_t fault_mask = g_sensor_fault_mask;  /* Локальная копия — без volatile overhead */

    /* Первый байт RX-буфера — адрес команды (отброшен), данные с байта 1 */
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
                /* Датчик неисправен — заполнить нулями, count = 0. */
                memset(g_sensor_batches[sensor_id].samples, 0x00,
                       sizeof(g_sensor_batches[sensor_id].samples));
                g_sensor_batches[sensor_id].count = 0U;
            }
            else
            {
                /* Датчик исправен — разобрать FIFO-буфер */
                ICM_ParseFIFOBuffer(
                    &g_fifo_data[b][s][data_offset],
                    data_len,
                    &g_sensor_batches[sensor_id]);
            }
        }
    }
}
