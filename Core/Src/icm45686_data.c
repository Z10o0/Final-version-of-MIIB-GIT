/**
 * @file    icm45686_data.c
 * @brief   Разбор 20-байтных HIRES FIFO-пакетов ICM-45686.
 *
 * Точный байтовый layout (Big-Endian, 20-bit данные):
 *
 *  Byte  0      Header
 *               bit7 = MSG (пустой пакет — пропустить)
 *               bit6 = ACCEL_EN
 *               bit5 = GYRO_EN
 *               bit4 = HIRES_EN   ← в HIRES режиме всегда 1
 *               bit3 = TMST_FIELD_EN ← timestamp в байтах 14-15
 *
 *  Bytes  1- 2  Accel X [19:12], [11:4]   (Big-Endian)
 *  Bytes  3- 4  Accel Y [19:12], [11:4]
 *  Bytes  5- 6  Accel Z [19:12], [11:4]
 *  Bytes  7- 8  Gyro  X [19:12], [11:4]
 *  Bytes  9-10  Gyro  Y [19:12], [11:4]
 *  Bytes 11-12  Gyro  Z [19:12], [11:4]
 *  Byte  13     Temp  [19:12]  (MSB температуры)
 *  Byte  14     Timestamp LSB
 *  Byte  15     Timestamp MSB
 *  Byte  16     Accel X [3:0] | Accel Y [3:0]   (nibble-pack)
 *  Byte  17     Accel Z [3:0] | Gyro  X [3:0]
 *  Byte  18     Gyro  Y [3:0] | Gyro  Z [3:0]
 *  Byte  19     Temp  [11:4]  (LSB температуры)
 *
 * Сборка 20-битного знакового значения для оси (пример Accel X):
 *   raw20 = ((uint32_t)pkt[1] << 12) | ((uint32_t)pkt[2] << 4)
 *           | ((pkt[16] >> 4) & 0x0F)
 *   int32_t val = (int32_t)(raw20 << 12) >> 12  ← sign-extend с бита 19
 */

#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include <string.h>

#ifndef ICM45686_FIFO_HEADER_TMST_BIT
#define ICM45686_FIFO_HEADER_TMST_BIT   (1U << 3)
#endif

#ifndef ICM45686_FIFO_HEADER_HIRES_BIT
#define ICM45686_FIFO_HEADER_HIRES_BIT  (1U << 4)
#endif

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/* ================================================================
 * Вспомогательный макрос: собирает 20-битное знаковое значение
 * из двух основных байтов (msb, lsb) и одного nibble (4 младших бита).
 *
 * msb_byte  — байт [19:12]
 * lsb_byte  — байт [11:4]
 * nibble    — биты [3:0]
 *
 * Формула:
 *   raw20 = (msb << 12) | (lsb << 4) | nibble   → беззнаковое 20-bit
 *   sign-extend: сдвиг влево на 12, затем арифм. вправо на 12
 * ================================================================ */
static inline int32_t ICM_Build20bit(uint8_t msb_byte,
                                     uint8_t lsb_byte,
                                     uint8_t nibble)
{
    uint32_t raw = ((uint32_t)msb_byte << 12U) |
                   ((uint32_t)lsb_byte <<  4U) |
                   ((uint32_t)nibble   &  0x0FU);
    /* Sign-extend с позиции бита 19 */
    if ((raw & 0x00080000UL) != 0UL)
    {
        raw |= 0xFFF00000UL;
    }
    return (int32_t)raw;
}

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

        /* Пропускаем пустые (MSG) пакеты */
        if ((header & ICM45686_FIFO_HEADER_MSG_BIT) != 0U)
        {
            offset += 20U;
            continue;
        }

        /* Обрабатываем только пакеты с данными */
        if (((header & ICM45686_FIFO_HEADER_ACCEL_BIT) != 0U) ||
            ((header & ICM45686_FIFO_HEADER_GYRO_BIT)  != 0U))
        {
            if (pkt_cnt < ICM_FIFO_POLL_PACKETS)
            {
                ICM_Sample_t *smp = &batch->samples[pkt_cnt];
                memset(smp, 0x00, sizeof(ICM_Sample_t));

                /* ================================================
                 * Accel: Big-Endian, 20-bit
                 * pkt[16] upper nibble = Accel X [3:0]
                 * pkt[16] lower nibble = Accel Y [3:0]
                 * pkt[17] upper nibble = Accel Z [3:0]
                 * ================================================ */
                smp->accel_x = ICM_Build20bit(pkt[1],  pkt[2],  (pkt[16] >> 4U));
                smp->accel_y = ICM_Build20bit(pkt[3],  pkt[4],  (pkt[16] & 0x0FU));
                smp->accel_z = ICM_Build20bit(pkt[5],  pkt[6],  (pkt[17] >> 4U));

                /* ================================================
                 * Gyro: Big-Endian, 20-bit
                 * pkt[17] lower nibble = Gyro X [3:0]
                 * pkt[18] upper nibble = Gyro Y [3:0]
                 * pkt[18] lower nibble = Gyro Z [3:0]
                 * ================================================ */
                smp->gyro_x  = ICM_Build20bit(pkt[7],  pkt[8],  (pkt[17] & 0x0FU));
                smp->gyro_y  = ICM_Build20bit(pkt[9],  pkt[10], (pkt[18] >> 4U));
                smp->gyro_z  = ICM_Build20bit(pkt[11], pkt[12], (pkt[18] & 0x0FU));

                /* ================================================
                 * Температура: 16-bit сырое значение
                 * pkt[13] = Temp [19:12]  (MSB)
                 * pkt[19] = Temp [11:4]   (LSB)
                 *
                 * Для перевода в °C (из даташита):
                 *   temp_c = (temp_raw / 128.0f) + 25.0f
                 * где temp_raw = sign-extended 16-бит из pkt[13]<<8|pkt[19]
                 * ================================================ */
                smp->temp_raw = (int16_t)(((uint16_t)pkt[13] << 8U) |
                                           (uint16_t)pkt[19]);

                /* ================================================
                 * Timestamp: Little-Endian, байты 14-15
                 * Присутствует только если TMST_FIELD_EN=1 в header
                 * ================================================ */
                if ((header & ICM45686_FIFO_HEADER_TMST_BIT) != 0U)
                {
                    smp->timestamp = (uint16_t)(((uint16_t)pkt[15] << 8U) |
                                                 (uint16_t)pkt[14]);
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

    const uint16_t data_offset = 1U;  /* байт 0 = SPI-команда, данные с байта 1 */
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
