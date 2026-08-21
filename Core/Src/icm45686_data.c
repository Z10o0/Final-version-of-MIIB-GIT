/**
 * @file icm45686_data.c
 *
 * ICM-45686 FIFO 20-byte HIRES parser
 * Строго по даташиту DS-000577 §6.1 + SREGDATAENDIANSEL=1 (Big Endian).
 *
 * ВАЖНО: датчик настроен в Big Endian (IREG 0xA267 bit1 = 1).
 * В Big Endian MSB-байт идёт ПЕРВЫМ в паре.
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
 * Сборка 20-bit знакового:
 *   raw = (MSB_byte << 12) | (LSB_byte << 4) | nibble
 *   sign-extend: (raw << 12) >> 12
 *
 * ------------------------------------------------------------------
 * [ИЗМЕНЕНИЯ для батч-режима 16 пакетов/опрос @ 100 Гц]
 *
 * ICM_ParseFIFOBuffer() логику не меняли: она уже была написана как
 * цикл "while (offset + 20 <= buf_len)" с ограничением
 * "n < ICM_FIFO_POLL_PACKETS", то есть уже умела разбирать несколько
 * пакетов подряд. Раньше ICM_FIFO_POLL_PACKETS был 1, теперь 16 —
 * функция автоматически разбирает все 16 пакетов без правок кода.
 *
 * buf_len, который передаёт ICM_ParseAllFIFO(), теперь равен
 * (ICM_FIFO_DMA_BUF_SIZE - 1) = 320 байт вместо прежних 20 байт —
 * это тоже обеспечивается автоматически через define в
 * icm45686_config.h, без изменений в этом файле.
 * ------------------------------------------------------------------
 */

#include <string.h>
#include "icm45686_data.h"
#include "icm45686_spi.h"
#include "icm45686_regs.h"

#define FIFO_HDR_MSG_BIT    (1U << 7)
#define FIFO_HDR_ACCEL_BIT  (1U << 6)
#define FIFO_HDR_GYRO_BIT   (1U << 5)
#define FIFO_HDR_HIRES_BIT  (1U << 4)
#define FIFO_HDR_TMST_BIT   (1U << 3)

ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/**
 * build20 — сборка 20-bit знакового числа из трёх компонентов.
 *
 * @param msb    байт [19:12] — старший байт (первый в BE-пакете)
 * @param lsb    байт [11:4]  — младший байт (второй в BE-пакете)
 * @param nibble биты [3:0]   — из byte17/18/19, уже >>4 или &0x0F
 */
static inline int32_t build20(uint8_t msb, uint8_t lsb, uint8_t nibble)
{
    int32_t raw = ((int32_t)(uint32_t)msb  << 12) |
                  ((int32_t)(uint32_t)lsb  << 4)  |
                  (int32_t)(uint32_t)(nibble & 0x0FU);
    /* sign-extend с позиции бит19 */
    return (raw << 12) >> 12;
}

/**
 * ICM_ParseFIFOBuffer — разобрать все валидные HIRES-пакеты из
 * непрерывного буфера FIFO одного датчика.
 *
 * @param raw_buf указатель на первый байт payload (после cmd/addr байта)
 * @param buf_len длина payload в байтах (кратно 20; при batch=16 → 320)
 * @param batch   куда записать результат (samples[] + count)
 */
void ICM_ParseFIFOBuffer(const uint8_t *raw_buf,
                          uint16_t buf_len,
                          ICM_SensorBatch_t *batch)
{
    uint16_t offset = 0U;
    uint8_t  n = 0U;
    const uint8_t *pkt;
    uint8_t hdr;

    batch->count = 0U;

    while ((uint16_t)(offset + 20U) <= buf_len)
    {
        pkt = &raw_buf[offset];
        hdr = pkt[0];

        /* MSG-пакет: маркер конца FIFO / служебный, пропускаем данные */
        if ((hdr & FIFO_HDR_MSG_BIT) != 0U)
        {
            offset += 20U;
            continue;
        }

        /* HIRES пакет (header bit4 = 1) */
        if (((hdr & FIFO_HDR_HIRES_BIT) != 0U) && (n < ICM_FIFO_POLL_PACKETS))
        {
            ICM_Sample_t *s = &batch->samples[n];

            /*
             * Big Endian: MSB-байт идёт ПЕРВЫМ.
             *
             * Акселерометр:
             *   pkt[1]=Ax MSB, pkt[2]=Ax LSB
             *   pkt[3]=Ay MSB, pkt[4]=Ay LSB
             *   pkt[5]=Az MSB, pkt[6]=Az LSB
             *
             * Гироскоп:
             *   pkt[7]=Gx MSB,  pkt[8]=Gx LSB
             *   pkt[9]=Gy MSB,  pkt[10]=Gy LSB
             *   pkt[11]=Gz MSB, pkt[12]=Gz LSB
             *
             * Nibbles:
             *   pkt[17]: hi=Ax[3:0], lo=Gx[3:0]
             *   pkt[18]: hi=Ay[3:0], lo=Gy[3:0]
             *   pkt[19]: hi=Az[3:0], lo=Gz[3:0]
             */
            s->accel_x = build20(pkt[1], pkt[2], (uint8_t)(pkt[17] >> 4U));
            s->accel_y = build20(pkt[3], pkt[4], (uint8_t)(pkt[18] >> 4U));
            s->accel_z = build20(pkt[5], pkt[6], (uint8_t)(pkt[19] >> 4U));

            s->gyro_x = build20(pkt[7],  pkt[8],  (uint8_t)(pkt[17] & 0x0FU));
            s->gyro_y = build20(pkt[9],  pkt[10], (uint8_t)(pkt[18] & 0x0FU));
            s->gyro_z = build20(pkt[11], pkt[12], (uint8_t)(pkt[19] & 0x0FU));

            /*
             * Температура: Big Endian, 2 байта.
             * T[°C] = temp_raw / 128.0f + 25.0f
             */
            s->temp_raw = (int16_t)(((uint16_t)pkt[13] << 8U) |
                                     (uint16_t)pkt[14]);

            /*
             * Timestamp: Big Endian. Разрешение: 1 мкс/LSB (TMST_RESOL=0).
             * uint16_t → диапазон 0..65535 мкс (~65 мс), затем wraparound.
             */
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

/**
 * ICM_ParseAllFIFO — разобрать FIFO всех 18 датчиков.
 *
 * Для каждого датчика: если он помечен неисправным (g_sensor_fault_mask),
 * весь батч обнуляется и count=0. Иначе разбирается payload длиной
 * (ICM_FIFO_DMA_BUF_SIZE - 1) байт, начиная с байта [1] (байт [0] —
 * SPI cmd/addr, данные не содержит).
 */
volatile uint32_t g_icm_parse_cyc_last = 0U;
volatile uint32_t g_icm_parse_cyc_max  = 0U;
volatile uint32_t g_icm_parse_us_last  = 0U;
volatile uint32_t g_icm_parse_us_max   = 0U;

void ICM_ParseAllFIFO(void)
{
    uint32_t start_cyc = DWT->CYCCNT;

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
                const uint8_t *src = (b == 5U)
                    ? &g_fifo_data_spi6[s][1U]
                    : &g_fifo_data[b][s][1U];

                ICM_ParseFIFOBuffer(
                    src,
                    (uint16_t)(ICM_FIFO_DMA_BUF_SIZE - 1U),
                    &g_sensor_batches[id]);
            }
        }
    }

    uint32_t delta_cyc = DWT->CYCCNT - start_cyc;
    g_icm_parse_cyc_last = delta_cyc;
    if (delta_cyc > g_icm_parse_cyc_max) g_icm_parse_cyc_max = delta_cyc;

    uint32_t us = delta_cyc / (SystemCoreClock / 1000000UL);
    g_icm_parse_us_last = us;
    if (us > g_icm_parse_us_max) g_icm_parse_us_max = us;
}
