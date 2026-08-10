/**
 * @file    uart_telemetry.c
 * @brief   Телеметрия по USART1 (RS-485) через DMA (LL-драйвер, без HAL).
 *
 * -----------------------------------------------------------------------
 * 1 вызов UART_BuildAndSendSyncFrame() = 1 пакет (348 байт):
 *   берётся samples[count-1] (последний/свежайший) из каждого
 *   из 18 батчей g_sensor_batches[].
 *
 * Структура пакета (348 байт @ 8 Мбод):
 *
 *   [0]      0xAA  }  заголовок (2 байта)
 *   [1]      0x55  }
 *   [2..3]         }  frame_counter uint16_t LE
 *   [4..345]       }  18 × 19 байт IMU-данных (HIRES 20-bit)
 *   [346..347]     }  CRC16-CCITT LE по байтам [2..345]
 *
 * Footer 0x0D 0x0A удалён. sensor_mask удалён.
 *
 * Бюджет UART:
 *   348 bytes × 10 UART bits/byte = 3480 bits/frame.
 *   At 1600 frames/s: 5.568 Mbit/s.
 *   At 8 Mbaud UART:  reserve = 2.432 Mbit/s.
 *
 * CRC16 CCITT: полином 0x1021, init = 0xFFFF.
 * DMA1 Stream1 → USART1_TX.
 * TX-буфер в Non-Cacheable D2 SRAM (MPU Region 0, 0x30000000).
 * -----------------------------------------------------------------------
 */

#include "uart_telemetry.h"
#include "icm45686_spi.h"    /* g_sensor_fault_mask */
#include "main.h"

/* ================================================================
 * TX-буфер: Non-Cacheable D2 SRAM (0x30000000), доступен DMA1.
 * aligned(4) обязательно для корректной работы DMA.
 * ================================================================ */
static uint8_t g_uart_tx_buf[UART_PKT_TOTAL_BYTES]
    __attribute__((section(".RAM_D2"), aligned(4)));

static volatile uint8_t  g_uart_busy       = 0U;
volatile uint32_t        g_uart_drop_count = 0U;

/* frame_counter: uint16_t, переполнение modulo 65536 через ~40.96 с */
static uint16_t          s_frame_counter   = 0U;

/* ================================================================
 * CRC16-CCITT (полином 0x1021, init = 0xFFFF)
 * ================================================================ */
static uint16_t CRC16_CCITT(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t  j;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (j = 0U; j < 8U; j++)
        {
            crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

/* ================================================================
 * UART_ToU20 — извлечение младших 20 бит из signed int32_t.
 *
 * ICM_Sample_t хранит 20-bit знаковые значения в виде sign-extended
 * int32_t. Маскируем 20 младших бит: они полностью сохраняют знак
 * (бит 19 = знаковый бит 20-bit числа) и данные.
 * ================================================================ */
static inline uint32_t UART_ToU20(int32_t value)
{
    return ((uint32_t)value) & 0x000FFFFFUL;
}

/* ================================================================
 * UART_PackIMU19 — упаковка одного ICM_Sample_t в 19-byte wire-блок.
 *
 * Формат (Big Endian для осей, temp, timestamp):
 *   [0]  Ax[19:12]          [6]  Gx[19:12]
 *   [1]  Ax[11:4]           [7]  Gx[11:4]
 *   [2]  Ay[19:12]          [8]  Gy[19:12]
 *   [3]  Ay[11:4]           [9]  Gy[11:4]
 *   [4]  Az[19:12]          [10] Gz[19:12]
 *   [5]  Az[11:4]           [11] Gz[11:4]
 *   [12] temp_raw[15:8]     [14] timestamp[15:8]
 *   [13] temp_raw[7:0]      [15] timestamp[7:0]
 *   [16] (Ax[3:0]<<4)|Gx[3:0]
 *   [17] (Ay[3:0]<<4)|Gy[3:0]
 *   [18] (Az[3:0]<<4)|Gz[3:0]
 * ================================================================ */
static void UART_PackIMU19(uint8_t dst[UART_IMU_WIRE_BYTES],
                           const ICM_Sample_t *src)
{
    const uint32_t ax = UART_ToU20(src->accel_x);
    const uint32_t ay = UART_ToU20(src->accel_y);
    const uint32_t az = UART_ToU20(src->accel_z);
    const uint32_t gx = UART_ToU20(src->gyro_x);
    const uint32_t gy = UART_ToU20(src->gyro_y);
    const uint32_t gz = UART_ToU20(src->gyro_z);

    /* Accel: старшие 16 бит каждой оси (биты 19..4), Big Endian */
    dst[0]  = (uint8_t)(ax >> 12U);
    dst[1]  = (uint8_t)(ax >>  4U);
    dst[2]  = (uint8_t)(ay >> 12U);
    dst[3]  = (uint8_t)(ay >>  4U);
    dst[4]  = (uint8_t)(az >> 12U);
    dst[5]  = (uint8_t)(az >>  4U);

    /* Gyro: старшие 16 бит каждой оси (биты 19..4), Big Endian */
    dst[6]  = (uint8_t)(gx >> 12U);
    dst[7]  = (uint8_t)(gx >>  4U);
    dst[8]  = (uint8_t)(gy >> 12U);
    dst[9]  = (uint8_t)(gy >>  4U);
    dst[10] = (uint8_t)(gz >> 12U);
    dst[11] = (uint8_t)(gz >>  4U);

    /* temp_raw: int16_t → Big Endian (старший байт первым) */
    dst[12] = (uint8_t)((uint16_t)src->temp_raw >> 8U);
    dst[13] = (uint8_t)((uint16_t)src->temp_raw & 0xFFU);

    /* timestamp: uint16_t → Big Endian */
    dst[14] = (uint8_t)(src->timestamp >> 8U);
    dst[15] = (uint8_t)(src->timestamp & 0xFFU);

    /* Младшие nibble: (axis[3:0] << 4) | gAxis[3:0] */
    dst[16] = (uint8_t)(((ax & 0x0FU) << 4U) | (gx & 0x0FU));
    dst[17] = (uint8_t)(((ay & 0x0FU) << 4U) | (gy & 0x0FU));
    dst[18] = (uint8_t)(((az & 0x0FU) << 4U) | (gz & 0x0FU));
}

/* ================================================================
 * Инициализация
 * ================================================================ */
void UART_Telemetry_Init(void)
{
    /* Разрешить TC-прерывание DMA1 Stream1 */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

    /* Разрешить DMA-режим TX USART1 */
    LL_USART_EnableDMAReq_TX(USART1);

    g_uart_busy       = 0U;
    g_uart_drop_count = 0U;
    s_frame_counter   = 0U;

    /* Заголовок постоянен — пишем один раз */
    g_uart_tx_buf[UART_OFFSET_HEADER]      = UART_PKT_HEADER_0;  /* 0xAA */
    g_uart_tx_buf[UART_OFFSET_HEADER + 1U] = UART_PKT_HEADER_1;  /* 0x55 */
}

/* ================================================================
 * Сборка и отправка 348-байтного HIRES-кадра.
 *
 * Вызывать из main-loop ПОСЛЕ ICM_ParseAllFIFO().
 *
 * Алгоритм:
 *   1. Проверить g_uart_busy; пропустить кадр если DMA занят.
 *   2. Записать frame_counter (uint16_t LE) в байты [2..3].
 *   3. Для каждого датчика S00..S17:
 *      - если валиден → UART_PackIMU19() в слот 4 + id*19;
 *      - если невалиден (count==0 или fault) → заполнить нулями.
 *   4. Инкрементировать frame_counter.
 *   5. CRC16 по байтам [2..345] (344 байта payload).
 *   6. Записать CRC LE в байты [346..347].
 *   7. Запустить DMA на 348 байт.
 * ================================================================ */
void UART_BuildAndSendSyncFrame(void)
{
    uint8_t  id;
    uint32_t imu_off;
    uint16_t crc;
    uint8_t  last_idx;

    /* Если DMA ещё передаёт предыдущий пакет — пропускаем */
    if (g_uart_busy != 0U)
    {
        g_uart_drop_count++;
        return;
    }

    /* Шаг 1: frame_counter (uint16_t, Little Endian) в байты [2..3] */
    g_uart_tx_buf[UART_OFFSET_COUNTER]      = (uint8_t)( s_frame_counter        & 0xFFU);
    g_uart_tx_buf[UART_OFFSET_COUNTER + 1U] = (uint8_t)((s_frame_counter >> 8U) & 0xFFU);

    /* Шаг 2: данные датчиков S00..S17 */
    for (id = 0U; id < (uint8_t)UART_SENSOR_COUNT; id++)
    {
        imu_off = UART_OFFSET_SAMPLES + (uint32_t)id * UART_IMU_WIRE_BYTES;

        if ((g_sensor_batches[id].count > 0U) &&
            ((g_sensor_fault_mask & (1UL << id)) == 0U))
        {
            /* Берём последний (свежайший) семпл батча */
            last_idx = g_sensor_batches[id].count - 1U;
            UART_PackIMU19(&g_uart_tx_buf[imu_off],
                           &g_sensor_batches[id].samples[last_idx]);
        }
        else
        {
            /* Датчик невалиден — слот обнуляем */
            uint8_t k;
            for (k = 0U; k < UART_IMU_WIRE_BYTES; k++)
            {
                g_uart_tx_buf[imu_off + k] = 0U;
            }
        }
    }

    /* Шаг 3: инкремент счётчика (после записи в буфер, modulo 65536) */
    s_frame_counter = (uint16_t)(s_frame_counter + 1U);

    /* Шаг 4: CRC16 по payload: байты [2..345], то есть 344 байта */
    crc = CRC16_CCITT(&g_uart_tx_buf[UART_OFFSET_COUNTER],
                      UART_PAYLOAD_BYTES);
    g_uart_tx_buf[UART_OFFSET_CRC]      = (uint8_t)( crc        & 0xFFU);
    g_uart_tx_buf[UART_OFFSET_CRC + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

    /* Шаг 5: запуск DMA1 Stream1 на 348 байт */
    g_uart_busy = 1U;

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1) != 0U) {}

    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1,
                            (uint32_t)g_uart_tx_buf);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1,
                            LL_USART_DMA_GetRegAddr(USART1,
                            LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1,
                         (uint32_t)UART_PKT_TOTAL_BYTES);

    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}

/* ================================================================
 * ISR: DMA TX Complete
 * Вызывается из DMA1_Stream1_IRQHandler.
 * ================================================================ */
void UART_DMA_TxComplete(void)
{
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    g_uart_busy = 0U;
}
