/**
 * @file    uart_telemetry.c
 * @brief   Телеметрия по USART1 (RS-485) через DMA (LL-драйвер, без HAL).
 *
 * -----------------------------------------------------------------------
 * [ОТЛАДОЧНЫЙ РЕЖИМ]: 1 пакет = последний семпл от каждого из 18 датчиков.
 *
 * Топология:
 *   UART_BuildAndSendSyncFrame() ← вызывается из main-loop
 *     шаг 1: собрать ICM_SyncFrame_t из g_sensor_batches[]
 *             (для каждого датчика берём samples[count-1])
 *     шаг 2: скопировать в TX-буфер, посчитать CRC16, запустить DMA
 *
 * Структура пакета (518 байт, @ 10 Мбит/с → 0.518 мс):
 *
 *   [0]     0xAA  }
 *   [1]     0x55  }  заголовок (2 б)
 *   [2..513]      }  ICM_SyncFrame_t (512 б)
 *                      [2..5]   frame_counter  uint32_t LE
 *                      [6..9]   sensor_mask    uint32_t LE
 *                      [10..513] samples[18]   ICM_Sample_t×18
 *   [514..515]    }  CRC16 CCITT, LE, по [2..513]
 *   [516]   0x0D  }  footer CR
 *   [517]   0x0A  }  footer LF
 *
 * CRC16 CCITT: полином 0x1021, init = 0xFFFF.
 * DMA1 Stream1 → USART1_TX.
 * TX-буфер в Non-Cacheable D2 SRAM (MPU Region 0, 0x30000000).
 * -----------------------------------------------------------------------
 */

#include "uart_telemetry.h"
#include "icm45686_spi.h"    /* g_sensor_fault_mask */
#include "main.h"
#include <string.h>

/* ================================================================
 * TX-буфер: Non-Cacheable D2 SRAM (0x30000000), доступен DMA1.
 * aligned(4) обязательно для корректной работы DMA.
 * ================================================================ */
static uint8_t g_uart_tx_buf[UART_PKT_TOTAL_BYTES]
    __attribute__((section(".RAM_D2"), aligned(4)));

static volatile uint8_t  g_uart_busy = 0U;
volatile uint32_t        g_uart_drop_count = 0U;

/* Внутренний кадр — сборка внутри .c, не экспортируется в .h */
static ICM_SyncFrame_t   s_sync_frame;
static uint32_t          s_frame_counter = 0U;

/* ================================================================
 * CRC16 CCITT (полином 0x1021, init = 0xFFFF)
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
 * Сборка синхронного кадра из g_sensor_batches[]
 *
 * [ОТЛАДКА] Берёт samples[count-1] — последний (свежайший) семпл батча.
 * Если датчик в fault-маске или count == 0 — слот обнуляется,
 * бит sensor_mask не выставляется.
 * ================================================================ */
static void BuildSyncFrame(void)
{
    uint8_t id;
    uint8_t last_idx;
    static const ICM_Sample_t k_zero_sample = {0, 0, 0, 0, 0, 0, 0, 0};

    s_sync_frame.frame_counter = s_frame_counter;
    s_frame_counter++;
    s_sync_frame.sensor_mask = 0U;

    for (id = 0U; id < (uint8_t)ICM_TOTAL_SENSORS; id++)
    {
        if ((g_sensor_batches[id].count > 0U) &&
            ((g_sensor_fault_mask & (1UL << id)) == 0U))
        {
            last_idx = g_sensor_batches[id].count - 1U;
            s_sync_frame.samples[id] = g_sensor_batches[id].samples[last_idx];
            s_sync_frame.sensor_mask |= (1UL << id);
        }
        else
        {
            s_sync_frame.samples[id] = k_zero_sample;
        }
    }
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

    /* Заголовок и footer постоянны — пишем один раз */
    g_uart_tx_buf[UART_OFFSET_HEADER]      = UART_PKT_HEADER_0;
    g_uart_tx_buf[UART_OFFSET_HEADER + 1U] = UART_PKT_HEADER_1;
    g_uart_tx_buf[UART_OFFSET_FOOTER]      = UART_PKT_FOOTER_0;
    g_uart_tx_buf[UART_OFFSET_FOOTER + 1U] = UART_PKT_FOOTER_1;
}

/* ================================================================
 * [ОТЛАДКА] Сборка и отправка синхронного кадра
 *
 * Вызывать из main-loop ПОСЛЕ ICM_ParseAllFIFO().
 * ================================================================ */
void UART_BuildAndSendSyncFrame(void)
{
    uint16_t crc;

    /* Если DMA ещё передаёт предыдущий пакет — пропускаем */
    if (g_uart_busy != 0U)
    {
        g_uart_drop_count++;
        return;
    }

    /* Шаг 1: собрать кадр из последних семплов каждого батча */
    BuildSyncFrame();

    /* Шаг 2: скопировать payload в TX-буфер */
    memcpy(&g_uart_tx_buf[UART_OFFSET_PAYLOAD],
           (const void *)&s_sync_frame,
           UART_PAYLOAD_BYTES);

    /* Шаг 3: CRC16 по всему payload */
    crc = CRC16_CCITT(&g_uart_tx_buf[UART_OFFSET_PAYLOAD],
                      UART_PAYLOAD_BYTES);
    g_uart_tx_buf[UART_OFFSET_CRC]      = (uint8_t)( crc        & 0xFFU);
    g_uart_tx_buf[UART_OFFSET_CRC + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

    /* Шаг 4: запуск DMA1 Stream1 */
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
                         UART_PKT_TOTAL_BYTES);

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
