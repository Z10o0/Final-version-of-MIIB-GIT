/**
 * @file    uart_telemetry.c
 * @brief   Телеметрия по USART1 через DMA (LL-драйвер, без HAL).
 *
 *          Архитектура передачи:
 *          1. UART_SendBatch() вызывается в main-loop.
 *          2. Формируется пакет в g_uart_tx_buf:
 *             [0xAA][0x55][counter_u32][payload: g_sensor_batches][CRC16][0x0D][0x0A]
 *          3. DMA1 Stream1 запускает передачу через USART1.
 *          4. По завершению DMA TC ISR (DMA1_Stream1) вызывает UART_DMA_TxComplete().
 *
 *          CRC16 CCITT (полином 0x1021, init=0xFFFF) считается по payload+counter.
 *
 *          Буфер размещён в SRAM D2 (доступна DMA1).
 *          При необходимости размещения в другой области памяти — добавить
 *          атрибут секции через linker-script или __attribute__((section("...")))
 */

#include "uart_telemetry.h"
#include "icm45686_data.h"
#include "main.h"
#include <string.h>

/* ================================================================
 * Приватные переменные
 * ================================================================ */

/* TX-буфер: должен быть в памяти доступной DMA1 (SRAM D2 или AXI-SRAM) */
static uint8_t g_uart_tx_buf[UART_PKT_TOTAL_BYTES] __attribute__((aligned(4)));

/* Флаг занятости DMA-передачи: 0 = свободно, 1 = передача идёт */
static volatile uint8_t g_uart_busy = 0U;

/* Счётчик переданных пакетов */
static volatile uint32_t g_uart_pkt_counter = 0U;

/* ================================================================
 * Приватные функции
 * ================================================================ */

/**
 * @brief  CRC16 CCITT (полином 0x1021, начальное значение 0xFFFF).
 *         Используется для контроля целостности пакета.
 * @param  data   Указатель на данные
 * @param  len    Количество байт
 * @retval CRC16
 */
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
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

/* ================================================================
 * Публичные функции
 * ================================================================ */

/**
 * @brief  Инициализация DMA-прерывания для USART1 TX.
 *         USART1 и DMA1 Stream1 уже настроены CubeMX — здесь
 *         только разрешаем прерывание завершения передачи.
 */
void UART_Telemetry_Init(void)
{
    /* Разрешить прерывание Transfer Complete для DMA1 Stream1 (USART1_TX) */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

    /* Разрешить DMA-режим передатчика USART1 */
    LL_USART_EnableDMAReq_TX(USART1);

    g_uart_busy = 0U;
    g_uart_pkt_counter = 0U;
}

/**
 * @brief  Формирует пакет и запускает неблокирующую DMA-передачу.
 *
 *         Структура пакета:
 *         Offset  Размер  Содержимое
 *         0       2       Заголовок 0xAA 0x55
 *         2       4       Счётчик пакетов (uint32_t, little-endian)
 *         6       N       Payload: g_sensor_batches (все 18 датчиков)
 *         6+N     2       CRC16 CCITT (little-endian) по байтам [2..6+N-1]
 *         6+N+2   2       Footer 0x0D 0x0A
 *
 *         Если DMA занят — пакет пропускается.
 */
void UART_SendBatch(void)
{
    uint16_t crc;
    uint32_t payload_offset;
    uint32_t crc_offset;
    uint32_t crc_cover_len;  /* Число байт для подсчёта CRC (counter + payload) */

    /* Если предыдущая передача не завершена — пропустить */
    if (g_uart_busy != 0U)
    {
        return;
    }

    /* --- Формирование заголовка --- */
    g_uart_tx_buf[0] = UART_PKT_HEADER_0;
    g_uart_tx_buf[1] = UART_PKT_HEADER_1;

    /* --- Счётчик пакетов (little-endian) --- */
    g_uart_tx_buf[2] = (uint8_t)( g_uart_pkt_counter        & 0xFFU);
    g_uart_tx_buf[3] = (uint8_t)((g_uart_pkt_counter >>  8U) & 0xFFU);
    g_uart_tx_buf[4] = (uint8_t)((g_uart_pkt_counter >> 16U) & 0xFFU);
    g_uart_tx_buf[5] = (uint8_t)((g_uart_pkt_counter >> 24U) & 0xFFU);

    payload_offset = 6U;

    /* --- Копирование данных датчиков --- */
    memcpy(&g_uart_tx_buf[payload_offset],
           (const void *)g_sensor_batches,
           UART_PAYLOAD_BYTES);

    /* --- CRC16 считается по counter (4 байта) + payload --- */
    crc_cover_len = 4U + (uint32_t)UART_PAYLOAD_BYTES;
    crc = CRC16_CCITT(&g_uart_tx_buf[2], crc_cover_len);

    crc_offset = payload_offset + (uint32_t)UART_PAYLOAD_BYTES;
    g_uart_tx_buf[crc_offset]     = (uint8_t)( crc        & 0xFFU);
    g_uart_tx_buf[crc_offset + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

    /* --- Footer CR LF --- */
    g_uart_tx_buf[crc_offset + 2U] = UART_PKT_FOOTER_0;
    g_uart_tx_buf[crc_offset + 3U] = UART_PKT_FOOTER_1;

    /* --- Запуск DMA1 Stream1 (USART1_TX) --- */
    g_uart_busy = 1U;
    g_uart_pkt_counter++;

    /* Отключить поток перед перенастройкой */
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1) != 0U) { /* ждать деактивации */ }

    /* Сброс флагов TC/TE */
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);

    /* Установить адрес источника и количество байт */
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)g_uart_tx_buf);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, (uint32_t)UART_PKT_TOTAL_BYTES);

    /* Включить поток — передача начинается автоматически */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}

/**
 * @brief  Обработчик завершения DMA TX.
 *         Вызывается из DMA1_Stream1_IRQHandler.
 *         Сбрасывает флаг занятости.
 */
void UART_DMA_TxComplete(void)
{
    /* Отключить поток DMA после завершения передачи */
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

    /* Освободить флаг занятости */
    g_uart_busy = 0U;
}
