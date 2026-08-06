/**
 * @file    uart_telemetry.h
 * @brief   Телеметрия по USART1 (RS-485) через DMA, LL-драйвер, без HAL.
 *
 * -----------------------------------------------------------------------
 * [ОТЛАДОЧНЫЙ РЕЖИМ]
 *
 * 1 вызов UART_BuildAndSendSyncFrame() = 1 синхронный кадр:
 *   берётся samples[count-1] (последний/свежайший) из каждого
 *   из 18 батчей g_sensor_batches[].
 *
 * -----------------------------------------------------------------------
 * Структура UART-пакета:
 *
 *  Смещение  Размер  Содержимое
 *  --------  ------  ------------------------------------------
 *     0        2     Заголовок: 0xAA 0x55
 *     2      512     ICM_SyncFrame_t
 *                      [0..3]   frame_counter (uint32_t, LE)
 *                      [4..7]   sensor_mask   (uint32_t, LE)
 *                      [8..511] samples[18]   (ICM_Sample_t ×18)
 *   514        2     CRC16 CCITT (LE) по байтам [2..513]
 *   516        2     Footer: 0x0D 0x0A
 *  --------  ------
 *  ИТОГО: 518 байт
 *
 * -----------------------------------------------------------------------
 * Временной бюджет @ 10 Мбит/с:
 *   518 × 10 / 10 000 000 = 0.518 мс < период TIM6 (1.5625 мс) → запас ×3.
 *
 * -----------------------------------------------------------------------
 * ICM_SyncFrame_t объявлена здесь, чтобы не трогать icm45686_data.h.
 */

#ifndef UART_TELEMETRY_H
#define UART_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "icm45686_data.h"   /* ICM_Sample_t, ICM_SensorBatch_t, g_sensor_batches */
#include "icm45686_config.h" /* ICM_TOTAL_SENSORS */

/* ================================================================
 * ICM_SyncFrame_t
 *
 * Синхронный кадр: 1 семпл от каждого из 18 датчиков.
 * sizeof = 4 + 4 + 18 × 28 = 512 байт.
 *
 * frame_counter : монотонный счётчик, растёт с каждым пакетом.
 * sensor_mask   : bit N = 1 → датчик N валиден (count > 0, не в fault).
 *                 bit N = 0 → слот samples[N] заполнен нулями.
 * ================================================================ */
typedef struct {
    uint32_t     frame_counter;
    uint32_t     sensor_mask;
    ICM_Sample_t samples[ICM_TOTAL_SENSORS];  /* 18 × 28 = 504 байта */
} ICM_SyncFrame_t;  /* sizeof = 512 байт */

/* ================================================================
 * Константы пакета
 * ================================================================ */
#define UART_PKT_HEADER_0       0xAAU
#define UART_PKT_HEADER_1       0x55U
#define UART_PKT_FOOTER_0       0x0DU   /* CR */
#define UART_PKT_FOOTER_1       0x0AU   /* LF */

/* Payload = ICM_SyncFrame_t = 512 байт */
#define UART_PAYLOAD_BYTES      ((uint32_t)sizeof(ICM_SyncFrame_t))

/* Полный размер пакета: 2 + 512 + 2 + 2 = 518 байт */
#define UART_PKT_TOTAL_BYTES    (2U + UART_PAYLOAD_BYTES + 2U + 2U)

/* Смещения внутри TX-буфера */
#define UART_OFFSET_HEADER      0U
#define UART_OFFSET_PAYLOAD     2U
#define UART_OFFSET_CRC         (UART_OFFSET_PAYLOAD + UART_PAYLOAD_BYTES)
#define UART_OFFSET_FOOTER      (UART_OFFSET_CRC + 2U)

/* ================================================================
 * Публичные функции
 * ================================================================ */

/**
 * @brief  Инициализация модуля.
 *         Разрешает TC-прерывание DMA1 Stream1 и DMA-режим TX USART1.
 *         Вызывать один раз после MX_USART1_UART_Init().
 */
void UART_Telemetry_Init(void);

/**
 * @brief  [ОТЛАДКА] Собирает ICM_SyncFrame_t из samples[count-1]
 *         каждого батча g_sensor_batches[] и запускает DMA TX.
 *
 *         Если DMA ещё занят предыдущей передачей — кадр пропускается
 *         (цена записывается в g_uart_drop_count).
 *
 *         Вызывать из main-loop ПОСЛЕ ICM_ParseAllFIFO().
 */
void UART_BuildAndSendSyncFrame(void);

/**
 * @brief  Вызывается из DMA1_Stream1_IRQHandler при завершении TX.
 *         Сбрасывает флаг занятости и отключает DMA-поток.
 */
void UART_DMA_TxComplete(void);

/**
 * @brief  Счётчик пропущенных кадров (DMA был занят).
 *         Доступен извне для отладки (breakpoint / watch).
 */
extern volatile uint32_t g_uart_drop_count;

#ifdef __cplusplus
}
#endif

#endif /* UART_TELEMETRY_H */
