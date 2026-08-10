/**
 * @file    uart_telemetry.h
 * @brief   Телеметрия по USART1 (RS-485) через DMA, LL-драйвер, без HAL.
 *
 * -----------------------------------------------------------------------
 * UART wire-формат (348 байт):
 *
 *  Смещение  Размер  Содержимое
 *  --------  ------  ------------------------------------------
 *     0        1     Заголовок: 0xAA
 *     1        1     Заголовок: 0x55
 *     2..3     2     frame_counter  (uint16_t, Little Endian)
 *     4..345 342     18 × 19 байт — данные S00..S17
 *   346..347   2     CRC16-CCITT (uint16_t, Little Endian)
 *                    считается по байтам [2..345] (344 байта payload)
 *  --------  ------
 *  ИТОГО: 348 байт
 *
 *  Примечание: footer 0x0D 0x0A и sensor_mask УДАЛЕНЫ.
 *
 * -----------------------------------------------------------------------
 * Формат одного IMU-блока (19 байт, offset внутри блока 0-based):
 *
 *  Байт  Содержимое
 *  ----  ------------------------------------------------
 *   0    Ax[19:12]
 *   1    Ax[11:4]
 *   2    Ay[19:12]
 *   3    Ay[11:4]
 *   4    Az[19:12]
 *   5    Az[11:4]
 *   6    Gx[19:12]
 *   7    Gx[11:4]
 *   8    Gy[19:12]
 *   9    Gy[11:4]
 *  10    Gz[19:12]
 *  11    Gz[11:4]
 *  12    temp_raw[15:8]   (Big Endian)
 *  13    temp_raw[7:0]
 *  14    timestamp[15:8]  (Big Endian)
 *  15    timestamp[7:0]
 *  16    (Ax[3:0] << 4) | Gx[3:0]   — младшие nibble X
 *  17    (Ay[3:0] << 4) | Gy[3:0]   — младшие nibble Y
 *  18    (Az[3:0] << 4) | Gz[3:0]   — младшие nibble Z
 *
 *  Начало блока датчика N (0-based) в UART буфере:
 *    imu_offset = UART_OFFSET_SAMPLES + N * UART_IMU_WIRE_BYTES
 *
 * -----------------------------------------------------------------------
 * Бюджет UART:
 *   348 bytes × 10 UART bits/byte = 3480 bits/frame.
 *   At 1600 frames/s: 5.568 Mbit/s.
 *   At 8 Mbaud UART:  reserve = 2.432 Mbit/s.
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
 * Константы wire-протокола
 * ================================================================ */
#define UART_PKT_HEADER_0       0xAAU
#define UART_PKT_HEADER_1       0x55U

#define UART_SENSOR_COUNT       18U
#define UART_IMU_WIRE_BYTES     19U
#define UART_COUNTER_BYTES      2U
#define UART_HEADER_BYTES       2U
#define UART_CRC_BYTES          2U

/* Payload = frame_counter (2) + 18 × 19 = 344 байта — именно по нему CRC */
#define UART_PAYLOAD_BYTES \
    (UART_COUNTER_BYTES + UART_SENSOR_COUNT * UART_IMU_WIRE_BYTES)

/* Полный пакет: header(2) + payload(344) + CRC(2) = 348 байт */
#define UART_PKT_TOTAL_BYTES \
    (UART_HEADER_BYTES + UART_PAYLOAD_BYTES + UART_CRC_BYTES)

/* Смещения внутри TX-буфера (0-based, в байтах) */
#define UART_OFFSET_HEADER      0U
#define UART_OFFSET_COUNTER     2U
#define UART_OFFSET_SAMPLES     4U
#define UART_OFFSET_CRC         346U

/* ================================================================
 * Compile-time проверки
 * ================================================================ */
_Static_assert(UART_PAYLOAD_BYTES == 344U,
               "Unexpected UART payload size");

_Static_assert(UART_PKT_TOTAL_BYTES == 348U,
               "Unexpected UART packet size");

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
 * @brief  Собирает 348-байтный кадр из samples[count-1] каждого батча
 *         g_sensor_batches[] в формате 19-byte HIRES и запускает DMA TX.
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
