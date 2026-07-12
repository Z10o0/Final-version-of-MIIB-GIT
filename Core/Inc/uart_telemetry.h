/**
 * @file    uart_telemetry.h
 * @brief   Телеметрия по USART1 через DMA.
 *
 *          Структура пакета (бинарный протокол):
 *          ┌──────────┬─────────┬───────────────────────────┬──────────┬───────────┐
 *          │ Заголовок│ Счётчик │ Данные 18 датчиков        │ CRC16    │ Конец     │
 *          │ 0xAA 0x55│ uint32  │ ICM_SensorBatch_t x18     │ uint16_t │ 0x0D 0x0A │
 *          │ 2 байта  │ 4 байта │ см. расчёт UART_PAYLOAD_  │ 2 байта  │ 2 байта   │
 *          └──────────┴─────────┴───────────────────────────┴──────────┴───────────┘
 *
 *          Передача полностью неблокирующая: DMA1 Stream1 (USART1_TX).
 *          В main-loop вызывать UART_SendBatch() — если предыдущая передача
 *          ещё не завершена, вызов игнорируется (пакет теряется, не подвисает).
 *
 *          HAL не используется. Только LL + прямая работа с регистрами.
 */

#ifndef UART_TELEMETRY_H
#define UART_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "icm45686_data.h"

/* ================================================================
 * Константы пакета
 * ================================================================ */
#define UART_PKT_HEADER_0   0xAAU   /* Первый байт заголовка  */
#define UART_PKT_HEADER_1   0x55U   /* Второй байт заголовка  */
#define UART_PKT_FOOTER_0   0x0DU   /* CR                     */
#define UART_PKT_FOOTER_1   0x0AU   /* LF                     */

/* Размер полезной нагрузки данных (18 датчиков × один ICM_SensorBatch_t) */
/* Один ICM_SensorBatch_t: ICM_FIFO_POLL_PACKETS * sizeof(ICM_Sample_t) + 2 */
/* Рассчитывается компилятором через sizeof(g_sensor_batches) */
#define UART_PAYLOAD_BYTES  (sizeof(g_sensor_batches))

/* Полный размер пакета:
   2 (header) + 4 (counter) + payload + 2 (CRC16) + 2 (footer) */
#define UART_PKT_TOTAL_BYTES    (2U + 4U + UART_PAYLOAD_BYTES + 2U + 2U)

/* ================================================================
 * Публичные функции
 * ================================================================ */

/**
 * @brief  Инициализация модуля телеметрии.
 *         Настраивает прерывание DMA1 Stream1 (USART1_TX TC).
 *         Вызывать один раз после MX_USART1_UART_Init().
 */
void UART_Telemetry_Init(void);

/**
 * @brief  Формирует пакет из текущих данных g_sensor_batches и
 *         запускает DMA-передачу по USART1.
 *         Если DMA занят (предыдущая передача не завершена) —
 *         вызов немедленно возвращается без действий.
 *         Вызывать из main-loop после ICM_ParseAllFIFO().
 */
void UART_SendBatch(void);

/**
 * @brief  Вызывается из DMA1_Stream1_IRQHandler при завершении TX.
 *         Сбрасывает флаг занятости и деактивирует DMA-поток.
 */
void UART_DMA_TxComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_TELEMETRY_H */
