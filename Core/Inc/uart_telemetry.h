/**
 * @file    uart_telemetry.h
 * @brief   Телеметрия USART1 (RS-485) через DMA с очередью TX-кадров.
 *
 * UART wire-формат, 348 байт:
 *
 *  Смещение  Размер  Содержимое
 *  --------  ------  -----------------------------------------------
 *     0..1     2     Header: 0xAA 0x55
 *     2..3     2     frame_counter, uint16_t Little Endian
 *     4..345 342     S00..S17: 18 × 19-byte HIRES IMU block
 *   346..347   2     CRC16-CCITT Little Endian по bytes [2..345]
 *
 * Полный размер: 2 + 2 + 18 * 19 + 2 = 348 bytes.
 *
 * Формат одного 19-byte IMU block:
 *
 *   [0]  Ax[19:12]               [10] Gz[19:12]
 *   [1]  Ax[11:4]                [11] Gz[11:4]
 *   [2]  Ay[19:12]               [12] temp_raw[15:8]
 *   [3]  Ay[11:4]                [13] temp_raw[7:0]
 *   [4]  Az[19:12]               [14] timestamp[15:8]
 *   [5]  Az[11:4]                [15] timestamp[7:0]
 *   [6]  Gx[19:12]               [16] Ax[3:0] | Gx[3:0]
 *   [7]  Gx[11:4]                [17] Ay[3:0] | Gy[3:0]
 *   [8]  Gy[19:12]               [18] Az[3:0] | Gz[3:0]
 *   [9]  Gy[11:4]
 *
 * temp_raw и timestamp: Big Endian внутри IMU block.
 * frame_counter и CRC: Little Endian.
 *
 * Footer и sensor_mask отсутствуют.
 *
 * UART budget:
 *   348 bytes × 10 UART bits/byte = 3480 bits/frame.
 *   At 1600 frames/s: 5.568 Mbit/s.
 *   At 8 Mbaud UART: reserve = 2.432 Mbit/s.
 */

#ifndef UART_TELEMETRY_H
#define UART_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "icm45686_data.h"
#include "icm45686_config.h"

/* ================================================================
 * Wire protocol constants
 * ================================================================ */
#define UART_PKT_HEADER_0       0xAAU
#define UART_PKT_HEADER_1       0x55U

#define UART_SENSOR_COUNT       18U
#define UART_IMU_WIRE_BYTES     19U
#define UART_COUNTER_BYTES      2U
#define UART_HEADER_BYTES       2U
#define UART_CRC_BYTES          2U

#define UART_PAYLOAD_BYTES \
    (UART_COUNTER_BYTES + UART_SENSOR_COUNT * UART_IMU_WIRE_BYTES)

#define UART_PKT_TOTAL_BYTES \
    (UART_HEADER_BYTES + UART_PAYLOAD_BYTES + UART_CRC_BYTES)

#define UART_OFFSET_HEADER      0U
#define UART_OFFSET_COUNTER     2U
#define UART_OFFSET_SAMPLES     4U
#define UART_OFFSET_CRC         346U

/*
 * Очередь кадров TX.
 *
 * Значение обязательно является степенью двойки, поскольку индексы
 * ring-buffer инкрементируются через mask UART_TX_QUEUE_MASK.
 *
 * 8 кадров × 348 bytes = 2784 bytes D2 SRAM.
 * При частоте 1600 Hz это до 5 ms кратковременного запаса.
 */
#define UART_TX_QUEUE_DEPTH     8U
#define UART_TX_QUEUE_MASK      (UART_TX_QUEUE_DEPTH - 1U)

_Static_assert(UART_PAYLOAD_BYTES == 344U,
               "Unexpected UART payload size");

_Static_assert(UART_PKT_TOTAL_BYTES == 348U,
               "Unexpected UART packet size");

_Static_assert((UART_TX_QUEUE_DEPTH & UART_TX_QUEUE_MASK) == 0U,
               "UART_TX_QUEUE_DEPTH must be a power of two");

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief Инициализация USART1 TX DMA telemetry.
 *
 * Разрешает DMA1 Stream1 TC interrupt и USART1 TX DMA request.
 * Вызывать после MX_USART1_UART_Init().
 */
void UART_Telemetry_Init(void);

/**
 * @brief Формирует кадр из последних FIFO samples и помещает его в TX queue.
 *
 * Вызывать после ICM_ParseAllFIFO().
 *
 * Если очередь не заполнена:
 * - кадр помещается в очередь;
 * - если DMA простаивает, немедленно запускается передача.
 *
 * Если очередь заполнена:
 * - новый кадр отбрасывается;
 * - увеличивается g_uart_drop_count.
 */
void UART_BuildAndSendSyncFrame(void);

/**
 * @brief Вызывать из DMA1_Stream1_IRQHandler() при DMA TC.
 *
 * Завершает текущий пакет и, если очередь не пуста, запускает
 * передачу следующего пакета без ожидания main-loop.
 */
void UART_DMA_TxComplete(void);

/* ================================================================
 * Debug counters
 * ================================================================ */

/* Число отброшенных кадров из-за полного TX ring-buffer. */
extern volatile uint32_t g_uart_drop_count;

/* Число вызовов UART_BuildAndSendSyncFrame(). */
extern volatile uint32_t g_uart_build_count;

/* Число кадров, принятых в очередь. */
extern volatile uint32_t g_uart_enqueue_count;

/* Число DMA-передач, реально запущенных. */
extern volatile uint32_t g_uart_dma_start_count;

/* Число DMA transfer-complete IRQ. */
extern volatile uint32_t g_uart_dma_tc_count;

/* Максимальная фактически достигнутая глубина очереди. */
extern volatile uint8_t g_uart_queue_high_watermark;

/* Текущая глубина очереди: 0...UART_TX_QUEUE_DEPTH. */
extern volatile uint8_t g_uart_queue_count;

/* Ошибки DMA1 Stream1 для USART1 TX. */
extern volatile uint32_t g_uart_dma_te_count;
extern volatile uint32_t g_uart_dma_dme_count;
extern volatile uint32_t g_uart_dma_fe_count;

#ifdef __cplusplus
}
#endif

#endif /* UART_TELEMETRY_H */
