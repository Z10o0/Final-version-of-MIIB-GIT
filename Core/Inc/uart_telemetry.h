/**
 * @file uart_telemetry.h
 * @brief Телеметрия USART1 (RS-485) через DMA с ring-buffer буферизацией TX-кадров.
 *
 * UART wire-формат, 690 байт:
 *
 *  Смещение  Размер  Содержимое
 *  --------  ------  -----------------------------------------------
 *  0..1      2       Header: 0xAA 0x55
 *  2..3      2       frame_counter, uint16_t Little Endian
 *  4..689    686     S00..S35: 36 x 19-byte HIRES IMU block
 *  688..689  2       CRC16-CCITT Little Endian по bytes [2..687]
 *
 *  Полный размер: 2 + 2 + 36 * 19 + 2 = 690 bytes.
 *
 *  Формат одного 19-byte IMU block:
 *
 *   [0]  Ax[19:12]        [10] Gz[19:12]
 *   [1]  Ax[11:4]         [11] Gz[11:4]
 *   [2]  Ay[19:12]        [12] temp_raw[15:8]
 *   [3]  Ay[11:4]         [13] temp_raw[7:0]
 *   [4]  Az[19:12]        [14] timestamp[15:8]
 *   [5]  Az[11:4]         [15] timestamp[7:0]
 *   [6]  Gx[19:12]        [16] Ax[3:0] | Gx[3:0]
 *   [7]  Gx[11:4]         [17] Ay[3:0] | Gy[3:0]
 *   [8]  Gy[19:12]        [18] Az[3:0] | Gz[3:0]
 *   [9]  Gy[11:4]
 *
 *  temp_raw и timestamp: Big Endian внутри IMU block.
 *  frame_counter и CRC: Little Endian.
 *
 * ------------------------------------------------------------------
 * [ИЗМЕНЕНИЯ v2] Ping-pong заменён на ring-buffer из
 * UART_TX_QUEUE_DEPTH слотов. 36 датчиков (6 SPI × 6).
 *
 * Расчёт по времени (баланс) при 8.1 Mbaud:
 *   1 кадр (690 байт): 690*10/8.1e6 ~= 852 мкс.
 *   16 кадров подряд: 16 * 852 мкс ~= 13.6 мс.
 *
 * ВНИМАНИЕ: 13.6 мс > 10 мс (период TIM6 @ 100 Гц).
 * При текущем baudrate 8.1 Mbaud и 36 датчиках UART не успевает
 * выгрести 16 кадров за один период! Требуется либо:
 *   а) baudrate >= 690*10*16*100 = 110.4 Mbit/s (нереально для RS-485)
 *   б) уменьшить ICM_FIFO_POLL_PACKETS до 8 -> 6.8 мс < 10 мс
 *   в) уменьшить UART_TX_QUEUE_DEPTH до запаса по расчёту
 * Документируется здесь для явного контроля разработчиком.
 * ------------------------------------------------------------------
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
#define UART_PKT_HEADER_0     0xAAU
#define UART_PKT_HEADER_1     0x55U

#define UART_SENSOR_COUNT     36U          /* 6 шин × 6 датчиков       */
#define UART_IMU_WIRE_BYTES   19U
#define UART_COUNTER_BYTES    2U
#define UART_HEADER_BYTES     2U
#define UART_CRC_BYTES        2U

#define UART_PAYLOAD_BYTES \
    (UART_COUNTER_BYTES + UART_SENSOR_COUNT * UART_IMU_WIRE_BYTES)
/* = 2 + 36*19 = 686 */

#define UART_PKT_TOTAL_BYTES \
    (UART_HEADER_BYTES + UART_PAYLOAD_BYTES + UART_CRC_BYTES)
/* = 2 + 686 + 2 = 690 */

#define UART_OFFSET_HEADER    0U
#define UART_OFFSET_COUNTER   2U
#define UART_OFFSET_SAMPLES   4U
#define UART_OFFSET_CRC       (UART_HEADER_BYTES + UART_PAYLOAD_BYTES)
/* = 688U — вычисляется из макросов, не хардкодится */

/*
 * Ring-buffer TX очередь.
 * UART_TX_QUEUE_DEPTH = ICM_FIFO_POLL_PACKETS = 16 слотов.
 * 16 x 690 bytes = 11 040 bytes D2 SRAM.
 *
 * Timing @ 12 Mbaud:
 *   16 кадров × 575 мкс = 9 200 мкс < 10 000 мкс (период TIM6).
 *   Запас: 800 мкс. Overlap не нужен — DMA выгребает батч до
 *   прихода следующего цикла опроса.
 *
 * Итоговый RAM_D2: 13 161 (фикс.) + 11 040 (queue) = 24 201 байт.
 */
#define UART_TX_QUEUE_DEPTH   ICM_FIFO_POLL_PACKETS   /* 16 */

_Static_assert(UART_PAYLOAD_BYTES == 686U,
               "Unexpected UART payload size: expected 2 + 36*19 = 686");

_Static_assert(UART_PKT_TOTAL_BYTES == 690U,
               "Unexpected UART packet size: expected 690");

/* ================================================================
 * Public API (сигнатуры не изменены)
 * ================================================================ */
void UART_Telemetry_Init(void);
void UART_BuildAndSendSyncFrame(void);
void UART_DMA_TxComplete(void);

/* ================================================================
 * Debug counters
 * ================================================================ */
extern volatile uint32_t g_uart_drop_count;
extern volatile uint32_t g_uart_build_count;
extern volatile uint32_t g_uart_enqueue_count;
extern volatile uint32_t g_uart_dma_start_count;
extern volatile uint32_t g_uart_dma_tc_count;
extern volatile uint8_t  g_uart_queue_high_watermark;
extern volatile uint8_t  g_uart_queue_count;

extern volatile uint32_t g_uart_dma_te_count;
extern volatile uint32_t g_uart_dma_dme_count;
extern volatile uint32_t g_uart_dma_fe_count;

#ifdef __cplusplus
}
#endif

#endif /* UART_TELEMETRY_H */
