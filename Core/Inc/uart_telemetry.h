/**
 * @file uart_telemetry.h
 * @brief Телеметрия USART1 (RS-485) через DMA с ring-buffer
 *        буферизацией TX-кадров.
 *
 * UART wire-формат, 690 байт:
 *
 *  Смещение  Размер  Содержимое
 *  --------  ------  -----------------------------------------------
 *  0..1      2       Header: 0xAA 0x55
 *  2..3      2       frame_counter, uint16_t Little Endian
 *  4..687    684     S00..S35: 36 x 19-byte HIRES IMU block
 *  688..689  2       CRC16-CCITT Little Endian по bytes [2..687]
 *
 *  Полный размер: 2 + 2 + 36 * 19 + 2 = 690 bytes.
 *  Один кадр содержит один усреднённый output sample 400 Гц от каждого
 *  из 36 датчиков (среднее 8 raw HIRES-отсчётов при ODR 3200 Гц).
 *  Wire format и CRC не изменены.
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
 *  timestamp в усреднённом кадре — timestamp последнего из восьми
 *  raw-пакетов окна (диагностическое поле, не усредняется).
 *
 * Один TIM6-батч @ 400 Гц -> один усреднённый RS-кадр 690 байт.
 * USART1 = 12 Mbaud, 8N1.
 * Время кадра: 690 * 10 / 12e6 = 575 мкс.
 * Период кадра: 2500 мкс.
 * Загрузка линии: 23%.
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

#define UART_SENSOR_COUNT     36U
#define UART_IMU_WIRE_BYTES   19U
#define UART_COUNTER_BYTES    2U
#define UART_HEADER_BYTES     2U
#define UART_CRC_BYTES        2U

#define UART_PAYLOAD_BYTES \
    (UART_COUNTER_BYTES + UART_SENSOR_COUNT * UART_IMU_WIRE_BYTES)

#define UART_PKT_TOTAL_BYTES \
    (UART_HEADER_BYTES + UART_PAYLOAD_BYTES + UART_CRC_BYTES)

#define UART_OFFSET_HEADER    0U
#define UART_OFFSET_COUNTER   2U
#define UART_OFFSET_SAMPLES   4U
#define UART_OFFSET_CRC       (UART_HEADER_BYTES + UART_PAYLOAD_BYTES)

/*
 * Глубина TX-очереди не зависит от размера FIFO-батча: на один батч
 * формируется ровно один кадр. 8 кадров = 20 мс запаса при 400 Гц.
 * Активный DMA-слот в глубину очереди входит.
 */
#define UART_TX_QUEUE_DEPTH   8U

_Static_assert(UART_PAYLOAD_BYTES == 686U,
               "Unexpected UART payload size: expected 2 + 36*19 = 686");
_Static_assert(UART_PKT_TOTAL_BYTES == 690U,
               "Unexpected UART packet size: expected 690");
_Static_assert(UART_TX_QUEUE_DEPTH >= 2U,
               "UART TX queue must contain at least two frames");
_Static_assert(UART_SENSOR_COUNT == ICM_TOTAL_SENSORS,
               "UART sensor count must match ICM sensor topology");

/* ================================================================
 * Public API
 * ================================================================ */
void UART_Telemetry_Init(void);

/* Один вызов создаёт один усреднённый output frame (400 Гц). */
void UART_BuildAndSendSyncFrame(void);

/* Вызывать только из DMA1_Stream1_IRQHandler() после очистки TC. */
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

/* Профилирование построения кадра (DWT) */
extern volatile uint32_t g_uart_build_cyc_last;
extern volatile uint32_t g_uart_build_cyc_max;
extern volatile uint32_t g_uart_build_us_last;
extern volatile uint32_t g_uart_build_us_max;

#ifdef __cplusplus
}
#endif

#endif /* UART_TELEMETRY_H */
