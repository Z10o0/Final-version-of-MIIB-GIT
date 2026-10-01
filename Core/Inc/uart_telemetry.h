/*
 * Телеметрия USART1 / RS-485 через DMA и TX ring buffer.
 *
 * Один TIM6-батч @ 400 Гц -> один усреднённый кадр 690 байт.
 *
 * USART1: 12 Mbaud, 8N1.
 * Время кадра: 690 * 10 / 12e6 = 575 мкс.
 * Период кадра: 2500 мкс.
 * Загрузка линии: 23%.
 *
 * Wire format:
 * [0..1]     Header AA 55.
 * [2..3]     uint16 frame counter, Little Endian.
 * [4..687]   36 IMU-блоков по 19 байт.
 * [688..689] CRC16-CCITT, Little Endian.
 *
 * CRC считается по bytes [2..687].
 *
 * IMU-блок:
 * [0]  Ax[19:12]       [10] Gz[19:12]
 * [1]  Ax[11:4]        [11] Gz[11:4]
 * [2]  Ay[19:12]       [12] temp_raw[15:8]
 * [3]  Ay[11:4]        [13] temp_raw[7:0]
 * [4]  Az[19:12]       [14] timestamp[15:8]
 * [5]  Az[11:4]        [15] timestamp[7:0]
 * [6]  Gx[19:12]       [16] Ax[3:0] | Gx[3:0]
 * [7]  Gx[11:4]        [17] Ay[3:0] | Gy[3:0]
 * [8]  Gy[19:12]       [18] Az[3:0] | Gz[3:0]
 * [9]  Gy[11:4]
 *
 * Оси содержат усреднённые знаковые HIRES20-значения.
 * Temperature также усредняется.
 * Timestamp относится к последнему raw sample окна.
 *
 * TX queue находится в .RAM_D1_DMA / AXI SRAM.
 * Перед каждым запуском UART DMA выполняется D-cache clean.
 */

#ifndef UART_TELEMETRY_H
#define UART_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "icm45686_data.h"
#include "icm45686_config.h"

/* Wire protocol constants: формат сохранён. */
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

#define UART_OFFSET_CRC \
    (UART_HEADER_BYTES + UART_PAYLOAD_BYTES)

/*
 * Независимая глубина очереди.
 * Не связана с числом raw samples в FIFO-батче.
 *
 * Восемь слотов по 690 байт = 5520 байт.
 * Активный DMA-кадр занимает один из этих слотов.
 */
#define UART_TX_QUEUE_DEPTH   8U

_Static_assert(UART_TX_QUEUE_DEPTH >= 2U,
               "UART TX queue must contain at least two frames");

_Static_assert(UART_PAYLOAD_BYTES == 686U,
               "Unexpected UART payload size: expected 2 + 36*19 = 686");

_Static_assert(UART_PKT_TOTAL_BYTES == 690U,
               "Unexpected UART packet size: expected 690");

/* Public API сохранён. */
void UART_Telemetry_Init(void);
void UART_BuildAndSendSyncFrame(void);
void UART_DMA_TxComplete(void);

/* Debug counters. */
extern volatile uint32_t g_uart_drop_count;
extern volatile uint32_t g_uart_build_count;
extern volatile uint32_t g_uart_enqueue_count;
extern volatile uint32_t g_uart_dma_start_count;
extern volatile uint32_t g_uart_dma_tc_count;

extern volatile uint8_t g_uart_queue_high_watermark;
extern volatile uint8_t g_uart_queue_count;

extern volatile uint32_t g_uart_dma_te_count;
extern volatile uint32_t g_uart_dma_dme_count;
extern volatile uint32_t g_uart_dma_fe_count;

#ifdef __cplusplus
}
#endif

#endif /* UART_TELEMETRY_H */
