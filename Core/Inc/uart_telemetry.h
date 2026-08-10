/**
 * @file uart_telemetry.h
 * @brief Телеметрия USART1 (RS-485) через DMA с ping-pong буферизацией TX-кадров.
 *
 * UART wire-формат, 348 байт (БЕЗ ИЗМЕНЕНИЙ):
 *
 *  Смещение  Размер  Содержимое
 *  --------  ------  -----------------------------------------------
 *  0..1      2       Header: 0xAA 0x55
 *  2..3      2       frame_counter, uint16_t Little Endian
 *  4..345    342     S00..S17: 18 × 19-byte HIRES IMU block
 *  346..347  2       CRC16-CCITT Little Endian по bytes [2..345]
 *
 *  Полный размер: 2 + 2 + 18 * 19 + 2 = 348 bytes.
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
 * [ИЗМЕНЕНИЯ] Ring-buffer (8 слотов) заменён на ping-pong (2 буфера).
 *
 * Причина: producer теперь вызывается раз в 10 мс (100 Гц) вместо
 * каждые 0.625 мс (1600 Гц). При 8.1 Mbaud передача 348-байтного
 * кадра занимает ~344 мкс — DMA гарантированно успевает освободиться
 * задолго до следующего кадра. Глубокая очередь на 8 слотов больше
 * не нужна: достаточно двух буферов (один передаётся DMA, второй
 * заполняется producer'ом).
 *
 * UART_BuildAndSendSyncFrame() и UART_DMA_TxComplete() сохраняют
 * прежние сигнатуры — вызовы в main.c/IRQ не меняются.
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
 * Wire protocol constants (без изменений)
 * ================================================================ */
#define UART_PKT_HEADER_0     0xAAU
#define UART_PKT_HEADER_1     0x55U

#define UART_SENSOR_COUNT     18U
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
#define UART_OFFSET_CRC       346U

/*
 * Ping-pong буферизация TX.
 *
 * Ровно 2 буфера: один активно передаётся DMA (read-буфер),
 * второй в это время заполняется producer'ом (write-буфер).
 * Роли переключаются в DMA TC IRQ.
 *
 * 2 буфера × 348 bytes = 696 bytes D2 SRAM (было 2784 bytes при
 * ring-buffer depth=8).
 */
#define UART_PINGPONG_BUFFERS  2U

_Static_assert(UART_PAYLOAD_BYTES == 344U,
               "Unexpected UART payload size");

_Static_assert(UART_PKT_TOTAL_BYTES == 348U,
               "Unexpected UART packet size");

/* ================================================================
 * Public API (сигнатуры не изменены)
 * ================================================================ */

/**
 * @brief Инициализация USART1 TX DMA telemetry.
 *
 * Разрешает DMA1 Stream1 TC interrupt и USART1 TX DMA request.
 * Обнуляет оба ping-pong буфера и внутреннее состояние.
 * Вызывать после MX_USART1_UART_Init().
 */
void UART_Telemetry_Init(void);

/**
 * @brief Формирует кадр из последних FIFO samples и отправляет его.
 *
 * Вызывать после ICM_ParseAllFIFO(), один раз за цикл опроса (100 Гц).
 *
 * Логика:
 *  - Пакет строится в текущем write-буфере.
 *  - Если DMA простаивает — передача запускается немедленно, buffer
 *    роли переключаются.
 *  - Если DMA активен — новый кадр помечается готовым (s_pp_ready=1)
 *    и будет передан автоматически из DMA TC IRQ.
 *
 * При частоте вызова 100 Гц и времени передачи ~344 мкс на 8.1 Mbaud
 * DMA гарантированно освобождается к следующему вызову — потерь
 * кадров не предполагается.
 */
void UART_BuildAndSendSyncFrame(void);

/**
 * @brief Вызывать из DMA1_Stream1_IRQHandler() при DMA TC.
 *
 * Если producer успел подготовить новый кадр (s_pp_ready==1) —
 * немедленно запускает передачу этого кадра. Иначе DMA остаётся
 * в состоянии idle до следующего вызова UART_BuildAndSendSyncFrame().
 */
void UART_DMA_TxComplete(void);

/* ================================================================
 * Debug counters
 *
 * g_uart_drop_count / g_uart_queue_high_watermark / g_uart_queue_count
 * оставлены как заглушки нулевой семантики для совместимости с уже
 * существующим кодом (например, диагностическими принтами в main.c),
 * который мог на них ссылаться. В ping-pong схеме кадры не отбрасы-
 * ваются и глубина очереди всегда 0..1, поэтому эти счётчики больше
 * не несут диагностической ценности, но публичный ABI сохранён.
 * ================================================================ */
extern volatile uint32_t g_uart_drop_count;
extern volatile uint32_t g_uart_build_count;
extern volatile uint32_t g_uart_enqueue_count;
extern volatile uint32_t g_uart_dma_start_count;
extern volatile uint32_t g_uart_dma_tc_count;
extern volatile uint8_t  g_uart_queue_high_watermark;
extern volatile uint8_t  g_uart_queue_count;

/* Ошибки DMA1 Stream1 для USART1 TX (без изменений). */
extern volatile uint32_t g_uart_dma_te_count;
extern volatile uint32_t g_uart_dma_dme_count;
extern volatile uint32_t g_uart_dma_fe_count;

#ifdef __cplusplus
}
#endif

#endif /* UART_TELEMETRY_H */
