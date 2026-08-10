/**
 * @file uart_telemetry.h
 * @brief Телеметрия USART1 (RS-485) через DMA с ring-buffer буферизацией TX-кадров.
 *
 * UART wire-формат, 348 байт (БЕЗ ИЗМЕНЕНИЙ):
 *
 *  Смещение  Размер  Содержимое
 *  --------  ------  -----------------------------------------------
 *  0..1      2       Header: 0xAA 0x55
 *  2..3      2       frame_counter, uint16_t Little Endian
 *  4..345    342     S00..S17: 18 x 19-byte HIRES IMU block
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
 * [ИЗМЕНЕНИЯ v2] Ping-pong (2 буфера) заменён на ring-buffer из
 * UART_TX_QUEUE_DEPTH слотов.
 *
 * Причина: раньше в UART уходил только ПОСЛЕДНИЙ сэмпл из батча
 * FIFO (16 сэмплов @ 1600 Гц, собранных за один цикл опроса TIM6
 * @ 100 Гц) -> реальная частота телеметрии была 100 Гц (каждый
 * 16-й пакет данных).
 *
 * Теперь UART_BuildAndSendSyncFrame() пакует и ставит в очередь ВСЕ
 * ICM_FIFO_POLL_PACKETS (16) сэмплов из батча как 16 отдельных
 * UART-кадров -> реальная частота телеметрии 100 Гц * 16 = 1600 Гц,
 * то есть 1600 полных пакетов данных в секунду.
 *
 * Расчёт по времени (баланс):
 *   1 кадр (348 байт) при 8.1 Mbaud: 348*10/8.1e6 ~= 344 мкс.
 *   16 кадров подряд: 16 * 344 мкс ~= 5.50 мс.
 *   Период между циклами опроса (TIM6 @ 100 Гц): 10 мс.
 *   Запас: 10 мс - 5.50 мс = 4.5 мс (DMA гарантированно "выгребает"
 *   всю очередь до прихода следующего батча).
 *
 * Глубина очереди UART_TX_QUEUE_DEPTH = 32 (2x ICM_FIFO_POLL_PACKETS)
 * даёт запас на overlap: producer может начать заполнять очередь
 * для СЛЕДУЮЩЕГО батча, пока DMA ещё "довыгребает" предыдущий.
 *
 * UART_BuildAndSendSyncFrame() и UART_DMA_TxComplete() сохраняют
 * прежние сигнатуры -- вызовы в main.c/IRQ НЕ меняются.
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
 * Ring-buffer TX очередь.
 *
 * [ИЗМЕНЕНИЕ v2] Глубина = 2 * ICM_FIFO_POLL_PACKETS, чтобы вместить
 * полный батч из 16 кадров плюс запас на overlap со следующим
 * циклом опроса. Один слот = один 348-байтный UART-кадр.
 *
 * 32 буфера x 348 bytes = 11136 bytes D2 SRAM
 * (было 696 bytes при ping-pong depth=2).
 */
#define UART_TX_QUEUE_DEPTH   (2U * ICM_FIFO_POLL_PACKETS)   /* 32 */

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
 * Обнуляет ring-buffer и внутреннее состояние очереди.
 * Вызывать после MX_USART1_UART_Init().
 */
void UART_Telemetry_Init(void);

/**
 * @brief Формирует ВСЕ ICM_FIFO_POLL_PACKETS (16) кадров из батча FIFO
 *        и ставит их в очередь на отправку.
 *
 * Вызывать после ICM_ParseAllFIFO(), один раз за цикл опроса (100 Гц).
 *
 * [ИЗМЕНЕНИЕ v2] Раньше отправлялся только последний сэмпл батча
 * (samples[count-1]) -> 1 кадр за цикл -> 100 кадров/с. Теперь
 * отправляются ВСЕ samples[0..count-1] как отдельные кадры ->
 * до 16 кадров за цикл -> 1600 кадров/с.
 *
 * Логика:
 *  - Для каждого валидного сэмпла в батче строится отдельный
 *    348-байтный кадр и кладётся в очередь ring-buffer.
 *  - Если DMA простаивает -- передача первого кадра из очереди
 *    запускается немедленно.
 *  - Остальные кадры передаются последовательно из DMA TC IRQ.
 */
void UART_BuildAndSendSyncFrame(void);

/**
 * @brief Вызывать из DMA1_Stream1_IRQHandler() при DMA TC.
 *
 * Если в очереди есть ещё непереданные кадры -- немедленно
 * запускает передачу следующего. Иначе DMA остаётся в состоянии
 * idle до следующего вызова UART_BuildAndSendSyncFrame().
 */
void UART_DMA_TxComplete(void);

/* ================================================================
 * Debug counters
 *
 * g_uart_drop_count теперь ИМЕЕТ смысл: если очередь переполнится
 * (что не должно происходить при штатной работе -- см. расчёт
 * времени в комментарии к файлу), новые кадры будут отбрасываться,
 * а счётчик увеличиваться. Мониторинг g_uart_drop_count в отладке
 * подтверждает отсутствие потерь.
 *
 * g_uart_queue_count -- текущая глубина очереди (0..UART_TX_QUEUE_DEPTH).
 * g_uart_queue_high_watermark -- максимум за всё время работы,
 * полезно для проверки, что запас по времени (см. выше) достаточен.
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
