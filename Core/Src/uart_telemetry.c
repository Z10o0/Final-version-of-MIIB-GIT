/**
 * @file uart_telemetry.c
 * @brief USART1 telemetry through DMA1 Stream1 with TX ping-pong buffering.
 *
 * ------------------------------------------------------------------
 * [ИЗМЕНЕНИЯ] Ring-buffer (8 слотов) заменён на ping-pong (2 буфера).
 *
 * main-loop @ 100 Hz:
 *   ICM_ParseAllFIFO()
 *   UART_BuildAndSendSyncFrame()
 *         |
 *         v
 *   ping-pong[write_idx]  <-- producer пишет сюда
 *         |
 *         v (если DMA idle: сразу; иначе через TC IRQ)
 *   DMA1 Stream1 -> USART1 TX  <-- читает ping-pong[read_idx]
 *
 * При 8.1 Mbaud передача одного 348-байтного кадра занимает:
 *   348 bytes * 10 bits/byte / 8.1e6 bit/s ≈ 344 мкс.
 * Producer вызывается раз в 10 мс (100 Гц) — DMA гарантированно
 * успевает освободиться между вызовами с огромным запасом (10 мс vs
 * 344 мкс), поэтому двух буферов достаточно и потерь кадров не будет.
 * ------------------------------------------------------------------
 */

#include "uart_telemetry.h"
#include "icm45686_spi.h"
#include "main.h"

/* ================================================================
 * TX ping-pong buffers в Non-Cacheable D2 SRAM.
 *
 * DMA читает буфер непосредственно из ping-pong slot.
 * Поэтому cache-clean перед запуском DMA не требуется.
 *
 * 2 буфера × 348 bytes = 696 bytes D2 SRAM (было 2784 bytes при
 * ring-buffer depth=8).
 * ================================================================ */
static uint8_t g_uart_tx_ping_pong[UART_PINGPONG_BUFFERS][UART_PKT_TOTAL_BYTES]
    __attribute__((section(".RAM_D2"), aligned(4)));

/*
 * Состояние ping-pong:
 *
 * s_pp_write_idx — индекс буфера, в который сейчас (или в следующий
 *                  раз) пишет producer.
 * s_pp_ready     — флаг: producer записал новый кадр, который ещё не
 *                  подхвачен DMA.
 * s_dma_active   — DMA сейчас передаёт один из буферов.
 *
 * Producer:  main-loop (UART_BuildAndSendSyncFrame).
 * Consumer:  DMA TC IRQ (UART_DMA_TxComplete).
 *
 * Инвариант: пока s_dma_active==1, DMA читает буфер с индексом
 * (s_pp_write_idx ^ 1U) — то есть "не текущий write". Producer
 * никогда не пишет в буфер, который в данный момент передаёт DMA.
 */
static volatile uint8_t s_pp_write_idx = 0U;
static volatile uint8_t s_pp_ready     = 0U;
static volatile uint8_t s_dma_active   = 0U;

/* frame_counter переполняется modulo 65536. При 100 Hz это ~655.36 s. */
static uint16_t s_frame_counter = 0U;

/* ================================================================
 * Public diagnostics
 *
 * g_uart_drop_count / g_uart_queue_high_watermark / g_uart_queue_count
 * оставлены как заглушки нулевой семантики для совместимости с уже
 * существующим кодом (main.c может их читать/печатать). В ping-pong
 * схеме кадры не отбрасываются, поэтому g_uart_drop_count всегда 0,
 * а "глубина очереди" всегда 0 или 1 (g_uart_queue_count = s_pp_ready).
 * ================================================================ */
volatile uint32_t g_uart_drop_count           = 0U;
volatile uint32_t g_uart_build_count          = 0U;
volatile uint32_t g_uart_enqueue_count        = 0U;
volatile uint32_t g_uart_dma_start_count      = 0U;
volatile uint32_t g_uart_dma_tc_count         = 0U;
volatile uint8_t  g_uart_queue_high_watermark = 0U;
volatile uint8_t  g_uart_queue_count          = 0U;

volatile uint32_t g_uart_dma_te_count  = 0U;
volatile uint32_t g_uart_dma_dme_count = 0U;
volatile uint32_t g_uart_dma_fe_count  = 0U;

/* ================================================================
 * CRC16-CCITT: polynomial 0x1021, init 0xFFFF. (без изменений)
 * ================================================================ */
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
 * Извлечение 20 полезных бит из sign-extended int32_t. (без изменений)
 * ================================================================ */
static inline uint32_t UART_ToU20(int32_t value)
{
    return ((uint32_t)value) & 0x000FFFFFUL;
}

/* ================================================================
 * Упаковка одного ICM_Sample_t в независимый от ABI 19-byte формат.
 * (без изменений)
 * ================================================================ */
static void UART_PackIMU19(uint8_t dst[UART_IMU_WIRE_BYTES],
                            const ICM_Sample_t *src)
{
    const uint32_t ax = UART_ToU20(src->accel_x);
    const uint32_t ay = UART_ToU20(src->accel_y);
    const uint32_t az = UART_ToU20(src->accel_z);
    const uint32_t gx = UART_ToU20(src->gyro_x);
    const uint32_t gy = UART_ToU20(src->gyro_y);
    const uint32_t gz = UART_ToU20(src->gyro_z);

    dst[0] = (uint8_t)(ax >> 12U);
    dst[1] = (uint8_t)(ax >> 4U);
    dst[2] = (uint8_t)(ay >> 12U);
    dst[3] = (uint8_t)(ay >> 4U);
    dst[4] = (uint8_t)(az >> 12U);
    dst[5] = (uint8_t)(az >> 4U);

    dst[6]  = (uint8_t)(gx >> 12U);
    dst[7]  = (uint8_t)(gx >> 4U);
    dst[8]  = (uint8_t)(gy >> 12U);
    dst[9]  = (uint8_t)(gy >> 4U);
    dst[10] = (uint8_t)(gz >> 12U);
    dst[11] = (uint8_t)(gz >> 4U);

    /* temp_raw: Big Endian. */
    dst[12] = (uint8_t)((uint16_t)src->temp_raw >> 8U);
    dst[13] = (uint8_t)((uint16_t)src->temp_raw & 0xFFU);

    /* timestamp: Big Endian. */
    dst[14] = (uint8_t)(src->timestamp >> 8U);
    dst[15] = (uint8_t)(src->timestamp & 0xFFU);

    /* Младшие 4 bits каждой из шести HIRES осей. */
    dst[16] = (uint8_t)(((ax & 0x0FU) << 4U) | (gx & 0x0FU));
    dst[17] = (uint8_t)(((ay & 0x0FU) << 4U) | (gy & 0x0FU));
    dst[18] = (uint8_t)(((az & 0x0FU) << 4U) | (gz & 0x0FU));
}

/* ================================================================
 * Обнуление одного 19-byte IMU slot. (без изменений)
 * ================================================================ */
static void UART_ClearIMU19(uint8_t dst[UART_IMU_WIRE_BYTES])
{
    uint8_t i;

    for (i = 0U; i < UART_IMU_WIRE_BYTES; i++)
    {
        dst[i] = 0U;
    }
}

/* ================================================================
 * Формирование полного пакета в предоставленный ping-pong slot.
 *
 * [ИЗМЕНЕНИЕ] Каждый датчик за один цикл опроса накопил до
 * ICM_FIFO_POLL_PACKETS (16) сэмплов в g_sensor_batches[id].samples[].
 * В UART-кадр упаковывается только ПОСЛЕДНИЙ по времени сэмпл
 * (samples[count-1]) — как и раньше, формат телеметрии передаёт один
 * "снимок" на кадр, а не всю пачку из 16 точек. Использование
 * последнего (самого свежего) сэмпла минимизирует задержку телеметрии
 * относительно момента чтения FIFO.
 *
 * Не запускает DMA, поэтому безопасно выполняется до критической
 * секции ping-pong producer/consumer.
 * ================================================================ */
static void UART_BuildPacket(uint8_t pkt[UART_PKT_TOTAL_BYTES])
{
    uint8_t  id;
    uint8_t  last_idx;
    uint32_t imu_off;
    uint16_t crc;

    pkt[UART_OFFSET_HEADER]      = UART_PKT_HEADER_0;
    pkt[UART_OFFSET_HEADER + 1U] = UART_PKT_HEADER_1;

    /* frame_counter: Little Endian. */
    pkt[UART_OFFSET_COUNTER]      = (uint8_t)(s_frame_counter & 0xFFU);
    pkt[UART_OFFSET_COUNTER + 1U] = (uint8_t)(s_frame_counter >> 8U);

    for (id = 0U; id < (uint8_t)UART_SENSOR_COUNT; id++)
    {
        imu_off = UART_OFFSET_SAMPLES +
                  ((uint32_t)id * UART_IMU_WIRE_BYTES);

        if ((g_sensor_batches[id].count > 0U) &&
            ((g_sensor_fault_mask & (1UL << id)) == 0U))
        {
            /* Берём самый свежий сэмпл из батча (последний из до 16). */
            last_idx = (uint8_t)(g_sensor_batches[id].count - 1U);

            UART_PackIMU19(&pkt[imu_off],
                           &g_sensor_batches[id].samples[last_idx]);
        }
        else
        {
            /*
             * Invalid IMU: весь 19-byte slot = 0.
             * MATLAB должен интерпретировать all-zero slot как NaN.
             */
            UART_ClearIMU19(&pkt[imu_off]);
        }
    }

    s_frame_counter = (uint16_t)(s_frame_counter + 1U);

    /* CRC16 по payload [2..345], 344 байта. */
    crc = CRC16_CCITT(&pkt[UART_OFFSET_COUNTER],
                       UART_PAYLOAD_BYTES);

    pkt[UART_OFFSET_CRC]      = (uint8_t)(crc & 0xFFU);
    pkt[UART_OFFSET_CRC + 1U] = (uint8_t)(crc >> 8U);
}

/* ================================================================
 * Запуск DMA передачи буфера с указанным индексом.
 *
 * Вызывать только внутри участка, где DMA1_Stream1_IRQn запрещён
 * либо из самого DMA1_Stream1 IRQ, либо при гарантированно idle DMA.
 * ================================================================ */
static void UART_StartDMAFromBuffer(uint8_t buf_idx)
{
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1) != 0U)
    {
        /* Wait until DMA stream is fully disabled. */
    }

    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);
    LL_DMA_ClearFlag_DME1(DMA1);
    LL_DMA_ClearFlag_FE1(DMA1);

    LL_DMA_SetMemoryAddress(DMA1,
                             LL_DMA_STREAM_1,
                             (uint32_t)&g_uart_tx_ping_pong[buf_idx][0]);

    LL_DMA_SetPeriphAddress(DMA1,
                             LL_DMA_STREAM_1,
                             LL_USART_DMA_GetRegAddr(
                                 USART1,
                                 LL_USART_DMA_REG_DATA_TRANSMIT));

    LL_DMA_SetDataLength(DMA1,
                          LL_DMA_STREAM_1,
                          (uint32_t)UART_PKT_TOTAL_BYTES);

    s_dma_active = 1U;
    g_uart_dma_start_count++;

    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}

/* ================================================================
 * Initialization.
 * ================================================================ */
void UART_Telemetry_Init(void)
{
    uint8_t  i;
    uint16_t j;

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1) != 0U)
    {
        /* Wait. */
    }

    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);
    LL_DMA_ClearFlag_DME1(DMA1);
    LL_DMA_ClearFlag_FE1(DMA1);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_DME(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_FE(DMA1, LL_DMA_STREAM_1);

    LL_USART_EnableDMAReq_TX(USART1);

    s_pp_write_idx = 0U;
    s_pp_ready     = 0U;
    s_dma_active   = 0U;

    s_frame_counter = 0U;

    g_uart_drop_count           = 0U;
    g_uart_build_count          = 0U;
    g_uart_enqueue_count        = 0U;
    g_uart_dma_start_count      = 0U;
    g_uart_dma_tc_count         = 0U;
    g_uart_queue_high_watermark = 0U;
    g_uart_queue_count          = 0U;
    g_uart_dma_te_count         = 0U;
    g_uart_dma_dme_count        = 0U;
    g_uart_dma_fe_count         = 0U;

    for (i = 0U; i < UART_PINGPONG_BUFFERS; i++)
    {
        for (j = 0U; j < UART_PKT_TOTAL_BYTES; j++)
        {
            g_uart_tx_ping_pong[i][j] = 0U;
        }
    }
}

/* ================================================================
 * Producer: сформировать пакет и отправить его через ping-pong.
 *
 * Вызывается после ICM_ParseAllFIFO(), один раз за цикл опроса (100 Гц).
 *
 * Логика:
 *  1. Записать пакет в g_uart_tx_ping_pong[s_pp_write_idx].
 *  2. Установить s_pp_ready = 1.
 *  3. Если DMA простаивает (s_dma_active==0):
 *       - немедленно запустить DMA из s_pp_write_idx;
 *       - переключить s_pp_write_idx на другой буфer;
 *       - сбросить s_pp_ready (кадр уже забран).
 *     Если DMA активен — ничего не запускаем: следующий запуск
 *     произойдёт автоматически в UART_DMA_TxComplete().
 * ================================================================ */
void UART_BuildAndSendSyncFrame(void)
{
    uint32_t irq_state;
    uint8_t  local_write_idx;

    g_uart_build_count++;

    /*
     * Короткая critical section вокруг чтения/изменения общих
     * переменных ping-pong состояния (s_dma_active, s_pp_write_idx,
     * s_pp_ready), которые также меняются в DMA IRQ.
     *
     * Само построение пакета (UART_BuildPacket) выполняется ВНЕ
     * critical section, чтобы не увеличивать IRQ latency.
     */
    irq_state = __get_PRIMASK();
    __disable_irq();
    local_write_idx = s_pp_write_idx;
    if (irq_state == 0U)
    {
        __enable_irq();
    }

    /* Построение пакета в текущем write-буфере (DMA его не читает). */
    UART_BuildPacket(g_uart_tx_ping_pong[local_write_idx]);

    irq_state = __get_PRIMASK();
    __disable_irq();

    s_pp_ready = 1U;
    g_uart_enqueue_count++;
    g_uart_queue_count = s_pp_ready;
    if (g_uart_queue_count > g_uart_queue_high_watermark)
    {
        g_uart_queue_high_watermark = g_uart_queue_count;
    }

    if (s_dma_active == 0U)
    {
        /* DMA простаивает — запускаем передачу немедленно. */
        UART_StartDMAFromBuffer(local_write_idx);

        /* Переключаем write-буфер на другой, кадр уже "забран" DMA. */
        s_pp_write_idx ^= 1U;
        s_pp_ready = 0U;
        g_uart_queue_count = 0U;
    }
    /*
     * Если DMA активен — ничего не делаем: UART_DMA_TxComplete()
     * увидит s_pp_ready==1 и запустит передачу этого же буфера
     * (local_write_idx == s_pp_write_idx, ещё не переключённого)
     * сразу после завершения текущей передачи.
     */

    if (irq_state == 0U)
    {
        __enable_irq();
    }
}

/* ================================================================
 * Consumer: DMA transfer complete.
 *
 * Вызывать только из DMA1_Stream1_IRQHandler() после очистки TC flag.
 *
 * Логика:
 *  - s_dma_active = 0.
 *  - Если producer успел подготовить новый кадр (s_pp_ready==1):
 *      кадр лежит в s_pp_write_idx (producer ещё не переключил индекс,
 *      т.к. DMA был занят) — запускаем передачу из s_pp_write_idx,
 *      затем переключаем s_pp_write_idx на другой буфер и сбрасываем
 *      s_pp_ready.
 *  - Если нового кадра нет (s_pp_ready==0) — DMA не перезапускается,
 *    остаётся idle до следующего вызова UART_BuildAndSendSyncFrame().
 * ================================================================ */
void UART_DMA_TxComplete(void)
{
    g_uart_dma_tc_count++;
    s_dma_active = 0U;

    if (s_pp_ready == 1U)
    {
        uint8_t ready_idx = s_pp_write_idx;

        UART_StartDMAFromBuffer(ready_idx);

        s_pp_write_idx ^= 1U;
        s_pp_ready = 0U;
        g_uart_queue_count = 0U;
    }
}
