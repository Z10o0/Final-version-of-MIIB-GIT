/**
 * @file uart_telemetry.c
 * @brief USART1 telemetry through DMA1 Stream1 with TX ring-buffer.
 *
 * ------------------------------------------------------------------
 * [ИЗМЕНЕНИЯ v2] Ping-pong (2 буфера) заменён на ring-buffer
 * (UART_TX_QUEUE_DEPTH слотов).
 *
 * Раньше: за один цикл опроса (100 Гц) FIFO каждого датчика отдавал
 * 16 сэмплов (1600 Гц ODR / 100 Гц poll rate), но в UART уходил
 * только последний (samples[count-1]) -> 1 кадр/цикл -> 100 кадров/с
 * (то есть "каждый 16-й пакет").
 *
 * Теперь: UART_BuildAndSendSyncFrame() формирует и ставит в очередь
 * ВСЕ до 16 сэмплов батча как отдельные 348-байтные кадры ->
 * 100 Гц * 16 = 1600 кадров/с ("каждый пакет").
 *
 * main-loop @ 100 Hz:
 *   ICM_ParseAllFIFO()
 *   UART_BuildAndSendSyncFrame()      <-- кладёт до 16 кадров в очередь
 *         |
 *         v
 *   ring-buffer[16 x 348 bytes]  <-- producer пишет сюда все 16 кадров
 *         |
 *         v (DMA TC IRQ вытягивает кадры один за другим)
 *   DMA1 Stream1 -> USART1 TX
 *
 * Расчёт по времени при 8.1 Mbaud:
 *   1 кадр (348 байт): 348 * 10 bits/byte / 8.1e6 bit/s ~= 344 мкс.
 *   16 кадров подряд:  16 * 344 мкс ~= 5.50 мс.
 *   Период опроса (TIM6 @ 100 Гц): 10 мс.
 *   Запас: 10 мс - 5.50 мс = 4.5 мс -- DMA гарантированно успевает
 *   "выгребать" очередь между циклами опроса, потерь кадров не будет
 *   при штатной работе (см. также g_uart_drop_count).
 * ------------------------------------------------------------------
 */

#include "uart_telemetry.h"
#include "icm45686_spi.h"
#include "main.h"

/* ================================================================
 * TX ring-buffer в Non-Cacheable D2 SRAM.
 *
 * DMA читает буфер непосредственно из слота очереди.
 * Поэтому cache-clean перед запуском DMA не требуется.
 *
 * UART_TX_QUEUE_DEPTH (32) x 348 bytes = 11136 bytes D2 SRAM
 * (было 696 bytes при ping-pong depth=2).
 * ================================================================ */
static uint8_t g_uart_tx_queue[UART_TX_QUEUE_DEPTH][UART_PKT_TOTAL_BYTES]
    __attribute__((section(".RAM_D2"), aligned(4)));

/*
 * Состояние ring-buffer очереди (классическая circular queue):
 *
 * s_q_head       — индекс слота, который сейчас передаёт DMA (или
 *                  будет передавать следующим), читает Consumer.
 * s_q_tail       — индекс слота, в который producer запишет
 *                  следующий кадр.
 * s_q_count      — число кадров, ожидающих отправки (0..UART_TX_QUEUE_DEPTH).
 * s_dma_active   — DMA сейчас передаёт слот s_q_head.
 *
 * Producer:  main-loop (UART_BuildAndSendSyncFrame), кладёт кадры
 *            в s_q_tail и увеличивает s_q_count.
 * Consumer:  DMA TC IRQ (UART_DMA_TxComplete), после завершения
 *            передачи текущего слота продвигает s_q_head и
 *            уменьшает s_q_count; если остались кадры — запускает
 *            следующую передачу.
 */
static volatile uint8_t s_q_head      = 0U;
static volatile uint8_t s_q_tail      = 0U;
static volatile uint8_t s_q_count     = 0U;
static volatile uint8_t s_dma_active  = 0U;

/* frame_counter переполняется modulo 65536. При 1600 Hz это ~41 s. */
static uint16_t s_frame_counter = 0U;

/* ================================================================
 * Public diagnostics
 *
 * [ИЗМЕНЕНИЕ v2] g_uart_drop_count теперь реально считает потерянные
 * кадры при переполнении очереди (не должно происходить при штатной
 * работе, см. расчёт времени выше). g_uart_queue_count/high_watermark
 * отражают реальную глубину ring-buffer очереди.
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
 * [ИЗМЕНЕНИЕ v2] Формирование ОДНОГО полного пакета для ЗАДАННОГО
 * индекса sample_idx (0..ICM_FIFO_POLL_PACKETS-1) в предоставленный
 * слот очереди.
 *
 * Раньше функция всегда брала последний сэмпл (samples[count-1]).
 * Теперь индекс сэмпла передаётся явно вызывающей стороной, что
 * позволяет вызывать UART_BuildPacket() в цикле по всем сэмплам
 * батча и получить 16 независимых кадров вместо одного.
 *
 * Если для конкретного датчика sample_idx >= g_sensor_batches[id].count
 * (то есть датчик за этот цикл отдал МЕНЬШЕ пакетов, чем другие -- FIFO
 * может слегка "разойтись" по числу валидных HIRES-пакетов между
 * датчиками), IMU slot заполняется нулями (invalid marker), как и
 * раньше при отсутствии данных.
 *
 * Не запускает DMA, поэтому безопасно выполняется до критической
 * секции очереди producer/consumer.
 * ================================================================ */
static void UART_BuildPacket(uint8_t pkt[UART_PKT_TOTAL_BYTES],
                              uint8_t sample_idx)
{
    uint8_t  id;
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

        if ((sample_idx < g_sensor_batches[id].count) &&
            ((g_sensor_fault_mask & (1UL << id)) == 0U))
        {
            /* Берём именно sample_idx-й сэмпл из батча (0..count-1). */
            UART_PackIMU19(&pkt[imu_off],
                           &g_sensor_batches[id].samples[sample_idx]);
        }
        else
        {
            /*
             * Invalid IMU (либо fault, либо этот датчик отдал меньше
             * пакетов, чем sample_idx): весь 19-byte slot = 0.
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
 * Запуск DMA передачи буфера с указанным индексом слота очереди.
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
                             (uint32_t)&g_uart_tx_queue[buf_idx][0]);

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

    s_q_head     = 0U;
    s_q_tail     = 0U;
    s_q_count    = 0U;
    s_dma_active = 0U;

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

    for (i = 0U; i < UART_TX_QUEUE_DEPTH; i++)
    {
        for (j = 0U; j < UART_PKT_TOTAL_BYTES; j++)
        {
            g_uart_tx_queue[i][j] = 0U;
        }
    }
}

/* ================================================================
 * Producer: сформировать ВСЕ кадры батча и поставить их в очередь.
 *
 * Вызывается после ICM_ParseAllFIFO(), один раз за цикл опроса (100 Гц).
 *
 * [ИЗМЕНЕНИЕ v2] Раньше строился и отправлялся ровно 1 кадр (последний
 * сэмпл батча). Теперь в цикле по ICM_FIFO_POLL_PACKETS (16) строится
 * и ставится в очередь кадр для КАЖДОГО индекса сэмпла: 0, 1, ..., 15.
 * Итог: 100 Гц * 16 = 1600 кадров/с на выходе UART.
 *
 * Логика на каждой итерации:
 *  1. Построить пакет для sample_idx в g_uart_tx_queue[s_q_tail]
 *     (вне critical section -- построение пакета не должно
 *     увеличивать IRQ latency).
 *  2. Короткая critical section: если очередь не полна -- продвинуть
 *     s_q_tail, увеличить s_q_count; если DMA простаивает -- сразу
 *     запустить передачу из головы очереди.
 *  3. Если очередь полна -- инкремент g_uart_drop_count, кадр
 *     отбрасывается (при штатной работе, см. расчёт времени в
 *     заголовке файла, это не должно происходить).
 * ================================================================ */
void UART_BuildAndSendSyncFrame(void)
{
    uint32_t irq_state;
    uint8_t  sample_idx;

    g_uart_build_count++;

    for (sample_idx = 0U; sample_idx < (uint8_t)ICM_FIFO_POLL_PACKETS;
         sample_idx++)
    {
        uint8_t local_tail;
        uint8_t queue_full;

        /*
         * Короткая critical section вокруг чтения общих переменных
         * очереди (s_q_count, s_q_tail), которые также меняются в
         * DMA IRQ.
         */
        irq_state = __get_PRIMASK();
        __disable_irq();
        queue_full = (s_q_count >= (uint8_t)UART_TX_QUEUE_DEPTH) ? 1U : 0U;
        local_tail = s_q_tail;
        if (irq_state == 0U)
        {
            __enable_irq();
        }

        if (queue_full != 0U)
        {
            /* Очередь переполнена -- кадр отбрасывается. */
            g_uart_drop_count++;
            continue;
        }

        /* Построение пакета в текущем tail-слоте (DMA его не читает,
           так как местоположение ещё не опубликовано producer'ом). */
        UART_BuildPacket(g_uart_tx_queue[local_tail], sample_idx);

        irq_state = __get_PRIMASK();
        __disable_irq();

        s_q_tail = (uint8_t)((s_q_tail + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
        s_q_count++;
        g_uart_enqueue_count++;
        g_uart_queue_count = s_q_count;
        if (g_uart_queue_count > g_uart_queue_high_watermark)
        {
            g_uart_queue_high_watermark = g_uart_queue_count;
        }

        if (s_dma_active == 0U)
        {
            /* DMA простаивает -- запускаем передачу немедленно
               из головы очереди (s_q_head), затем "изымаем" этот
               кадр из очереди (он уже забран DMA). */
            UART_StartDMAFromBuffer(s_q_head);

            s_q_head = (uint8_t)((s_q_head + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
            s_q_count--;
            g_uart_queue_count = s_q_count;
        }
        /*
         * Если DMA активен -- ничего не делаем: UART_DMA_TxComplete()
         * увидит s_q_count > 0 и запустит передачу следующего кадра
         * из головы очереди сразу после завершения текущей передачи.
         */

        if (irq_state == 0U)
        {
            __enable_irq();
        }
    }
}

/* ================================================================
 * Consumer: DMA transfer complete.
 *
 * Вызывать только из DMA1_Stream1_IRQHandler() после очистки TC flag.
 *
 * [ИЗМЕНЕНИЕ v2] Раньше проверялся флаг s_pp_ready (0 или 1). Теперь
 * проверяется s_q_count (0..UART_TX_QUEUE_DEPTH) -- если в очереди
 * остались непереданные кадры, немедленно запускается передача
 * следующего кадра из головы очереди (ring-buffer semantics).
 * ================================================================ */
void UART_DMA_TxComplete(void)
{
    g_uart_dma_tc_count++;
    s_dma_active = 0U;

    if (s_q_count > 0U)
    {
        UART_StartDMAFromBuffer(s_q_head);

        s_q_head = (uint8_t)((s_q_head + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
        s_q_count--;
        g_uart_queue_count = s_q_count;
    }
}
