/* =============================================================================
 * uart_telemetry.c — USART1 telemetry via DMA1 Stream1, ring-buffer TX queue.
 * [FIX v3] Race-condition в producer устранён: слот резервируется атомарно
 *           ДО построения пакета. aligned(32) для cache-line корректности.
 * ============================================================================= */

#include "uart_telemetry.h"
#include "icm45686_spi.h"
#include "main.h"

/* ================================================================
 * TX ring-buffer: Non-Cacheable D2 SRAM.
 * aligned(32) — граница cache line (32 байта на Cortex-M7).
 * ================================================================ */
static uint8_t g_uart_tx_queue[UART_TX_QUEUE_DEPTH][UART_PKT_TOTAL_BYTES]
    __attribute__((section(".RAM_D1_DMA"), aligned(32)));

static volatile uint8_t s_q_head     = 0U;
static volatile uint8_t s_q_tail     = 0U;
static volatile uint8_t s_q_count    = 0U;
static volatile uint8_t s_dma_active = 0U;

static uint16_t s_frame_counter = 0U;

volatile uint32_t g_uart_drop_count           = 0U;
volatile uint32_t g_uart_build_count          = 0U;
volatile uint32_t g_uart_enqueue_count        = 0U;
volatile uint32_t g_uart_dma_start_count      = 0U;
volatile uint32_t g_uart_dma_tc_count         = 0U;
volatile uint8_t  g_uart_queue_high_watermark = 0U;
volatile uint8_t  g_uart_queue_count          = 0U;
volatile uint32_t g_uart_dma_te_count         = 0U;
volatile uint32_t g_uart_dma_dme_count        = 0U;
volatile uint32_t g_uart_dma_fe_count         = 0U;

/* ================================================================
 * CRC16-CCITT: poly 0x1021, init 0xFFFF.
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
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            else
                crc = (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static inline uint32_t UART_ToU20(int32_t value)
{
    return ((uint32_t)value) & 0x000FFFFFUL;
}

static void UART_PackIMU19(uint8_t dst[UART_IMU_WIRE_BYTES],
                            const ICM_Sample_t *src)
{
    const uint32_t ax = UART_ToU20(src->accel_x);
    const uint32_t ay = UART_ToU20(src->accel_y);
    const uint32_t az = UART_ToU20(src->accel_z);
    const uint32_t gx = UART_ToU20(src->gyro_x);
    const uint32_t gy = UART_ToU20(src->gyro_y);
    const uint32_t gz = UART_ToU20(src->gyro_z);

    dst[0]  = (uint8_t)(ax >> 12U);
    dst[1]  = (uint8_t)(ax >> 4U);
    dst[2]  = (uint8_t)(ay >> 12U);
    dst[3]  = (uint8_t)(ay >> 4U);
    dst[4]  = (uint8_t)(az >> 12U);
    dst[5]  = (uint8_t)(az >> 4U);
    dst[6]  = (uint8_t)(gx >> 12U);
    dst[7]  = (uint8_t)(gx >> 4U);
    dst[8]  = (uint8_t)(gy >> 12U);
    dst[9]  = (uint8_t)(gy >> 4U);
    dst[10] = (uint8_t)(gz >> 12U);
    dst[11] = (uint8_t)(gz >> 4U);
    dst[12] = (uint8_t)((uint16_t)src->temp_raw >> 8U);
    dst[13] = (uint8_t)((uint16_t)src->temp_raw & 0xFFU);
    dst[14] = (uint8_t)(src->timestamp >> 8U);
    dst[15] = (uint8_t)(src->timestamp & 0xFFU);
    dst[16] = (uint8_t)(((ax & 0x0FU) << 4U) | (gx & 0x0FU));
    dst[17] = (uint8_t)(((ay & 0x0FU) << 4U) | (gy & 0x0FU));
    dst[18] = (uint8_t)(((az & 0x0FU) << 4U) | (gz & 0x0FU));
}

static void UART_ClearIMU19(uint8_t dst[UART_IMU_WIRE_BYTES])
{
    uint8_t i;
    for (i = 0U; i < UART_IMU_WIRE_BYTES; i++) { dst[i] = 0U; }
}

static void UART_BuildPacket(uint8_t pkt[UART_PKT_TOTAL_BYTES],
                              uint8_t sample_idx)
{
    uint8_t  id;
    uint32_t imu_off;
    uint16_t crc;

    pkt[UART_OFFSET_HEADER]      = UART_PKT_HEADER_0;
    pkt[UART_OFFSET_HEADER + 1U] = UART_PKT_HEADER_1;
    pkt[UART_OFFSET_COUNTER]     = (uint8_t)(s_frame_counter & 0xFFU);
    pkt[UART_OFFSET_COUNTER + 1U]= (uint8_t)(s_frame_counter >> 8U);

    for (id = 0U; id < (uint8_t)UART_SENSOR_COUNT; id++)
    {
        imu_off = UART_OFFSET_SAMPLES + ((uint32_t)id * UART_IMU_WIRE_BYTES);

        if ((sample_idx < g_sensor_batches[id].count) &&
            ((g_sensor_fault_mask & (1UL << id)) == 0U))
        {
            UART_PackIMU19(&pkt[imu_off],
                           &g_sensor_batches[id].samples[sample_idx]);
        }
        else
        {
            UART_ClearIMU19(&pkt[imu_off]);
        }
    }

    s_frame_counter = (uint16_t)(s_frame_counter + 1U);

    crc = CRC16_CCITT(&pkt[UART_OFFSET_COUNTER], UART_PAYLOAD_BYTES);
    pkt[UART_OFFSET_CRC]      = (uint8_t)(crc & 0xFFU);
    pkt[UART_OFFSET_CRC + 1U] = (uint8_t)(crc >> 8U);
}

static void UART_StartDMAFromBuffer(uint8_t buf_idx)
{
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1) != 0U) {}

    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);
    LL_DMA_ClearFlag_DME1(DMA1);
    LL_DMA_ClearFlag_FE1(DMA1);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1,
                            (uint32_t)&g_uart_tx_queue[buf_idx][0]);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1,
                            LL_USART_DMA_GetRegAddr(USART1,
                                LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1,
                         (uint32_t)UART_PKT_TOTAL_BYTES);

    s_dma_active = 1U;
    g_uart_dma_start_count++;
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}

void UART_Telemetry_Init(void)
{
    uint8_t  i;
    uint16_t j;

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1) != 0U) {}

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
        for (j = 0U; j < UART_PKT_TOTAL_BYTES; j++)
            g_uart_tx_queue[i][j] = 0U;
}

/* ================================================================
 * Producer — [FIX v3] слот резервируется атомарно ДО построения пакета.
 *
 * Порядок операций:
 *  1. Критическая секция: проверка полноты, резервирование слота
 *     (advance s_q_tail, increment s_q_count) — занимает ~10 тактов.
 *  2. Построение пакета в зарезервированном слоте — вне критической
 *     секции, IRQ не блокируются.
 *  3. Критическая секция: если DMA простаивает — запустить передачу
 *     из s_q_head, изъять кадр из очереди.
 *
 * Это устраняет race: два одновременных вызова (теоретически при RTOS
 * или вложенных ISR) никогда не получат один и тот же local_tail.
 * ================================================================ */
void UART_BuildAndSendSyncFrame(void)
{
    uint8_t sample_idx;

    g_uart_build_count++;

    for (sample_idx = 0U;
         sample_idx < (uint8_t)ICM_FIFO_POLL_PACKETS;
         sample_idx++)
    {
        uint8_t local_tail;

        /* --- Шаг 1: атомарное резервирование слота --- */
        __disable_irq();
        if (s_q_count >= (uint8_t)UART_TX_QUEUE_DEPTH)
        {
            __enable_irq();
            g_uart_drop_count++;
            continue;
        }
        local_tail = s_q_tail;
        s_q_tail   = (uint8_t)((s_q_tail + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
        s_q_count++;
        g_uart_enqueue_count++;
        g_uart_queue_count = s_q_count;
        if (g_uart_queue_count > g_uart_queue_high_watermark)
        {
            g_uart_queue_high_watermark = g_uart_queue_count;
        }
        __enable_irq();

        /* --- Шаг 2: построение пакета (вне critical section) --- */
        UART_BuildPacket(g_uart_tx_queue[local_tail], sample_idx);

        /* --- Шаг 3: запуск DMA если простаивает --- */
        __disable_irq();
        if (s_dma_active == 0U)
        {
            /* DMA простаивает — запускаем из головы очереди немедленно */
            UART_StartDMAFromBuffer(s_q_head);
            s_q_head = (uint8_t)((s_q_head + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
            s_q_count--;
            g_uart_queue_count = s_q_count;
        }
        __enable_irq();
    }
}

/* ================================================================
 * Consumer: DMA TC IRQ.
 * Вызывать только из DMA1_Stream1_IRQHandler() после очистки TC flag.
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
