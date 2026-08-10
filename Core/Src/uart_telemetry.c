/**
 * @file    uart_telemetry.c
 * @brief   USART1 telemetry through DMA1 Stream1 with TX ring-buffer.
 *
 * Очередь нужна для компенсации кратковременного джиттера:
 *
 *   main-loop / TIM task:
 *       ICM_ParseAllFIFO()
 *       UART_BuildAndSendSyncFrame()
 *              |
 *              v
 *       ring-buffer [packet 0 ... packet 7]
 *              |
 *              v
 *       DMA1 Stream1 -> USART1 TX
 *
 * Важно:
 *   Ring-buffer не повышает физическую пропускную способность UART.
 *   При 8 Mbaud максимум для 348-byte пакета ~2298 packets/s.
 *   Нормальная producer rate должна быть 1600 packets/s.
 */

#include "uart_telemetry.h"
#include "icm45686_spi.h"
#include "main.h"

/* ================================================================
 * TX queue в Non-Cacheable D2 SRAM.
 *
 * DMA читает packet непосредственно из queue slot.
 * Поэтому cache-clean перед запуском DMA не требуется.
 * ================================================================ */
static uint8_t g_uart_tx_queue[UART_TX_QUEUE_DEPTH][UART_PKT_TOTAL_BYTES]
    __attribute__((section(".RAM_D2"), aligned(4)));

/*
 * Индексы queue:
 *
 * s_q_read  — слот, который DMA сейчас передаёт либо передаст следующим.
 * s_q_write — свободный слот для добавления следующего пакета.
 * s_q_count — количество кадров: 0...UART_TX_QUEUE_DEPTH.
 *
 * Producer: main-loop.
 * Consumer: DMA TC IRQ.
 *
 * Изменения общих queue state защищены временным маскированием
 * только DMA1 Stream1 IRQ, а не всех глобальных прерываний.
 */
static volatile uint8_t s_q_read  = 0U;
static volatile uint8_t s_q_write = 0U;
static volatile uint8_t s_q_count = 0U;

static volatile uint8_t s_dma_active = 0U;

/* frame_counter переполняется modulo 65536 каждые 40.96 s при 1600 Hz. */
static uint16_t s_frame_counter = 0U;

/* ================================================================
 * Public diagnostics
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
 * CRC16-CCITT: polynomial 0x1021, init 0xFFFF.
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
 * Извлечение 20 полезных бит sign-extended int32_t.
 *
 * Этот вариант верен, если build20() возвращает обычное
 * sign-extended 20-bit значение:
 *
 *   -524288 ... +524287.
 *
 * Если в актуальном icm45686_data.c данные left-aligned:
 *
 *   bits [31:12] = 20-bit sample, bits [11:0] = 0,
 *
 * замени return на:
 *
 * return (((uint32_t)value >> 12U) & 0x000FFFFFUL);
 * ================================================================ */
static inline uint32_t UART_ToU20(int32_t value)
{
    return ((uint32_t)value) & 0x000FFFFFUL;
}

/* ================================================================
 * Упаковка одного ICM_Sample_t в независимый от ABI 19-byte формат.
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

    dst[0]  = (uint8_t)(ax >> 12U);
    dst[1]  = (uint8_t)(ax >>  4U);
    dst[2]  = (uint8_t)(ay >> 12U);
    dst[3]  = (uint8_t)(ay >>  4U);
    dst[4]  = (uint8_t)(az >> 12U);
    dst[5]  = (uint8_t)(az >>  4U);

    dst[6]  = (uint8_t)(gx >> 12U);
    dst[7]  = (uint8_t)(gx >>  4U);
    dst[8]  = (uint8_t)(gy >> 12U);
    dst[9]  = (uint8_t)(gy >>  4U);
    dst[10] = (uint8_t)(gz >> 12U);
    dst[11] = (uint8_t)(gz >>  4U);

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
 * Обнуление одного 19-byte IMU slot.
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
 * Формирование полного пакета в предоставленный queue slot.
 *
 * Не запускает DMA, поэтому безопасно выполняется до критической
 * секции queue producer/consumer.
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

    /*
     * Инкремент только для сформированного кадра.
     *
     * Кадр, отброшенный при полном buffer, не получает отдельного
     * значения counter: последовательность на UART покажет этот gap.
     */
    s_frame_counter = (uint16_t)(s_frame_counter + 1U);

    /* CRC16 по payload [2..345], 344 байта. */
    crc = CRC16_CCITT(&pkt[UART_OFFSET_COUNTER],
                      UART_PAYLOAD_BYTES);

    pkt[UART_OFFSET_CRC]      = (uint8_t)(crc & 0xFFU);
    pkt[UART_OFFSET_CRC + 1U] = (uint8_t)(crc >> 8U);
}

/* ================================================================
 * Запуск DMA для текущего read slot.
 *
 * Вызывать только внутри участка, где DMA1_Stream1_IRQn запрещён
 * либо из самого DMA1_Stream1 IRQ.
 * ================================================================ */
static void UART_StartDMAFromReadSlot(void)
{
    uint8_t read_slot;

    if ((s_dma_active != 0U) || (s_q_count == 0U))
    {
        return;
    }

    read_slot = s_q_read;

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
                            (uint32_t)&g_uart_tx_queue[read_slot][0]);

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
    uint8_t i;

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

    s_q_read     = 0U;
    s_q_write    = 0U;
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
    g_uart_dma_te_count  = 0U;
    g_uart_dma_dme_count = 0U;
    g_uart_dma_fe_count  = 0U;


    for (i = 0U; i < UART_TX_QUEUE_DEPTH; i++)
    {
        uint16_t j;

        for (j = 0U; j < UART_PKT_TOTAL_BYTES; j++)
        {
            g_uart_tx_queue[i][j] = 0U;
        }
    }
}

/* ================================================================
 * Producer: сформировать и поставить пакет в ring-buffer.
 *
 * Вызывается после ICM_ParseAllFIFO().
 * ================================================================ */
void UART_BuildAndSendSyncFrame(void)
{
    uint8_t write_slot;
    uint32_t irq_state;

    g_uart_build_count++;

    /*
     * Короткая critical section нужна, потому что s_q_count,
     * s_q_read и s_dma_active могут одновременно меняться в DMA IRQ.
     *
     * Сохраняем PRIMASK и запрещаем IRQ буквально на несколько
     * инструкций. Packet build выполняется ВНЕ critical section.
     */
    irq_state = __get_PRIMASK();
    __disable_irq();

    if (s_q_count >= UART_TX_QUEUE_DEPTH)
    {
        g_uart_drop_count++;

        if (irq_state == 0U)
        {
            __enable_irq();
        }

        return;
    }

    write_slot = s_q_write;

    if (irq_state == 0U)
    {
        __enable_irq();
    }

    /*
     * Формирование пакета происходит в ещё не опубликованном
     * write_slot. DMA его читать не может.
     */
    UART_BuildPacket(g_uart_tx_queue[write_slot]);

    irq_state = __get_PRIMASK();
    __disable_irq();

    /*
     * Публикуем готовый пакет только после полной записи 348 bytes.
     * Пока s_q_write не сдвинут, consumer этот слот не видит.
     */
    s_q_write = (uint8_t)((s_q_write + 1U) & UART_TX_QUEUE_MASK);
    s_q_count++;

    g_uart_enqueue_count++;
    g_uart_queue_count = s_q_count;

    if (s_q_count > g_uart_queue_high_watermark)
    {
        g_uart_queue_high_watermark = s_q_count;
    }

    /*
     * Если DMA был idle, немедленно начать передачу текущего read slot.
     */
    UART_StartDMAFromReadSlot();

    if (irq_state == 0U)
    {
        __enable_irq();
    }
}

/* ================================================================
 * Consumer: DMA transfer complete.
 *
 * Вызывать только из DMA1_Stream1_IRQHandler() после очистки TC flag.
 * ================================================================ */
void UART_DMA_TxComplete(void)
{
    g_uart_dma_tc_count++;

    /*
     * Текущий read slot полностью передан DMA.
     * Освобождаем его и запускаем следующий без ожидания main-loop.
     */
    if (s_q_count > 0U)
    {
        s_q_read = (uint8_t)((s_q_read + 1U) & UART_TX_QUEUE_MASK);
        s_q_count--;
    }

    g_uart_queue_count = s_q_count;
    s_dma_active = 0U;

    /*
     * Если producer успел положить ещё кадры, передаём следующий
     * непосредственно из DMA IRQ.
     */
    UART_StartDMAFromReadSlot();
}
