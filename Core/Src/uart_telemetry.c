/* =============================================================================
 * uart_telemetry.c — USART1 telemetry via DMA1 Stream1, ring-buffer TX queue.
 *
 * [FIX v4] Исправления по итогам анализа реальных счётчиков в debug:
 *  1. FE (FIFO error) диагностика для DMA1 Stream1 больше не считается
 *     ошибкой — DMA FIFO mode для этого стрима ОТКЛЮЧЁН
 *     (LL_DMA_DisableFifoMode в main.c), поэтому флаг FE в этом режиме
 *     не имеет смысла как индикатор реальной ошибки и ранее давал
 *     g_uart_dma_fe_count ~= g_uart_dma_start_count (ложный alarm на
 *     каждом старте DMA). IT_FE больше не включается.
 *  2. Двухфазная модель очереди: producer теперь резервирует индекс
 *     tail (s_q_tail) ДО построения пакета, но инкрементирует
 *     s_q_count и делает пакет видимым для consumer только ПОСЛЕ
 *     того, как UART_BuildPacket() полностью завершён. Это устраняет
 *     теоретическое окно, в котором consumer/DMA restart мог увидеть
 *     "число элементов" больше, чем реально готовых пакетов.
 *  3. Добавлены defensive asserts (в отладочной сборке) на выход
 *     s_q_count за пределы [0, UART_TX_QUEUE_DEPTH] — раньше на скрине
 *     наблюдалось g_uart_queue_count = 94 при глубине очереди 32,
 *     что физически невозможно при корректной инвариантной логике.
 *  4. UART_DMA_TxComplete() использует "быстрый" restart без
 *     LL_DMA_DisableStream()+busy-wait, так как после TC поток уже
 *     аппаратно остановлен — экономит время ISR на каждом из
 *     ICM_FIFO_POLL_PACKETS кадров подряд.
 * ============================================================================= */

#include "uart_telemetry.h"
#include "icm45686_spi.h"
#include "main.h"

#ifndef NDEBUG
#define UART_Q_ASSERT(cond)  do { if (!(cond)) { __BKPT(0); } } while (0)
#else
#define UART_Q_ASSERT(cond)  do { } while (0)
#endif

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


/* Таблица CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xorout) */
static const uint16_t g_crc16_ccitt_table[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063,
    0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B,
    0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,

    0x1231, 0x0210, 0x3273, 0x2252,
    0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A,
    0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,

    0x2462, 0x3443, 0x0420, 0x1401,
    0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509,
    0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,

    0x3693, 0x26B2, 0x16D1, 0x06F0,
    0x7657, 0x6676, 0x5655, 0x4674,
    0xB71B, 0xA73A, 0x9719, 0x8738,
    0xF7DF, 0xE7FE, 0xD7DD, 0xC7FC,

    0x48C4, 0x58E5, 0x6886, 0x78A7,
    0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF,
    0x8948, 0x9969, 0xA90A, 0xB92B,

    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
    0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E,
    0x9B79, 0x8B58, 0xBB3B, 0xAB1A,

    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5,
    0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
    0xAD2A, 0xBD0B, 0x8D68, 0x9D49,

    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4,
    0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC,
    0xBF1B, 0xAF3A, 0x9F59, 0x8F78,

    0x9188, 0x81A9, 0xB1CA, 0xA1EB,
    0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3,
    0x5004, 0x4025, 0x7046, 0x6067,

    0x83B9, 0x9398, 0xA3FB, 0xB3DA,
    0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2,
    0x4235, 0x5214, 0x6277, 0x7256,

    0xB5EA, 0xA5CB, 0x95A8, 0x8589,
    0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481,
    0x7466, 0x6447, 0x5424, 0x4405,

    0xA7DB, 0xB7FA, 0x8799, 0x97B8,
    0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0,
    0x6657, 0x7676, 0x4615, 0x5634,

    0xD94C, 0xC96D, 0xF90E, 0xE92F,
    0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827,
    0x18C0, 0x08E1, 0x3882, 0x28A3,

    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E,
    0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16,
    0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,

    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D,
    0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45,
    0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,

    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C,
    0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74,
    0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
/* ================================================================
 * CRC16-CCITT: poly 0x1021, init 0xFFFF.
 * ================================================================ */
static uint16_t CRC16_CCITT(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        uint8_t idx = (uint8_t)(((crc >> 8) & 0xFFU) ^ data[i]);
        crc = (uint16_t)((crc << 8) ^ g_crc16_ccitt_table[idx]);
    }

    return crc;
}

static inline uint32_t UART_ToU20(int32_t value)
{
    return ((uint32_t)value) & 0x000FFFFFUL;
}

static inline void UART_PackIMU19(uint8_t dst[UART_IMU_WIRE_BYTES],
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


static void UART_BuildPacket(uint8_t pkt[UART_PKT_TOTAL_BYTES],
                             uint8_t sample_idx)
{
    uint8_t  id;
    uint32_t imu_off;
    uint16_t crc;

    /* Заголовок и счётчик кадра */
    pkt[UART_OFFSET_HEADER]      = UART_PKT_HEADER_0;
    pkt[UART_OFFSET_HEADER + 1U] = UART_PKT_HEADER_1;
    pkt[UART_OFFSET_COUNTER]     = (uint8_t)(s_frame_counter & 0xFFU);
    pkt[UART_OFFSET_COUNTER + 1U]= (uint8_t)(s_frame_counter >> 8U);

    /* Один раз обнуляем весь IMU-пейлоад кадра
     * (36 сенсоров × 19 байт = UART_SENSOR_COUNT * UART_IMU_WIRE_BYTES). */
    memset(&pkt[UART_OFFSET_SAMPLES],
           0x00,
           (size_t)UART_SENSOR_COUNT * (size_t)UART_IMU_WIRE_BYTES);

    /* Упаковываем только валидных сенсоров, остальные уже нули. */
    for (id = 0U; id < (uint8_t)UART_SENSOR_COUNT; id++)
    {
        if ((sample_idx < g_sensor_batches[id].count) &&
            ((g_sensor_fault_mask & (1UL << id)) == 0U))
        {
            imu_off = UART_OFFSET_SAMPLES
                    + ((uint32_t)id * (uint32_t)UART_IMU_WIRE_BYTES);

            UART_PackIMU19(&pkt[imu_off],
                           &g_sensor_batches[id].samples[sample_idx]);
        }
        /* else: уже нули, ничего не делаем */
    }

    s_frame_counter = (uint16_t)(s_frame_counter + 1U);

    /* Табличный CRC-CCITT по payload (как и раньше,
     * начиная с счётчика кадра). */
    crc = CRC16_CCITT(&pkt[UART_OFFSET_COUNTER], UART_PAYLOAD_BYTES);
    pkt[UART_OFFSET_CRC]      = (uint8_t)(crc & 0xFFU);
    pkt[UART_OFFSET_CRC + 1U] = (uint8_t)(crc >> 8U);
}

/* ================================================================
 * Полный запуск DMA — используется только когда поток мог быть
 * в неопределённом состоянии (первый старт из producer, когда
 * s_dma_active был 0, но стрим не гарантированно disabled).
 * ================================================================ */
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

/* ================================================================
 * [FIX v4] Быстрый restart — вызывается ТОЛЬКО из UART_DMA_TxComplete().
 * После TC поток DMA уже аппаратно остановлен HW-автоматом
 * (Normal mode, EN сброшен самим DMA), поэтому DisableStream()+
 * busy-wait избыточны и лишь удлиняют ISR на каждый из
 * ICM_FIFO_POLL_PACKETS кадров подряд.
 * ================================================================ */
static void UART_StartDMAFromBuffer_Fast(uint8_t buf_idx)
{
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);
    LL_DMA_ClearFlag_DME1(DMA1);
    LL_DMA_ClearFlag_FE1(DMA1);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1,
                            (uint32_t)&g_uart_tx_queue[buf_idx][0]);
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

    /* [FIX v4] IT_FE НЕ включается: DMA FIFO mode для этого стрима
     * отключён (LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1) в
     * main.c MX_USART1_UART_Init()), флаг FE в Direct mode не несёт
     * смысла реальной ошибки и ранее давал ложный алярм на каждом
     * старте DMA (g_uart_dma_fe_count ~= g_uart_dma_start_count). */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_DME(DMA1, LL_DMA_STREAM_1);
    /* LL_DMA_EnableIT_FE(DMA1, LL_DMA_STREAM_1);  -- УБРАНО, см. выше */

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
 * Producer — [FIX v4] Двухфазная модель: reserve -> build -> commit.
 *
 * Порядок операций:
 *  1. Критическая секция: проверка полноты по s_q_count, резервация
 *     индекса слота local_tail = s_q_tail, продвижение s_q_tail.
 *     s_q_count ЕЩЁ НЕ увеличивается — слот зарезервирован, но не
 *     виден consumer'у как готовый элемент очереди.
 *  2. Построение пакета в зарезервированном слоте — вне критической
 *     секции, IRQ не блокируются.
 *  3. Критическая секция: коммит — увеличение s_q_count (теперь
 *     пакет считается готовым), обновление watermark, и если DMA
 *     простаивает — немедленный запуск передачи из s_q_head.
 *
 * Это устраняет как race producer/producer (актуально при будущем
 * переносе на RTOS/nested ISR), так и логическое окно, в котором
 * consumer мог бы увидеть "количество элементов" больше, чем реально
 * дописанных пакетов.
 * ================================================================ */
volatile uint32_t g_uart_build_cyc_last = 0U;
volatile uint32_t g_uart_build_cyc_max  = 0U;
volatile uint32_t g_uart_build_us_last  = 0U;
volatile uint32_t g_uart_build_us_max   = 0U;

void UART_BuildAndSendSyncFrame(void)
{
    uint32_t start_cyc = DWT->CYCCNT;
    uint8_t  sample_idx;

    g_uart_build_count++;

    for (sample_idx = 0U;
         sample_idx < (uint8_t)ICM_FIFO_POLL_PACKETS;
         sample_idx++)
    {
        uint8_t local_tail;

        /* Шаг 1: резервируем индекс в очереди */
        __disable_irq();
        if (s_q_count >= (uint8_t)UART_TX_QUEUE_DEPTH)
        {
            __enable_irq();
            g_uart_drop_count++;
            continue;
        }
        local_tail = s_q_tail;
        s_q_tail   = (uint8_t)((s_q_tail + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
        __enable_irq();

        /* Шаг 2: строим кадр вне критической секции —
         * теперь уже с быстрым CRC и одним memset. */
        UART_BuildPacket(g_uart_tx_queue[local_tail], sample_idx);

        /* Шаг 3: коммит слота + запуск DMA при простое */
        __disable_irq();
        s_q_count++;
        UART_Q_ASSERT(s_q_count <= (uint8_t)UART_TX_QUEUE_DEPTH);

        g_uart_enqueue_count++;
        g_uart_queue_count = s_q_count;
        if (g_uart_queue_count > g_uart_queue_high_watermark)
        {
            g_uart_queue_high_watermark = g_uart_queue_count;
        }

        if (s_dma_active == 0U)
        {
            UART_StartDMAFromBuffer(s_q_head);
            s_q_head = (uint8_t)((s_q_head + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
            s_q_count--;
            UART_Q_ASSERT(s_q_count <= (uint8_t)UART_TX_QUEUE_DEPTH);
            g_uart_queue_count = s_q_count;
        }
        __enable_irq();
    }

    /* Профиль времени — уже есть, оставляем как ты сделал. */
    {
        uint32_t delta_cyc = DWT->CYCCNT - start_cyc;
        g_uart_build_cyc_last = delta_cyc;
        if (delta_cyc > g_uart_build_cyc_max) g_uart_build_cyc_max = delta_cyc;

        uint32_t us = delta_cyc / (SystemCoreClock / 1000000UL);
        g_uart_build_us_last = us;
        if (us > g_uart_build_us_max) g_uart_build_us_max = us;
    }
}

/* ================================================================
 * Consumer: DMA TC IRQ.
 * Вызывать только из DMA1_Stream1_IRQHandler() после очистки TC flag.
 * [FIX v4] Использует быстрый restart без DisableStream()+busy-wait,
 * так как поток уже аппаратно остановлен HW после TC (Normal mode).
 * ================================================================ */
void UART_DMA_TxComplete(void)
{
    g_uart_dma_tc_count++;
    s_dma_active = 0U;

    UART_Q_ASSERT(s_q_count <= (uint8_t)UART_TX_QUEUE_DEPTH);

    if (s_q_count > 0U)
    {
        UART_StartDMAFromBuffer_Fast(s_q_head);
        s_q_head = (uint8_t)((s_q_head + 1U) % (uint8_t)UART_TX_QUEUE_DEPTH);
        s_q_count--;
        UART_Q_ASSERT(s_q_count <= (uint8_t)UART_TX_QUEUE_DEPTH);
        g_uart_queue_count = s_q_count;
    }
}
