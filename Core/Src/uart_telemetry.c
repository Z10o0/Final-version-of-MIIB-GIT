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
#include <string.h>

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
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U
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
