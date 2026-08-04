/* =============================================================================
 * icm45686_spi.c  — ПАРАЛЛЕЛЬНАЯ версия, все 3 шины SPI работают одновременно
 *
 * Адаптировано по образцу BMI323_spi.c (проект БПЛА).
 *
 * КЛЮЧЕВЫЕ ИЗМЕНЕНИЯ:
 *   1. ICM_StartBurstRead() — запускает SPI1, SPI5, SPI4 одновременно
 *   2. ICM_Bus_t.eot_handled — флаг защиты от двойного входа в ICM_OnSpiEot
 *   3. ICM_FinishBus() — теперь только выставляет transfer_complete + вызывает
 *      ICM_TryCompleteBatch() (без цепочки SPI1→SPI5→SPI4)
 *   4. ICM_TryCompleteBatch() — проверяет готовность всех трёх шин
 *   5. TX-буферы и RX-буферы перенесены в .RAM_D2 (Not Cacheable по MPU)
 *      → отпадает необходимость в SCB_Clean/Invalidate
 *
 * Шины и DMA (топология МIIБ без изменений):
 *   SPI1  RX→DMA1/Stream2   TX→DMA1/Stream3   датчики 0..5
 *   SPI5  RX→DMA2/Stream2   TX→DMA2/Stream3   датчики 6..11
 *   SPI4  RX→DMA2/Stream0   TX→DMA2/Stream1   датчики 12..17
 * =============================================================================
 */

#include "icm45686_spi.h"
#include <string.h>

/* ===========================================================================
 *  Прототипы локальных функций
 * ========================================================================== */

static void    ICM_DelayUs(uint32_t us);
static void    ICM_DelayMs(uint32_t ms);

static void    ICM_CS_Low (const ICM_Sensor_t *s);
static void    ICM_CS_High(const ICM_Sensor_t *s);

static void    ICM_SPI_EnsureDisabled(SPI_TypeDef *spi);
static void    ICM_SPI_WaitEOT       (SPI_TypeDef *spi);
static void    ICM_SPI_DrainRx       (SPI_TypeDef *spi, uint32_t n);

static uint8_t ICM_BusIndex       (const ICM_Bus_t *bus);
static uint8_t ICM_FindNextHealthy(const ICM_Bus_t *bus, uint8_t from);

static void    ICM_ClearDmaFlags(const ICM_Bus_t *bus);

static void    ICM_StartBusRead   (ICM_Bus_t *bus, uint8_t idx);
static void    ICM_NextSensor     (ICM_Bus_t *bus);
static void    ICM_OnSpiEot       (ICM_Bus_t *bus);
static void    ICM_FinishBus      (ICM_Bus_t *bus);
static void    ICM_TryCompleteBatch(void);
static void    ICM_MarkFault      (ICM_Sensor_t *s);

/* ===========================================================================
 *  DMA-буферы в D2 SRAM (Not Cacheable по MPU Region)
 *
 *  TX и RX буферы размещены в .RAM_D2 (0x30000000, SRAM D2).
 *  MPU настраивается Not Cacheable → DMA и CPU видят одни данные.
 *  SCB_Clean/Invalidate не требуется.
 * ========================================================================== */

/* Приёмные буферы: [шина][датчик][байты]. */
uint8_t g_fifo_data[ICM_SPI_BUS_COUNT]
                   [ICM_SENSORS_PER_BUS]
                   [ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

/* TX-буферы: по одному на шину. */
static uint8_t g_tx_spi1[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi5[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi4[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

/* Флаги состояния (volatile — используются в main и ISR). */
volatile uint8_t  g_fifo_batch_ready  = 0U;
volatile uint8_t  g_dma_cycle_active  = 0U;
volatile uint32_t g_sensor_fault_mask = 0U;
volatile uint32_t g_dma_error_mask    = 0U;
volatile uint32_t g_tim6_skip_count   = 0U;
volatile uint32_t g_clk_ok_mask   = 0U;
volatile uint32_t g_clk_fail_mask = 0U;

/* ============================================================
 * Таблица соединений (Распиновка подключения плат):
 *
 * SPI1 (DMA1, Stream2/3):
 *   SCK=PA5, MISO=PA6, MOSI=PA7
 *   Датчик 1  → CS: PB12
 *   Датчик 2  → CS: PB13
 *   Датчик 3  → CS: PE8
 *   Датчик 4  → CS: PE9
 *   Датчик 5  → CS: PF13
 *   Датчик 6  → CS: PF14
 *
 * SPI5 (DMA2, Stream2/3):
 *   SCK=PF7, MISO=PF8, MOSI=PF9
 *   Датчик 7  → CS: PE14
 *   Датчик 8  → CS: PE15
 *   Датчик 9  → CS: PE7
 *   Датчик 10 → CS: PG1
 *   Датчик 11 → CS: PB0
 *   Датчик 12 → CS: PB1
 *
 * SPI4 (DMA2, Stream0/1):
 *   SCK=PE2, MISO=PE5, MOSI=PE6
 *   Датчик 13 → CS: PE10
 *   Датчик 14 → CS: PE11
 *   Датчик 15 → CS: PF15
 *   Датчик 16 → CS: PG0
 *   Датчик 17 → CS: PC4
 *   Датчик 18 → CS: PC5
 * ============================================================ */

ICM_Bus_t g_bus_spi1 =
{
    .spi           = SPI1,
    .dma           = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .tx_buf        = g_tx_spi1,
    .eot_handled   = 0U,
    .sensors =
    {
        { SPI1, GPIOB, LL_GPIO_PIN_12, 0U,  0U },
        { SPI1, GPIOB, LL_GPIO_PIN_13, 1U,  0U },
        { SPI1, GPIOE, LL_GPIO_PIN_8,  2U,  0U },
        { SPI1, GPIOE, LL_GPIO_PIN_9,  3U,  0U },
        { SPI1, GPIOF, LL_GPIO_PIN_13, 4U,  0U },
        { SPI1, GPIOF, LL_GPIO_PIN_14, 5U,  0U }
    }
};

ICM_Bus_t g_bus_spi5 =
{
    .spi           = SPI5,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .tx_buf        = g_tx_spi5,
    .eot_handled   = 0U,
    .sensors =
    {
        { SPI5, GPIOE, LL_GPIO_PIN_14, 6U,  0U },
        { SPI5, GPIOE, LL_GPIO_PIN_15, 7U,  0U },
        { SPI5, GPIOE, LL_GPIO_PIN_7,  8U,  0U },
        { SPI5, GPIOG, LL_GPIO_PIN_1,  9U,  0U },
        { SPI5, GPIOB, LL_GPIO_PIN_0,  10U, 0U },
        { SPI5, GPIOB, LL_GPIO_PIN_1,  11U, 0U }
    }
};

ICM_Bus_t g_bus_spi4 =
{
    .spi           = SPI4,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,
    .tx_buf        = g_tx_spi4,
    .eot_handled   = 0U,
    .sensors =
    {
        { SPI4, GPIOE, LL_GPIO_PIN_10, 12U, 0U },
        { SPI4, GPIOE, LL_GPIO_PIN_11, 13U, 0U },
        { SPI4, GPIOF, LL_GPIO_PIN_15, 14U, 0U },
        { SPI4, GPIOG, LL_GPIO_PIN_0,  15U, 0U },
        { SPI4, GPIOC, LL_GPIO_PIN_4,  16U, 0U },
        { SPI4, GPIOC, LL_GPIO_PIN_5,  17U, 0U }
    }
};

/* ===========================================================================
 *  ICM_BusesInit
 * ========================================================================== */
void ICM_BusesInit(void)
{
    uint8_t i;

    g_bus_spi1.tx_buf = g_tx_spi1;
    g_bus_spi5.tx_buf = g_tx_spi5;
    g_bus_spi4.tx_buf = g_tx_spi4;

    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));
    memset(g_tx_spi1,   0xFFU, sizeof(g_tx_spi1));
    memset(g_tx_spi5,   0xFFU, sizeof(g_tx_spi5));
    memset(g_tx_spi4,   0xFFU, sizeof(g_tx_spi4));

    g_tx_spi1[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi5[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi4[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        ICM_CS_High(&g_bus_spi1.sensors[i]);
        ICM_CS_High(&g_bus_spi5.sensors[i]);
        ICM_CS_High(&g_bus_spi4.sensors[i]);

        g_bus_spi1.sensors[i].fault = 0U;
        g_bus_spi5.sensors[i].fault = 0U;
        g_bus_spi4.sensors[i].fault = 0U;
    }

    g_bus_spi1.current_sensor_idx = 0U;
    g_bus_spi5.current_sensor_idx = 0U;
    g_bus_spi4.current_sensor_idx = 0U;

    g_bus_spi1.transfer_complete  = 0U;
    g_bus_spi5.transfer_complete  = 0U;
    g_bus_spi4.transfer_complete  = 0U;

    /* Сброс eot_handled */
    g_bus_spi1.eot_handled = 0U;
    g_bus_spi5.eot_handled = 0U;
    g_bus_spi4.eot_handled = 0U;

    g_fifo_batch_ready  = 0U;
    g_dma_cycle_active  = 0U;
    g_sensor_fault_mask = 0U;
    g_dma_error_mask    = 0U;
    g_tim6_skip_count   = 0U;
}

/* ===========================================================================
 *  ICM_WriteReg — блокирующая запись одного регистра
 * ========================================================================== */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_SPI_EnsureDisabled(spi);
    LL_SPI_SetTransferSize(spi, 2U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, reg & 0x7FU);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 2U);

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}
}

/* ===========================================================================
 *  ICM_ReadReg — блокирующее чтение одного регистра
 * ========================================================================== */
uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg)
{
    SPI_TypeDef *spi = sensor->spi;
    uint8_t      dummy;
    uint8_t      result;

    ICM_SPI_EnsureDisabled(spi);
    LL_SPI_SetTransferSize(spi, 2U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, (reg & 0x7FU) | ICM45686_SPI_READ_BIT);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, 0xFFU);

    ICM_SPI_WaitEOT(spi);

    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
    dummy = LL_SPI_ReceiveData8(spi);
    (void)dummy;

    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
    result = LL_SPI_ReceiveData8(spi);

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}

    return result;
}

/* ===========================================================================
 *  ICM_WriteIReg — блокирующая запись Internal Register (IREG)
 * ========================================================================== */
void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t       addr_h,
                   uint8_t       addr_l,
                   uint8_t       value)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);
    ICM_DelayUs(ICM45686_IREG_DELAY_US);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_DATA, value);
    ICM_DelayUs(ICM45686_IREG_DELAY_US);
}

/* ===========================================================================
 *  ICM_ReadIReg — блокирующее чтение Internal Register (IREG)
 * ========================================================================== */
uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t       addr_h,
                     uint8_t       addr_l)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);
    ICM_DelayUs(ICM45686_IREG_DELAY_US);
    return ICM_ReadReg(sensor, ICM45686_REG_IREG_DATA);
}

/* ===========================================================================
 *  ICM_InitAllSensors  — с проверкой захвата внешнего тактирования
 * ========================================================================== */

uint32_t ICM_InitAllSensors(void)
{
    ICM_Bus_t * const buses[ICM_SPI_BUS_COUNT] =
    {
        &g_bus_spi1,
        &g_bus_spi5,
        &g_bus_spi4
    };

    uint8_t  bus_idx;
    uint8_t  sensor_idx;
    uint8_t  status;
    uint32_t timeout;

    g_sensor_fault_mask = 0U;
    g_clk_ok_mask       = 0U;
    g_clk_fail_mask     = 0U;

    /* ------------------------------------------------------------------
     * Шаг 0. Power-on delay 3 мс.
     * ------------------------------------------------------------------ */
    ICM_DelayMs(3U);

    for (bus_idx = 0U; bus_idx < ICM_SPI_BUS_COUNT; bus_idx++)
    {
        for (sensor_idx = 0U; sensor_idx < ICM_SENSORS_PER_BUS; sensor_idx++)
        {
            ICM_Sensor_t *sensor = &buses[bus_idx]->sensors[sensor_idx];

            /* ==============================================================
             * Шаг 1. WHO_AM_I — проверка SPI-связи ДО reset.
             * ============================================================== */
            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I) != ICM45686_WHO_AM_I_VALUE)
            {
                ICM_MarkFault(sensor);
                continue;
            }

            /* ==============================================================
             * Шаг 2. Soft reset.
             * ============================================================== */
            ICM_WriteReg(sensor, ICM45686_REG_REG_MISC2, ICM45686_SOFT_RESET);
            ICM_DelayUs(ICM45686_RESET_DELAY_US);

            /* ==============================================================
             * Шаг 3. WHO_AM_I после reset — убеждаемся что датчик жив.
             * ============================================================== */
            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I) != ICM45686_WHO_AM_I_VALUE)
            {
                ICM_MarkFault(sensor);
                continue;
            }

            /* ==============================================================
             * Шаг 4. IREG: IOC_PAD_SCENARIO_OVRD → активация CLKIN на пине 9.
             *
             * ICM45686_CLKIN_ENABLE_VAL = 0x06:
             *   bit[2:1] = PAD_SCENARIO_OVRD = 0x03 → CLKIN mode
             *   bit[0]   = PAD_SCENARIO_OVRD_EN = 1  → override активен
             * ============================================================== */
            ICM_WriteIReg(sensor,
                          ICM45686_IREG_TOP1_ADDR_H,
                          ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L,
                          ICM45686_CLKIN_ENABLE_VAL);

            /* ==============================================================
             * Шаг 5. REG_MISC1: переключение на внешний клок (OSC_ID = EXT).
             *
             * ICM45686_OSC_ID_OVRD_EXT_CLK = 0x08 → OSC_ID = EXT CLK (CLKIN).
             * Выполняется ПОСЛЕ IREG — пин 9 уже настроен как CLKIN.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_REG_MISC1,
                         ICM45686_OSC_ID_OVRD_EXT_CLK);

            /* ==============================================================
             * Шаг 5б. Разрешить прерывание PLL_RDY в INT1_CONFIG1.
             * Нужно для того, чтобы INT1_STATUS1[0] корректно обновлялся.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_INT1_CONFIG1,
                         ICM45686_INT1_PLL_RDY_EN);

            /* ==============================================================
             * Шаг 6. Polling PLL_RDY в INT1_STATUS1[0].
             *
             * Типовое время захвата PLL = 3–5 мс при 32.768 кГц CLKIN.
             * Шаг опроса: 1 мс. Таймаут: ICM45686_PLL_TIMEOUT_US / 1000 = 10 мс.
             * INT1_STATUS1 — регистр read-clear, читать 1 раз за итерацию.
             * ============================================================== */
            timeout = ICM45686_PLL_TIMEOUT_US / 1000U;  /* 10 итераций по 1 мс */
            status  = 0U;

            do
            {
                ICM_DelayMs(1U);
                status = ICM_ReadReg(sensor, ICM45686_REG_INT1_STATUS1);

                if ((status & ICM45686_INT1_STATUS_PLL_RDY) != 0U)
                {
                    break;
                }

                if (timeout > 0U)
                {
                    timeout--;
                }
            }
            while (timeout != 0U);

            if ((status & ICM45686_INT1_STATUS_PLL_RDY) != 0U)
            {
                /* PLL захватил внешний клок — датчик работает от CLKIN */
                g_clk_ok_mask |= (1UL << sensor->sensor_id);
            }
            else
            {
                /* PLL не захватил внешний клок за 10 мс.
                 * НЕ помечаем как fault — датчик может работать на внутреннем RC,
                 * но фиксируем в g_clk_fail_mask для диагностики. */
                g_clk_fail_mask |= (1UL << sensor->sensor_id);
            }

            /* ==============================================================
             * Шаг 7. IREG: SMC_CONTROL_0 — настройка timestamp core.
             * Обязательно ДО PWR_MGMT0.
             * ============================================================== */
            ICM_WriteIReg(sensor,
                          ICM45686_IREG_TOP1_ADDR_H,
                          ICM45686_IREG_SMC_CONTROL_0_L,
                          ICM45686_SMC_CONTROL_0_VALUE);

            /* ==============================================================
             * Шаг 8. RTC_CONFIG: RTC_MODE_EN (bit5).
             * ODR генерируется от внешней 32.768 кГц clock domain.
             * Включаем только если PLL захватил клок.
             * ============================================================== */
            if ((g_clk_ok_mask & (1UL << sensor->sensor_id)) != 0U)
            {
                ICM_WriteReg(sensor,
                             ICM45686_REG_RTC_CONFIG,
                             ICM45686_RTC_MODE_EN);
            }

            /* ==============================================================
             * Шаг 9. FSR и ODR.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_ACCEL_CONFIG0,
                         ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE);

            ICM_WriteReg(sensor,
                         ICM45686_REG_GYRO_CONFIG0,
                         ICM_GYRO_FS_VALUE | ICM_GYRO_ODR_VALUE);

            /* ==============================================================
             * Шаг 10. Конфигурация FIFO.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG0,
                         ICM45686_FIFO_MODE_STREAM | ICM45686_FIFO_DEPTH_2K);

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG10,
                         (uint8_t)(ICM_FIFO_WATERMARK_BYTES & 0x00FFU));

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG11,
                         (uint8_t)((ICM_FIFO_WATERMARK_BYTES >> 8U) & 0x00FFU));

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG4,
                         ICM45686_FIFO_TMST_FSYNC_EN);

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN);

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN |
                         ICM45686_FIFO_IF_EN);

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG2,
                         ICM45686_FIFO_FLUSH);

            /* ==============================================================
             * Шаг 11. PWR_MGMT0: Gyro LN + Accel LN.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_PWR_MGMT0,
                         ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN);

            /* ==============================================================
             * Шаг 12. Startup delay 200 мс (LN-режим).
             * ============================================================== */
            ICM_DelayMs(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}
/* ===========================================================================
 *  ICM_StartBurstRead — ПАРАЛЛЕЛЬНЫЙ старт всех трёх шин
 *
 *  Все три шины (SPI1, SPI5, SPI4) запускаются одновременно.
 *  Каждая шина работает независимо, опрашивая свои 6 датчиков.
 *  g_fifo_batch_ready выставляется только когда все три шины завершили.
 * ========================================================================== */
void ICM_StartBurstRead(void)
{
    uint8_t first1, first5, first4;

    if ((g_fifo_batch_ready != 0U) || (g_dma_cycle_active != 0U))
    {
        g_tim6_skip_count++;
        return;
    }

    g_dma_cycle_active = 1U;
    g_fifo_batch_ready = 0U;

    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;

    /* Сброс eot_handled перед каждым новым циклом */
    g_bus_spi1.eot_handled = 0U;
    g_bus_spi5.eot_handled = 0U;
    g_bus_spi4.eot_handled = 0U;

    first1 = ICM_FindNextHealthy(&g_bus_spi1, 0U);
    first5 = ICM_FindNextHealthy(&g_bus_spi5, 0U);
    first4 = ICM_FindNextHealthy(&g_bus_spi4, 0U);

    if (first1 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi1, first1); }
    else                               { ICM_FinishBus(&g_bus_spi1); }

    if (first5 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi5, first5); }
    else                               { ICM_FinishBus(&g_bus_spi5); }

    if (first4 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi4, first4); }
    else                               { ICM_FinishBus(&g_bus_spi4); }
}

/* Обёртка для вызова из TIM6 ISR */
void ICM_StartBurstRead_SPI1(void)
{
    ICM_StartBurstRead();
}

/* ===========================================================================
 *  ISR-обёртки DMA RX TC/TE
 * ========================================================================== */
void ICM_DMA_RxComplete_SPI1(void) { ICM_NextSensor(&g_bus_spi1); }
void ICM_DMA_RxComplete_SPI5(void) { ICM_NextSensor(&g_bus_spi5); }
void ICM_DMA_RxComplete_SPI4(void) { ICM_NextSensor(&g_bus_spi4); }

void ICM_DMA_Error_SPI1(void) { g_dma_error_mask |= (1UL << 0U); ICM_NextSensor(&g_bus_spi1); }
void ICM_DMA_Error_SPI5(void) { g_dma_error_mask |= (1UL << 1U); ICM_NextSensor(&g_bus_spi5); }
void ICM_DMA_Error_SPI4(void) { g_dma_error_mask |= (1UL << 2U); ICM_NextSensor(&g_bus_spi4); }

/* ===========================================================================
 *  ISR-обёртки SPI EOT
 * ========================================================================== */
void ICM_SPI_Eot_SPI1(void) { ICM_OnSpiEot(&g_bus_spi1); }
void ICM_SPI_Eot_SPI5(void) { ICM_OnSpiEot(&g_bus_spi5); }
void ICM_SPI_Eot_SPI4(void) { ICM_OnSpiEot(&g_bus_spi4); }

/* ===========================================================================
 *  ICM_StartBusRead (static)
 * ========================================================================== */
static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx)
{
    ICM_Sensor_t *sensor;
    uint8_t       bus_idx;
    uint8_t      *rx_buf;
    uint32_t      t;

    bus_idx = ICM_BusIndex(bus);
    if (bus_idx >= ICM_SPI_BUS_COUNT)   { return; }
    if (idx     >= ICM_SENSORS_PER_BUS) { return; }

    sensor = &bus->sensors[idx];
    rx_buf = g_fifo_data[bus_idx][idx];

    bus->eot_handled = 0U;

    /* 1. Выключить DMA-streams */
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_rx) != 0U) {}
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_tx) != 0U) {}

    /* 2. Сброс флагов DMA */
    ICM_ClearDmaFlags(bus);

    /* 3. Очистка буферов (RAM_D2 — Not Cacheable, Clean/Invalidate не нужны) */
    memset(rx_buf,      0x00U, ICM_FIFO_DMA_BUF_SIZE);
    memset(bus->tx_buf, 0x00U, ICM_FIFO_DMA_BUF_SIZE);
    bus->tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    /* 4. Программирование DMA RX */
    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_rx,
                            LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_rx, (uint32_t)rx_buf);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_rx, ICM_FIFO_DMA_BUF_SIZE);

    /* DMA TX */
    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_tx,
                            LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_tx, (uint32_t)bus->tx_buf);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_tx, ICM_FIFO_DMA_BUF_SIZE);

    /* 5. TC + TE IRQ на RX-stream */
    LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream_rx);

    /* 6. EOT IRQ запрещён до DMA TC */
    LL_SPI_DisableIT_EOT(bus->spi);

    bus->current_sensor_idx = idx;

    /* 7. CS LOW */
    ICM_CS_Low(sensor);
    ICM_DelayUs(2U);

    /* SPE=0 обязателен для SetTransferSize */
    ICM_SPI_EnsureDisabled(bus->spi);

    /* Drain RX FIFO */
    while (LL_SPI_IsActiveFlag_RXP(bus->spi) != 0U)
    {
        (void)LL_SPI_ReceiveData8(bus->spi);
    }
    WRITE_REG(bus->spi->IFCR, 0x0FF8U);

    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);
    LL_SPI_SetInternalSSLevel(bus->spi, LL_SPI_SS_LEVEL_HIGH);

    /* DMA streams ДО SPE=1 */
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_tx);

    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    LL_SPI_Enable(bus->spi);
    __DSB();
    LL_SPI_StartMasterTransfer(bus->spi);
}

/* ===========================================================================
 *  ICM_NextSensor (static) — вызывается из DMA RX TC/TE IRQ
 * ========================================================================== */
static void ICM_NextSensor(ICM_Bus_t *bus)
{
    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    LL_SPI_EnableIT_EOT(bus->spi);

    /* Race: EOT мог выставиться между DMA TC и EnableIT_EOT */
    if (LL_SPI_IsActiveFlag_EOT(bus->spi) != 0U)
    {
        ICM_OnSpiEot(bus);
    }
}

/* ===========================================================================
 *  ICM_OnSpiEot (static)
 * ========================================================================== */
static void ICM_OnSpiEot(ICM_Bus_t *bus)
{
    uint8_t prev_idx;
    uint8_t bus_idx;
    uint8_t next_idx;

    if (LL_SPI_IsActiveFlag_EOT(bus->spi) == 0U) { return; }

    /* Защита от двойного входа (из NextSensor + из SPI_IRQHandler) */
    if (bus->eot_handled != 0U)
    {
        LL_SPI_DisableIT_EOT(bus->spi);
        LL_SPI_ClearFlag_EOT(bus->spi);
        return;
    }
    bus->eot_handled = 1U;

    LL_SPI_DisableIT_EOT(bus->spi);
    LL_SPI_ClearFlag_EOT(bus->spi);
    LL_SPI_ClearFlag_TXTF(bus->spi);
    WRITE_REG(bus->spi->IFCR, 0x0FF8U);

    prev_idx = bus->current_sensor_idx;
    if (prev_idx >= ICM_SENSORS_PER_BUS) { ICM_FinishBus(bus); return; }

    ICM_CS_High(&bus->sensors[prev_idx]);
    LL_SPI_Disable(bus->spi);
    __DSB();

    bus_idx = ICM_BusIndex(bus);
    (void)bus_idx;

    next_idx = ICM_FindNextHealthy(bus, (uint8_t)(prev_idx + 1U));
    if (next_idx < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(bus, next_idx);
    }
    else
    {
        ICM_FinishBus(bus);
    }
}

/* ===========================================================================
 *  ICM_FinishBus / ICM_TryCompleteBatch
 *
 *  ICM_FinishBus больше НЕ запускает следующую шину (была цепочка SPI1→SPI5→SPI4).
 *  Теперь просто выставляет transfer_complete и проверяет готовность всех трёх.
 * ========================================================================== */
static void ICM_FinishBus(ICM_Bus_t *bus)
{
    bus->transfer_complete = 1U;
    ICM_TryCompleteBatch();
}

static void ICM_TryCompleteBatch(void)
{
    if ((g_bus_spi1.transfer_complete != 0U) &&
        (g_bus_spi5.transfer_complete != 0U) &&
        (g_bus_spi4.transfer_complete != 0U))
    {
        g_dma_cycle_active = 0U;
        g_fifo_batch_ready = 1U;
    }
}

/* ===========================================================================
 *  Вспомогательные функции
 * ========================================================================== */

static void ICM_MarkFault(ICM_Sensor_t *s)
{
    s->fault = 1U;
    g_sensor_fault_mask |= (1UL << s->sensor_id);
}

static void ICM_CS_Low(const ICM_Sensor_t *s)
{
    LL_GPIO_ResetOutputPin(s->cs_port, s->cs_pin);
}

static void ICM_CS_High(const ICM_Sensor_t *s)
{
    LL_GPIO_SetOutputPin(s->cs_port, s->cs_pin);
}

static uint8_t ICM_BusIndex(const ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1) { return 0U; }
    if (bus == &g_bus_spi5) { return 1U; }
    return 2U;
}

static uint8_t ICM_FindNextHealthy(const ICM_Bus_t *bus, uint8_t from)
{
    uint8_t i;
    for (i = from; i < ICM_SENSORS_PER_BUS; i++)
    {
        if (bus->sensors[i].fault == 0U) { return i; }
    }
    return ICM_SENSORS_PER_BUS;
}

static void ICM_ClearDmaFlags(const ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1)
    {
        /* SPI1: DMA1 Stream 2 (RX) + Stream 3 (TX) */
        LL_DMA_ClearFlag_TC2(DMA1);  LL_DMA_ClearFlag_HT2(DMA1);
        LL_DMA_ClearFlag_TE2(DMA1);  LL_DMA_ClearFlag_DME2(DMA1);
        LL_DMA_ClearFlag_FE2(DMA1);
        LL_DMA_ClearFlag_TC3(DMA1);  LL_DMA_ClearFlag_HT3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1);  LL_DMA_ClearFlag_DME3(DMA1);
        LL_DMA_ClearFlag_FE3(DMA1);
    }
    else if (bus == &g_bus_spi5)
    {
        /* SPI5: DMA2 Stream 2 (RX) + Stream 3 (TX) */
        LL_DMA_ClearFlag_TC2(DMA2);  LL_DMA_ClearFlag_HT2(DMA2);
        LL_DMA_ClearFlag_TE2(DMA2);  LL_DMA_ClearFlag_DME2(DMA2);
        LL_DMA_ClearFlag_FE2(DMA2);
        LL_DMA_ClearFlag_TC3(DMA2);  LL_DMA_ClearFlag_HT3(DMA2);
        LL_DMA_ClearFlag_TE3(DMA2);  LL_DMA_ClearFlag_DME3(DMA2);
        LL_DMA_ClearFlag_FE3(DMA2);
    }
    else
    {
        /* SPI4: DMA2 Stream 0 (RX) + Stream 1 (TX) */
        LL_DMA_ClearFlag_TC0(DMA2);  LL_DMA_ClearFlag_HT0(DMA2);
        LL_DMA_ClearFlag_TE0(DMA2);  LL_DMA_ClearFlag_DME0(DMA2);
        LL_DMA_ClearFlag_FE0(DMA2);
        LL_DMA_ClearFlag_TC1(DMA2);  LL_DMA_ClearFlag_HT1(DMA2);
        LL_DMA_ClearFlag_TE1(DMA2);  LL_DMA_ClearFlag_DME1(DMA2);
        LL_DMA_ClearFlag_FE1(DMA2);
    }
}

static void ICM_SPI_EnsureDisabled(SPI_TypeDef *spi)
{
    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);
        while (LL_SPI_IsEnabled(spi) != 0U) {}
    }
}

static void ICM_SPI_WaitEOT(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t n)
{
    while (n != 0U)
    {
        while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
        (void)LL_SPI_ReceiveData8(spi);
        n--;
    }
}

static void ICM_DelayUs(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while (cycles > 3U)
    {
        __NOP();
        cycles -= 3U;
    }
}

static void ICM_DelayMs(uint32_t ms)
{
    while (ms != 0U)
    {
        ICM_DelayUs(1000U);
        ms--;
    }
}
