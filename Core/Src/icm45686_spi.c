/* =============================================================================
 * icm45686_spi.c — ПАРАЛЛЕЛЬНАЯ версия, все 3 шины SPI работают одновременно
 *
 *  1.  WHO_AM_I
 *  2.  Soft Reset (REG_MISC2 bit1)
 *  3.  Ждём INT1_STATUS0 bit7 (RESET_DONE) — надёжнее чем WHO_AM_I
 *  4.  0x30[bit1]=1, [bit0]=0  — AUX1 off
 *  5.  0x31[bit2]=1, [1:0]=10b — INT2 → CLKIN
 *  6.  IREG 0xA268[bit2]=0     — I3C STC mode off
 *  7.  IREG 0xA57B[1:0]=0b10   — accel_src_ctrl = FIR+interp
 *  8.  IREG 0xA4A6[6:5]=0b10   — gyro_src_ctrl  = FIR+interp
 *  9а. IREG 0xA258[bit4|bit0]  — ACCEL_LP_CLK_SEL + tmst_en (оба при CLKIN!)
 *  9б. 0x26[bit6|bit5]=1       — rtc_align + rtc_mode
 * 10.  ACCEL_CONFIG0 + GYRO_CONFIG0
 * 11.  PWR_MGMT0: Gyro LN + Accel LN  ← ДО FIFO
 * 12.  FIFO: flush → config0 → watermark → config4(TMST) → config3 → IF_EN
 *      [FIX] Порядок изменён: FLUSH первый, FIFO_CONFIG4 ДО IF_EN
 * 13.  Startup delay 200 мс
 * =============================================================================
 */

#include "icm45686_spi.h"
#include <string.h>

#define ICM45686_INT1_STATUS0_RESET_DONE    (1U << 7)
#define ICM45686_ACCEL_LP_CLK_SEL           (1U << 4)

static void     ICM_DelayUs            (uint32_t us);
static void     ICM_DelayMs            (uint32_t ms);
static void     ICM_CS_Low             (const ICM_Sensor_t *s);
static void     ICM_CS_High            (const ICM_Sensor_t *s);
static void     ICM_SPI_EnsureDisabled (SPI_TypeDef *spi);
static void     ICM_SPI_WaitEOT        (SPI_TypeDef *spi);
static void     ICM_SPI_DrainRx        (SPI_TypeDef *spi, uint32_t n);
static uint8_t  ICM_BusIndex           (const ICM_Bus_t *bus);
static uint8_t  ICM_FindNextHealthy    (const ICM_Bus_t *bus, uint8_t from);
static void     ICM_ClearDmaFlags      (const ICM_Bus_t *bus);
static void     ICM_WaitIRegReady      (void);
static void     ICM_StartBusRead       (ICM_Bus_t *bus, uint8_t idx);
static void     ICM_OnDmaRxComplete    (ICM_Bus_t *bus);           /* [REWRITE v2] заменяет ICM_NextSensor */
static void     ICM_OnSpiEot           (ICM_Bus_t *bus);
static void     ICM_FinishBus          (ICM_Bus_t *bus);
static void     ICM_TryCompleteBatch   (void);
static void     ICM_MarkFault          (ICM_Sensor_t *s);
static void     ICM_RecoverBus         (ICM_Bus_t *bus);           /* [NEW] */
static uint8_t  ICM_BusTimedOut        (const ICM_Bus_t *bus);     /* [NEW] */
static void     ICM_ServiceReintegration(ICM_Bus_t *bus);          /* [NEW] */
static void     ICM_BusesInit_PrecomputeDescriptors(ICM_Bus_t *bus, uint8_t bus_idx); /* [NEW] */
static uint32_t ICM_DWT_ElapsedUs      (uint32_t cyc_start);       /* [NEW] */
static void     ICM_SetEvent           (uint32_t mask);            /* [NEW] */

#define ICM_SENSOR_MAX_FAULTS_LOCAL     3U     /* число подряд идущих ошибок до изоляции датчика */
#define ICM_REINTEGRATION_CYCLES_LOCAL  500U   /* число watchdog-тиков до повторной попытки reintegration */
#define ICM_DMA_TIMEOUT_US_LOCAL        600U   /* максимально допустимая длительность burst-транзакции, мкс */

volatile uint32_t g_icm_events = 0U;   /* [NEW] атомарный event bitmap */
icm_profile_t      g_icm_profile;       /* [NEW] DWT-профилирование */

uint8_t g_fifo_data[ICM_SPI_BUS_COUNT][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi1[ICM_FIFO_DMA_BUF_SIZE] __attribute__((section(".RAM_D2"), aligned(32)));
static uint8_t g_tx_spi5[ICM_FIFO_DMA_BUF_SIZE] __attribute__((section(".RAM_D2"), aligned(32)));
static uint8_t g_tx_spi4[ICM_FIFO_DMA_BUF_SIZE] __attribute__((section(".RAM_D2"), aligned(32)));

volatile uint8_t  g_fifo_batch_ready  = 0U;
volatile uint8_t  g_dma_cycle_active  = 0U;
volatile uint32_t g_sensor_fault_mask = 0U;
volatile uint32_t g_dma_error_mask    = 0U;
volatile uint32_t g_tim6_skip_count   = 0U;
volatile uint32_t g_clk_ok_mask       = 0U;
volatile uint32_t g_clk_fail_mask     = 0U;

ICM_Bus_t g_bus_spi1 =
{
    .spi           = SPI1,
    .dma           = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .tx_buf        = g_tx_spi1,
    .state         = BUS_IDLE,
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
    .state         = BUS_IDLE,
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
    .state         = BUS_IDLE,
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

/* [NEW] Заполняет icm_dma_desc_t для всех 6 датчиков шины ОДИН РАЗ при старте.
 * Hot path (ICM_StartBusRead) больше не вычисляет адреса — только копирует
 * готовые значения из descriptor'а в регистры DMA. */
static void ICM_BusesInit_PrecomputeDescriptors(ICM_Bus_t *bus, uint8_t bus_idx)
{
    uint8_t i;
    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        bus->dma_desc[i].rx_mem_addr = (uint32_t)g_fifo_data[bus_idx][i];
        bus->dma_desc[i].tx_mem_addr = (uint32_t)bus->tx_buf;
        bus->dma_desc[i].length      = (uint16_t)ICM_FIFO_DMA_BUF_SIZE;
    }
}

/* [NEW] Инициализация DWT cycle counter — используется для DWT-based
 * задержек (см. ICM_DelayUs) и watchdog-таймаутов вместо NOP-loop,
 * непредсказуемого на суперскалярном конвейере Cortex-M7. */
void ICM_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void ICM_BusesInit(void)
{
    uint8_t i;

    /* [NEW] DWT должен быть готов до первого вызова ICM_DelayUs()/watchdog */
    ICM_DWT_Init();

    g_bus_spi1.tx_buf = g_tx_spi1;
    g_bus_spi5.tx_buf = g_tx_spi5;
    g_bus_spi4.tx_buf = g_tx_spi4;

    /* memset здесь — INIT-TIME, не hot path, поэтому остаётся без изменений.
     * Runtime memset() в ICM_StartBusRead() убран полностью (см. ниже):
     * TX-шаблон формируется один раз и больше никогда не перезаписывается. */
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
        g_bus_spi1.sensors[i].fault           = 0U;
        g_bus_spi5.sensors[i].fault           = 0U;
        g_bus_spi4.sensors[i].fault           = 0U;
        g_bus_spi1.sensors[i].fault_count     = 0U;   /* [NEW] */
        g_bus_spi5.sensors[i].fault_count     = 0U;   /* [NEW] */
        g_bus_spi4.sensors[i].fault_count     = 0U;   /* [NEW] */
        g_bus_spi1.sensors[i].reint_countdown = 0U;   /* [NEW] */
        g_bus_spi5.sensors[i].reint_countdown = 0U;   /* [NEW] */
        g_bus_spi4.sensors[i].reint_countdown = 0U;   /* [NEW] */
    }

    /* [NEW] Precompute DMA descriptors — устраняет runtime address math
     * из hot path ICM_StartBusRead(). */
    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi1, 0U);
    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi5, 1U);
    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi4, 2U);

    /* [REWRITE v2] Явная FSM вместо разрозненных флагов */
    g_bus_spi1.state              = BUS_IDLE;
    g_bus_spi5.state              = BUS_IDLE;
    g_bus_spi4.state              = BUS_IDLE;
    g_bus_spi1.current_sensor_idx = 0U;
    g_bus_spi5.current_sensor_idx = 0U;
    g_bus_spi4.current_sensor_idx = 0U;
    g_bus_spi1.timeout_count      = 0U;   /* [NEW] */
    g_bus_spi5.timeout_count      = 0U;   /* [NEW] */
    g_bus_spi4.timeout_count      = 0U;   /* [NEW] */
    g_bus_spi1.dma_error_count    = 0U;   /* [NEW] */
    g_bus_spi5.dma_error_count    = 0U;   /* [NEW] */
    g_bus_spi4.dma_error_count    = 0U;   /* [NEW] */

    /* Оставлено для совместимости со старым кодом отладки/телеметрии.
     * Новой acquisition-логикой уже не читается как источник истины. */
    g_bus_spi1.transfer_complete  = 0U;
    g_bus_spi5.transfer_complete  = 0U;
    g_bus_spi4.transfer_complete  = 0U;
    g_bus_spi1.eot_handled        = 0U;
    g_bus_spi5.eot_handled        = 0U;
    g_bus_spi4.eot_handled        = 0U;

    g_icm_events        = 0U;   /* [NEW] */
    g_fifo_batch_ready  = 0U;
    g_dma_cycle_active  = 0U;
    g_sensor_fault_mask = 0U;
    g_dma_error_mask    = 0U;
    g_tim6_skip_count   = 0U;
    g_clk_ok_mask       = 0U;
    g_clk_fail_mask     = 0U;

    memset(&g_icm_profile, 0, sizeof(g_icm_profile));  /* [NEW] */
}

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

static void ICM_WaitIRegReady(void)
{
    ICM_DelayUs(10U);
}

void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t       addr_h,
                   uint8_t       addr_l,
                   uint8_t       value)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);
    ICM_WaitIRegReady();
    ICM_WriteReg(sensor, ICM45686_REG_IREG_DATA, value);
    ICM_WaitIRegReady();
}

uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t       addr_h,
                     uint8_t       addr_l)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);
    ICM_WaitIRegReady();
    return ICM_ReadReg(sensor, ICM45686_REG_IREG_DATA);
}

/* --------------------------------------------------------------------------
 * ICM_WriteIRegBurst — burst-запись IREG за одну CS-транзакцию.
 *
 * Даташит DS-000577, §14.2, правило 3:
 *   "The above programming steps must be performed in a
 *    single burst-write transaction to prevent an
 *    un-intended read-pre-fetch operation."
 *
 * Формат пакета (5 байт за один CS):
 *   [IREG_ADDR_15_8 | WRITE] [addr_h] [addr_l_value] [IREG_DATA | WRITE] [value]
 *
 * Примечание: ICM_ReadIReg оставляем как есть — по §14.5 адрес и чтение
 * данных намеренно разделены двумя транзакциями (read-prefetch модель).
 * -------------------------------------------------------------------------- */
static void ICM_WriteIRegBurst(ICM_Sensor_t *sensor,
                                uint8_t       addr_h,
                                uint8_t       addr_l,
                                uint8_t       value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_SPI_EnsureDisabled(spi);
    LL_SPI_SetTransferSize(spi, 5U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    /* Байт 0: адрес IREG_ADDR_15_8 (запись) */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, ICM45686_REG_IREG_ADDR_15_8 & 0x7FU);

    /* Байт 1: старшие 8 бит внутреннего адреса */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, addr_h);

    /* Байт 2: младшие 8 бит внутреннего адреса */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, addr_l);

    /* Байт 3: адрес IREG_DATA (запись, auto-increment уже не нужен) */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, ICM45686_REG_IREG_DATA & 0x7FU);

    /* Байт 4: само значение */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 5U);

    ICM_CS_High(sensor);
    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}

    /* Минимальное время между IREG-операциями: 4 мкс (§14.3) */
    ICM_DelayUs(10U);
}

uint32_t ICM_InitAllSensors(void)
{
    ICM_Bus_t * const buses[ICM_SPI_BUS_COUNT] = { &g_bus_spi1, &g_bus_spi5, &g_bus_spi4 };
    uint8_t  bus_idx, sensor_idx, reg_val;
    uint32_t timeout;

    g_sensor_fault_mask = 0U;
    g_clk_ok_mask       = 0U;
    g_clk_fail_mask     = 0U;


    for (bus_idx = 0U; bus_idx < ICM_SPI_BUS_COUNT; bus_idx++)
    {
        for (sensor_idx = 0U; sensor_idx < ICM_SENSORS_PER_BUS; sensor_idx++)
        {
            ICM_Sensor_t *sensor = &buses[bus_idx]->sensors[sensor_idx];

            /* ------------------------------------------------------------------
             * ШАГ 1: WHO_AM_I — проверка связи ДО сброса
             * ------------------------------------------------------------------ */
            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I) != ICM45686_WHO_AM_I_VALUE)
            {
                ICM_MarkFault(sensor);
                continue;
            }

            /* ------------------------------------------------------------------
             * ШАГ 2: Soft Reset
             * ------------------------------------------------------------------ */
            ICM_WriteReg(sensor, ICM45686_REG_REG_MISC2, ICM45686_MISC2_SOFT_RST);

            /* ------------------------------------------------------------------
             * ШАГ 3: Ожидание RESET_DONE (INT1_STATUS0[bit7])
             * Надёжнее чем фиксированная задержка (DS-000577 §7)
             * ------------------------------------------------------------------ */
            timeout = 1000U;
            do {
                ICM_DelayUs(10U);
                reg_val = ICM_ReadReg(sensor, ICM45686_REG_INT1_STATUS0);
                timeout--;
            } while (((reg_val & ICM45686_INT1_STATUS0_RESET_DONE) == 0U) &&
                     (timeout != 0U));

            if (timeout == 0U) { ICM_MarkFault(sensor); continue; }

            /* ------------------------------------------------------------------
             * ШАГ 3.5: Big Endian — SREGDATAENDIANSEL=1 (IREG 0xA267, bit1)
             *
             * DS-000577 §15: по умолчанию Little Endian. Описание регистров
             * в даташите — Big Endian. Устанавливаем первым после сброса,
             * до любых других настроек.
             *
             * [FIX-3] Используем ICM_WriteIRegBurst — единая CS-транзакция.
             * ------------------------------------------------------------------ */
            ICM_WriteIRegBurst(sensor, 0xA2U, 0x67U, 0x02U);

            /* ------------------------------------------------------------------
             * ШАГ 4: AUX1 off
             * IOC_PAD_SCENARIO_AUX_OVRD (0x30):
             *   bit1 (AUX1_ENABLE_OVRD)     = 1 — включаем override
             *   bit0 (AUX1_ENABLE_OVRD_VAL) = 0 — AUX1 выключен
             * ------------------------------------------------------------------ */
            reg_val  = ICM_ReadReg(sensor, ICM45686_REG_IOC_PAD_AUX_OVRD);
            reg_val |= ICM45686_AUX1_ENABLE_OVRD;
            reg_val &= ~ICM45686_AUX1_ENABLE_OVRD_VAL;
            ICM_WriteReg(sensor, ICM45686_REG_IOC_PAD_AUX_OVRD, reg_val);

            /* ------------------------------------------------------------------
             * ШАГ 5: INT2 → CLKIN
             * IOC_PAD_SCENARIO_OVRD (0x31):
             *   bit2 (INT2_CFG_OVRD_EN)  = 1 — включаем override
             *   bits[1:0]                = 0b10 — режим CLKIN
             * ------------------------------------------------------------------ */
            reg_val  = ICM_ReadReg(sensor, ICM45686_REG_IOC_PAD_SCENARIO_OVRD);
            reg_val |= ICM45686_INT2_CFG_OVRD_EN;
            reg_val  = (reg_val & ~0x03U) | ICM45686_INT2_CFG_CLKIN_VAL;
            ICM_WriteReg(sensor, ICM45686_REG_IOC_PAD_SCENARIO_OVRD, reg_val);

            /* ------------------------------------------------------------------
             * ШАГ 5.5 [FIX-5]: RTC_CONFIG (0x26)
             * rtc_align (bit6) = 1 — выравнивание первого пакета по CLKIN
             * rtc_mode  (bit5) = 1 — включаем RTC/CLKIN режим
             * Ранее шаг был описан в комментарии "9б", но не реализован.
             * ------------------------------------------------------------------ */
            ICM_WriteReg(sensor, ICM45686_REG_RTC_CONFIG,
                         ICM45686_RTC_ALIGN_EN | ICM45686_RTC_MODE_EN);

            /* ------------------------------------------------------------------
             * ШАГ 6: I3C STC mode off (IREG 0xA268, bit2 = 0)
             * [FIX-3] ICM_WriteIRegBurst для записи
             * ------------------------------------------------------------------ */
            reg_val  = ICM_ReadIReg(sensor,
                                    ICM45686_IREG_I3C_STC_CFG_H,
                                    ICM45686_IREG_I3C_STC_CFG_L);
            reg_val &= ~ICM45686_I3C_STC_MODE_BIT;
            ICM_WriteIRegBurst(sensor,
                               ICM45686_IREG_I3C_STC_CFG_H,
                               ICM45686_IREG_I3C_STC_CFG_L,
                               reg_val);

            /* ------------------------------------------------------------------
             * ШАГ 7: Accel source = FIR + interpolation
             * IREG 0xA57B, bits[1:0] = 0b10
             * [FIX-3] ICM_WriteIRegBurst для записи
             * ------------------------------------------------------------------ */
            reg_val  = ICM_ReadIReg(sensor,
                                    ICM45686_IREG_ACCEL_SRC_CTRL_H,
                                    ICM45686_IREG_ACCEL_SRC_CTRL_L);
            reg_val  = (reg_val & ~0x03U) | ICM45686_ACCEL_SRC_FIR_INTERP;
            ICM_WriteIRegBurst(sensor,
                               ICM45686_IREG_ACCEL_SRC_CTRL_H,
                               ICM45686_IREG_ACCEL_SRC_CTRL_L,
                               reg_val);

            /* ------------------------------------------------------------------
             * ШАГ 8: Gyro source = FIR + interpolation
             * IREG 0xA4A6, bits[6:5] = 0b10
             * [FIX-3] ICM_WriteIRegBurst для записи
             * ------------------------------------------------------------------ */
            reg_val  = ICM_ReadIReg(sensor,
                                    ICM45686_IREG_GYRO_SRC_CTRL_H,
                                    ICM45686_IREG_GYRO_SRC_CTRL_L);
            reg_val  = (reg_val & ~ICM45686_GYRO_SRC_CTRL_MASK)
                     | (ICM45686_GYRO_SRC_FIR_INTERP << ICM45686_GYRO_SRC_CTRL_SHIFT);
            ICM_WriteIRegBurst(sensor,
                               ICM45686_IREG_GYRO_SRC_CTRL_H,
                               ICM45686_IREG_GYRO_SRC_CTRL_L,
                               reg_val);

            /* ------------------------------------------------------------------
             * ШАГ 9: ODR + FSR (до питания — допустимо, датчики ещё OFF)
             * ------------------------------------------------------------------ */
            ICM_WriteReg(sensor, ICM45686_REG_ACCEL_CONFIG0,
                         ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE);
            ICM_WriteReg(sensor, ICM45686_REG_GYRO_CONFIG0,
                         ICM_GYRO_FS_VALUE  | ICM_GYRO_ODR_VALUE);

            /* ------------------------------------------------------------------
             * ШАГ 10: PWR_MGMT0 — включаем Gyro LN + Accel LN
             * Питание ДОЛЖНО быть ДО tmst_en и ДО FIFO!
             * ------------------------------------------------------------------ */
            ICM_WriteReg(sensor, ICM45686_REG_PWR_MGMT0,
                         ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN);

            /* Минимум 200 мкс для стабилизации PLL после включения (DS-000577 §5) */
            ICM_DelayUs(500U);

            /* ------------------------------------------------------------------
             * ШАГ 11 [FIX-1]: tmst_en ПОСЛЕ PWR_MGMT0
             *
             * SMC_CONTROL_0 (IREG 0xA258):
             *   bit0 (TMST_EN)           = 1 — запускает сам счётчик
             *   bit4 (ACCEL_LP_CLK_SEL)  = 0 — только для LP-режима, убираем
             *
             * Счётчик требует активного тактирования от датчиков.
             * До PWR_MGMT0 он не тикает → timestamp = 0 в каждом пакете.
             * [FIX-3] ICM_WriteIRegBurst для записи
             * ------------------------------------------------------------------ */
            reg_val  = ICM_ReadIReg(sensor,
                                    ICM45686_IREG_SMC_CONTROL_0_H,
                                    ICM45686_IREG_SMC_CONTROL_0_L);
            reg_val |=  ICM45686_TMST_EN;
            reg_val &= ~ICM45686_ACCEL_LP_CLK_SEL;
            ICM_WriteIRegBurst(sensor,
                               ICM45686_IREG_SMC_CONTROL_0_H,
                               ICM45686_IREG_SMC_CONTROL_0_L,
                               reg_val);

            /* Даём счётчику 500 мкс на инициализацию */
            ICM_DelayUs(500U);

            /* ------------------------------------------------------------------
             * ШАГ 11.5 [FIX-2]: TMST_WOM_CONFIG (0x23)
             *
             * TMST_DELTA_EN (bit3) = 1:
             *   В FIFO пишется DELTA между текущим и предыдущим timestamp.
             *   Без этого бита в пакете всегда 0x0000, даже если счётчик тикает.
             *
             * TMST_RESOL (bit2) = 0:
             *   Разрешение 1 мкс/LSB (максимальная точность).
             *   При 1 — 16 мкс/LSB.
             *
             * Остальные биты [1:0] — WOM, оставляем 0.
             * ------------------------------------------------------------------ */
            ICM_WriteReg(sensor, 0x23U,
                         (1U << 3) |   /* TMST_DELTA_EN = 1 */
                         (0U << 2));   /* TMST_RESOL    = 0 → 1 мкс/LSB */

            /* ------------------------------------------------------------------
             * ШАГ 12: FIFO CONFIGURATION (20-BYTE HIRES MODE)
             *
             * Правильный порядок (DS-000577 §6):
             *   1. IF_EN=0 (остановить)
             *   2. FLUSH
             *   3. Режим и глубина
             *   4. Watermark (LSB первым!)
             *   5. FIFO_CONFIG4 (timestamp enable)
             *   6. Accel+Gyro+HIRES (без IF_EN)
             *   7. + IF_EN (последним!)
             *
             * [FIX-4] Убрана дублирующая запись FIFO_CONFIG4 из шага 9.
             * ------------------------------------------------------------------ */

            /* 1. Останавливаем FIFO-интерфейс */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG3, 0x00U);

            /* 2. Flush — сбрасываем указатели FIFO */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG2, ICM45686_FIFO_FLUSH);
            ICM_DelayUs(100U);

            /* 3. Stream mode + глубина FIFO
             * ICM45686_FIFO_DEPTH_MAX = 0x1E (011110) = безопасное значение
             * (0x1F = 8K только при всех отключённых APEX, DS-000577 §17.28)
             */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG0,
                         ICM45686_FIFO_MODE_STREAM | ICM45686_FIFO_DEPTH_MAX);

            /* 4. Watermark (LSB первым, затем MSB — DS-000577 §17.29) */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_0,
                         (uint8_t)( ICM_FIFO_WATERMARK_BYTES        & 0x00FFU));
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_1,
                         (uint8_t)((ICM_FIFO_WATERMARK_BYTES >> 8U) & 0x00FFU));

            /* 5. Разрешаем timestamp в FIFO-пакете
             * bit1 = FIFO_TMST_FSYNC_EN — вставляет поле в пакет.
             * Работает только совместно с tmst_en=1 (ШАГ 11).
             */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG4,
                         ICM45686_FIFO_TMST_FSYNC_EN);

            /* 6. Включаем Accel + Gyro + HIRES, но IF_EN ещё НЕ ставим */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN |
                         ICM45686_FIFO_GYRO_EN  |
                         ICM45686_FIFO_HIRES_EN);

            /* 7. Последним включаем IF_EN — открываем FIFO-интерфейс */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN  |
                         ICM45686_FIFO_GYRO_EN   |
                         ICM45686_FIFO_HIRES_EN  |
                         ICM45686_FIFO_IF_EN);

            /* ------------------------------------------------------------------
             * ШАГ 13: Startup delay — ждём первые валидные пакеты
             * ICM45686_STARTUP_DELAY_MS = 200 мс (из icm45686_regs.h)
             * ------------------------------------------------------------------ */
            ICM_DelayMs(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}

void ICM_StartBurstRead(void)
{
    uint8_t first1, first5, first4;

    g_icm_profile.tim6_total++;   /* [NEW] */

    /* [REWRITE v2] Занятость цикла определяется через bus->state,
     * а не через пару неатомарных флагов g_fifo_batch_ready/g_dma_cycle_active.
     * Цикл считается активным, если хотя бы одна шина не в BUS_IDLE. */
    if ((g_bus_spi1.state != BUS_IDLE) ||
        (g_bus_spi5.state != BUS_IDLE) ||
        (g_bus_spi4.state != BUS_IDLE))
    {
        g_tim6_skip_count++;
        g_icm_profile.frame_skip_count++;                 /* [NEW] */
        ICM_SetEvent(ICM_EVT_FRAME_SKIP);                  /* [NEW] */
        return;
    }

    g_icm_profile.acq_start_cyc = DWT->CYCCNT;              /* [NEW] */

    /* Совместимость со старым кодом отладки/телеметрии */
    g_dma_cycle_active = 1U;
    g_fifo_batch_ready  = 0U;

    first1 = ICM_FindNextHealthy(&g_bus_spi1, 0U);
    first5 = ICM_FindNextHealthy(&g_bus_spi5, 0U);
    first4 = ICM_FindNextHealthy(&g_bus_spi4, 0U);

    if (first1 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi1, first1); }
    else                               { g_bus_spi1.state = BUS_COMPLETE; ICM_FinishBus(&g_bus_spi1); }

    if (first5 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi5, first5); }
    else                               { g_bus_spi5.state = BUS_COMPLETE; ICM_FinishBus(&g_bus_spi5); }

    if (first4 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi4, first4); }
    else                               { g_bus_spi4.state = BUS_COMPLETE; ICM_FinishBus(&g_bus_spi4); }
}

void ICM_StartBurstRead_SPI1(void) { ICM_StartBurstRead(); }

/* [REWRITE v2] DMA RX Transfer-Complete IRQ теперь маршрутизируется на
 * ICM_OnDmaRxComplete(), а НЕ на старую ICM_NextSensor(), которая могла
 * вызвать ICM_OnSpiEot() напрямую (inline) при уже установленном EOT-флаге,
 * создавая риск двойной обработки. Теперь EOT обрабатывается ИСКЛЮЧИТЕЛЬНО
 * из SPIx_IRQHandler → ICM_SPI_Eot_SPIx() → ICM_OnSpiEot(). */
void ICM_DMA_RxComplete_SPI1(void) { ICM_OnDmaRxComplete(&g_bus_spi1); }
void ICM_DMA_RxComplete_SPI5(void) { ICM_OnDmaRxComplete(&g_bus_spi5); }
void ICM_DMA_RxComplete_SPI4(void) { ICM_OnDmaRxComplete(&g_bus_spi4); }

void ICM_DMA_Error_SPI1(void)
{
    g_dma_error_mask |= (1UL << 0U);
    g_bus_spi1.dma_error_count++;      /* [NEW] */
    g_bus_spi1.state = BUS_ERROR;      /* [NEW] */
    ICM_RecoverBus(&g_bus_spi1);       /* [REWRITE v2] вместо прямого ICM_NextSensor() */
}
void ICM_DMA_Error_SPI5(void)
{
    g_dma_error_mask |= (1UL << 1U);
    g_bus_spi5.dma_error_count++;
    g_bus_spi5.state = BUS_ERROR;
    ICM_RecoverBus(&g_bus_spi5);
}
void ICM_DMA_Error_SPI4(void)
{
    g_dma_error_mask |= (1UL << 2U);
    g_bus_spi4.dma_error_count++;
    g_bus_spi4.state = BUS_ERROR;
    ICM_RecoverBus(&g_bus_spi4);
}

void ICM_SPI_Eot_SPI1(void) { ICM_OnSpiEot(&g_bus_spi1); }
void ICM_SPI_Eot_SPI5(void) { ICM_OnSpiEot(&g_bus_spi5); }
void ICM_SPI_Eot_SPI4(void) { ICM_OnSpiEot(&g_bus_spi4); }

/* ----------------------------------------------------------------------------
 * ICM_StartBusRead — [REWRITE v2]
 *   - убран runtime memset() RX/TX (единственное изменение TX было —
 *     запись первого байта команды, теперь это делается один раз в Init);
 *   - адреса/длина берутся из precomputed descriptor вместо вычисления;
 *   - busy-wait на LL_DMA_IsEnabledStream() ограничен таймаутом spin-counter,
 *     при превышении — переход в BUS_ERROR/BUS_RECOVERY вместо deadlock.
 * -------------------------------------------------------------------------- */
static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx)
{
    ICM_Sensor_t          *sensor;
    const icm_dma_desc_t  *desc;
    uint32_t               spin;

    if (idx >= ICM_SENSORS_PER_BUS) { return; }

    sensor = &bus->sensors[idx];
    desc   = &bus->dma_desc[idx];

    bus->state              = BUS_START_SENSOR;
    bus->current_sensor_idx = idx;
    bus->eot_handled         = 0U; /* совместимость */

    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    /* [REWRITE v2] Timeout-bounded wait вместо бесконечного busy-wait */
    spin = 2000U;
    while ((LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_rx) != 0U) && (spin != 0U)) { spin--; }
    spin = 2000U;
    while ((LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_tx) != 0U) && (spin != 0U)) { spin--; }

    if ((LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_rx) != 0U) ||
        (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_tx) != 0U))
    {
        bus->state = BUS_ERROR;
        ICM_RecoverBus(bus);
        return;
    }

    ICM_ClearDmaFlags(bus);

    /* [REWRITE v2] БЕЗ memset — RX полностью перезаписывается DMA,
     * TX-шаблон уже содержит команду FIFO_DATA|READ в байте 0 и 0xFF
     * во всех остальных байтах (сформирован один раз в ICM_BusesInit). */
    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_rx,
                            LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_rx, desc->rx_mem_addr);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_rx, desc->length);

    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_tx,
                            LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_tx, desc->tx_mem_addr);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_tx, desc->length);

    LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream_rx);
    LL_SPI_DisableIT_EOT(bus->spi);

    ICM_CS_Low(sensor);
    ICM_DelayUs(2U);

    ICM_SPI_EnsureDisabled(bus->spi);

    while (LL_SPI_IsActiveFlag_RXP(bus->spi) != 0U)
        (void)LL_SPI_ReceiveData8(bus->spi);
    WRITE_REG(bus->spi->IFCR, 0x0FF8U);

    LL_SPI_SetTransferSize(bus->spi, desc->length);
    LL_SPI_SetInternalSSLevel(bus->spi, LL_SPI_SS_LEVEL_HIGH);

    LL_DMA_EnableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_tx);

    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    LL_SPI_Enable(bus->spi);
    __DSB();
    LL_SPI_StartMasterTransfer(bus->spi);

    bus->dma_start_cyc = DWT->CYCCNT;   /* [NEW] метка времени для watchdog */
    bus->state          = BUS_DMA_ACTIVE;
}

/* ----------------------------------------------------------------------------
 * ICM_OnDmaRxComplete — [REWRITE v2] заменяет ICM_NextSensor().
 *
 * КЛЮЧЕВОЕ ОТЛИЧИЕ ОТ ОРИГИНАЛА: эта функция НИКОГДА не вызывает
 * ICM_OnSpiEot() напрямую. Раньше ICM_NextSensor() делала:
 *     if (LL_SPI_IsActiveFlag_EOT(bus->spi) != 0U) ICM_OnSpiEot(bus);
 * что создавало inline-путь обработки EOT в дополнение к IRQ-пути,
 * то есть риск двойной обработки одного и того же события. Теперь
 * функция только переводит FSM в BUS_WAIT_EOT и включает EOT IRQ —
 * если EOT физически уже установлен, NVIC доставит SPIx IRQ отдельно
 * и ровно один раз сразу после выхода из этого хендлера.
 * -------------------------------------------------------------------------- */
static void ICM_OnDmaRxComplete(ICM_Bus_t *bus)
{
    if (bus->state != BUS_DMA_ACTIVE)
    {
        /* Неожиданный DMA TC вне фазы BUS_DMA_ACTIVE — fault, не sequencing */
        bus->dma_error_count++;
        bus->state = BUS_ERROR;
        ICM_RecoverBus(bus);
        return;
    }

    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    bus->state = BUS_WAIT_EOT;
    LL_SPI_EnableIT_EOT(bus->spi);
    /* NO direct call to ICM_OnSpiEot() here — architecturally forbidden. */
}

static void ICM_OnSpiEot(ICM_Bus_t *bus)
{
    uint8_t prev_idx;
    uint8_t next_idx;

    if (LL_SPI_IsActiveFlag_EOT(bus->spi) == 0U) { return; }

    /* [REWRITE v2] Защита от EOT не в ожидаемой фазе — раньше эту роль
     * играл eot_handled, теперь состояние FSM само служит защитой. */
    if (bus->state != BUS_WAIT_EOT)
    {
        LL_SPI_DisableIT_EOT(bus->spi);
        LL_SPI_ClearFlag_EOT(bus->spi);
        bus->dma_error_count++;
        return;
    }

    LL_SPI_DisableIT_EOT(bus->spi);
    LL_SPI_ClearFlag_EOT(bus->spi);
    LL_SPI_ClearFlag_TXTF(bus->spi);
    WRITE_REG(bus->spi->IFCR, 0x0FF8U);

    prev_idx = bus->current_sensor_idx;
    if (prev_idx >= ICM_SENSORS_PER_BUS) { bus->state = BUS_COMPLETE; ICM_FinishBus(bus); return; }

    ICM_CS_High(&bus->sensors[prev_idx]);
    LL_SPI_Disable(bus->spi);
    __DSB();

    bus->state = BUS_NEXT_SENSOR;

    next_idx = ICM_FindNextHealthy(bus, (uint8_t)(prev_idx + 1U));
    if (next_idx < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(bus, next_idx);
    }
    else
    {
        bus->state = BUS_COMPLETE;
        ICM_FinishBus(bus);
    }
}

static void ICM_FinishBus(ICM_Bus_t *bus)
{
    bus->transfer_complete = 1U; /* совместимость со старым кодом отладки */
    ICM_TryCompleteBatch();
}

/* [REWRITE v2] Состояние батча теперь определяется через bus->state,
 * событие BATCH_READY выставляется атомарно через ICM_SetEvent(). */
static void ICM_TryCompleteBatch(void)
{
    if ((g_bus_spi1.state == BUS_COMPLETE) &&
        (g_bus_spi5.state == BUS_COMPLETE) &&
        (g_bus_spi4.state == BUS_COMPLETE))
    {
        g_icm_profile.acq_end_cyc     = DWT->CYCCNT;                              /* [NEW] */
        g_icm_profile.acq_lat_last_us = ICM_DWT_ElapsedUs(g_icm_profile.acq_start_cyc); /* [NEW] */
        if (g_icm_profile.acq_lat_last_us > g_icm_profile.acq_lat_max_us)
        {
            g_icm_profile.acq_lat_max_us = g_icm_profile.acq_lat_last_us;
        }
        g_icm_profile.batch_count++;

        g_bus_spi1.state = BUS_IDLE;
        g_bus_spi5.state = BUS_IDLE;
        g_bus_spi4.state = BUS_IDLE;

        g_dma_cycle_active = 0U;
        g_fifo_batch_ready  = 1U;

        ICM_SetEvent(ICM_EVT_BATCH_READY);   /* [NEW] */
    }
}

static void ICM_MarkFault(ICM_Sensor_t *s)
{
    s->fault = 1U;
    g_sensor_fault_mask |= (1UL << s->sensor_id);
}

/* ----------------------------------------------------------------------------
 * [NEW] ICM_RecoverBus — переводит шину в контролируемое восстановление
 * вместо бесконечного зависания при DMA/SPI fault. Если конкретный
 * датчик накопил слишком много ошибок подряд — изолирует его на
 * ICM_REINTEGRATION_CYCLES_LOCAL тиков watchdog'а и продолжает burst
 * со следующего здорового датчика уже в этом же цикле, не дожидаясь
 * следующего TIM6 update.
 * -------------------------------------------------------------------------- */
static void ICM_RecoverBus(ICM_Bus_t *bus)
{
    uint8_t idx = bus->current_sensor_idx;
    uint8_t next_idx;

    if (idx < ICM_SENSORS_PER_BUS)
    {
        ICM_CS_High(&bus->sensors[idx]);
        bus->sensors[idx].fault_count++;
        if (bus->sensors[idx].fault_count >= ICM_SENSOR_MAX_FAULTS_LOCAL)
        {
            ICM_MarkFault(&bus->sensors[idx]);
            bus->sensors[idx].reint_countdown = ICM_REINTEGRATION_CYCLES_LOCAL;
        }
    }

    ICM_SPI_EnsureDisabled(bus->spi);
    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);
    ICM_ClearDmaFlags(bus);
    LL_SPI_DisableIT_EOT(bus->spi);
    LL_SPI_ClearFlag_EOT(bus->spi);

    if      (bus == &g_bus_spi1) { ICM_SetEvent(ICM_EVT_BUS0_FAULT); }
    else if (bus == &g_bus_spi5) { ICM_SetEvent(ICM_EVT_BUS1_FAULT); }
    else                          { ICM_SetEvent(ICM_EVT_BUS2_FAULT); }

    next_idx = ICM_FindNextHealthy(bus, (uint8_t)(idx + 1U));
    if (next_idx < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(bus, next_idx);
    }
    else
    {
        bus->state = BUS_COMPLETE;
        ICM_FinishBus(bus);
    }
}

/* ----------------------------------------------------------------------------
 * [NEW] ICM_BusTimedOut / ICM_ServiceReintegration / ICM_WatchdogTick
 *
 * Раньше в проекте отсутствовал системный watchdog: застрявший DMA/SPI
 * приводил к постоянному deadlock до ручного сброса MCU. Теперь
 * ICM_WatchdogTick() нужно вызывать из main loop (или отдельного
 * низкоприоритетного TIM7 IRQ) с частотой заметно выше 100 Гц —
 * рекомендуется ~1 кГц — чтобы обнаруживать stuck DMA/EOT задолго
 * до следующего TIM6 burst.
 * -------------------------------------------------------------------------- */
static uint8_t ICM_BusTimedOut(const ICM_Bus_t *bus)
{
    if ((bus->state == BUS_DMA_ACTIVE) || (bus->state == BUS_WAIT_EOT))
    {
        if (ICM_DWT_ElapsedUs(bus->dma_start_cyc) > ICM_DMA_TIMEOUT_US_LOCAL)
        {
            return 1U;
        }
    }
    return 0U;
}

static void ICM_ServiceReintegration(ICM_Bus_t *bus)
{
    uint8_t i;
    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        ICM_Sensor_t *s = &bus->sensors[i];
        if ((s->fault != 0U) && (s->reint_countdown != 0U))
        {
            s->reint_countdown--;
            if (s->reint_countdown == 0U)
            {
                /* Даём датчику ещё один шанс. fault_count оставляем:
                 * если ошибка повторится быстро, следующий ICM_RecoverBus()
                 * снова изолирует датчик с полным countdown. */
                s->fault       = 0U;
                s->fault_count = 0U;
            }
        }
    }
}

void ICM_WatchdogTick(void)
{
    ICM_Bus_t *buses[ICM_SPI_BUS_COUNT] = { &g_bus_spi1, &g_bus_spi5, &g_bus_spi4 };
    uint8_t    i;

    for (i = 0U; i < ICM_SPI_BUS_COUNT; i++)
    {
        ICM_Bus_t *bus = buses[i];

        if (ICM_BusTimedOut(bus) != 0U)
        {
            bus->timeout_count++;
            g_icm_profile.dma_timeout_count++;
            ICM_SetEvent(ICM_EVT_DMA_TIMEOUT);
            ICM_RecoverBus(bus);
        }

        ICM_ServiceReintegration(bus);
    }
}

/* ----------------------------------------------------------------------------
 * [NEW] Атомарный event bitmap: ICM_SetEvent (из ISR) / ICM_ConsumeEvents
 * (из main loop) через LDREX/STREX exclusive access. Это устраняет race
 * condition, при которой два независимых volatile-флага
 * (g_fifo_batch_ready, g_dma_cycle_active) могли наблюдаться main loop
 * в полуобновлённом состоянии, если ISR прерывал их запись посередине.
 * -------------------------------------------------------------------------- */
static void ICM_SetEvent(uint32_t mask)
{
    uint32_t prev;
    do {
        prev = __LDREXW((uint32_t *)&g_icm_events);
    } while (__STREXW(prev | mask, (uint32_t *)&g_icm_events) != 0U);
    __DMB();
}

uint32_t ICM_ConsumeEvents(void)
{
    uint32_t events;
    do {
        events = __LDREXW((uint32_t *)&g_icm_events);
    } while (__STREXW(0U, (uint32_t *)&g_icm_events) != 0U);
    __DMB();
    return events;
}

static uint32_t ICM_DWT_ElapsedUs(uint32_t cyc_start)
{
    uint32_t now   = DWT->CYCCNT;
    uint32_t delta = now - cyc_start; /* корректно и при overflow (unsigned) */
    return delta / (SystemCoreClock / 1000000UL);
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
        LL_DMA_ClearFlag_TC2(DMA1); LL_DMA_ClearFlag_HT2(DMA1);
        LL_DMA_ClearFlag_TE2(DMA1); LL_DMA_ClearFlag_DME2(DMA1);
        LL_DMA_ClearFlag_FE2(DMA1);
        LL_DMA_ClearFlag_TC3(DMA1); LL_DMA_ClearFlag_HT3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1); LL_DMA_ClearFlag_DME3(DMA1);
        LL_DMA_ClearFlag_FE3(DMA1);
    }
    else if (bus == &g_bus_spi5)
    {
        LL_DMA_ClearFlag_TC2(DMA2); LL_DMA_ClearFlag_HT2(DMA2);
        LL_DMA_ClearFlag_TE2(DMA2); LL_DMA_ClearFlag_DME2(DMA2);
        LL_DMA_ClearFlag_FE2(DMA2);
        LL_DMA_ClearFlag_TC3(DMA2); LL_DMA_ClearFlag_HT3(DMA2);
        LL_DMA_ClearFlag_TE3(DMA2); LL_DMA_ClearFlag_DME3(DMA2);
        LL_DMA_ClearFlag_FE3(DMA2);
    }
    else
    {
        LL_DMA_ClearFlag_TC0(DMA2); LL_DMA_ClearFlag_HT0(DMA2);
        LL_DMA_ClearFlag_TE0(DMA2); LL_DMA_ClearFlag_DME0(DMA2);
        LL_DMA_ClearFlag_FE0(DMA2);
        LL_DMA_ClearFlag_TC1(DMA2); LL_DMA_ClearFlag_HT1(DMA2);
        LL_DMA_ClearFlag_TE1(DMA2); LL_DMA_ClearFlag_DME1(DMA2);
        LL_DMA_ClearFlag_FE1(DMA2);
    }
}

/* ----------------------------------------------------------------------------
 * [REWRITE v2] ICM_SPI_EnsureDisabled — ограниченный timeout вместо
 * бесконечного while(). Используется и в hot path, и в register R/W;
 * при стакнутом SPI (например аппаратный clock-stretching сбой) функция
 * гарантированно возвращает управление, а не блокирует систему навечно.
 * -------------------------------------------------------------------------- */
static void ICM_SPI_EnsureDisabled(SPI_TypeDef *spi)
{
    uint32_t spin = 5000U;
    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);
        while ((LL_SPI_IsEnabled(spi) != 0U) && (spin != 0U)) { spin--; }
    }
}

/* Используется только в блокирующих регистровых Read/Write (init-time),
 * не в hot acquisition path. Добавлен защитный таймаут для устойчивости
 * при init-time сбое связи с датчиком. */
static void ICM_SPI_WaitEOT(SPI_TypeDef *spi)
{
    uint32_t spin = 100000U;
    while ((LL_SPI_IsActiveFlag_EOT(spi) == 0U) && (spin != 0U)) { spin--; }
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t n)
{
    uint32_t spin;
    while (n != 0U)
    {
        spin = 100000U;
        while ((LL_SPI_IsActiveFlag_RXP(spi) == 0U) && (spin != 0U)) { spin--; }
        (void)LL_SPI_ReceiveData8(spi);
        n--;
    }
}

/* ----------------------------------------------------------------------------
 * [REWRITE v2] ICM_DelayUs/ICM_DelayMs — DWT cycle counter вместо NOP-loop.
 *
 * ПОЧЕМУ: старая реализация "while (cycles > 3) { __NOP(); cycles -= 3; }"
 * предполагает ровно 3 такта на итерацию цикла — это верно только для
 * простого in-order конвейера. На Cortex-M7 (dual-issue, суперскалярный,
 * с branch predictor и I-Cache) фактическое число тактов на итерацию
 * непредсказуемо и зависит от выравнивания кода, состояния предсказателя
 * переходов и конфликтов с кешем инструкций. DWT->CYCCNT считает реальные
 * тактовые циклы ядра напрямую — единственно надёжный способ измерения
 * времени на Cortex-M7 без использования отдельного таймера.
 * -------------------------------------------------------------------------- */
static void ICM_DelayUs(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000UL) * us;
    while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
}

static void ICM_DelayMs(uint32_t ms)
{
    while (ms != 0U) { ICM_DelayUs(1000U); ms--; }
}

static void ICM_CS_Low(const ICM_Sensor_t *s)
{
    LL_GPIO_ResetOutputPin(s->cs_port, s->cs_pin);
}

static void ICM_CS_High(const ICM_Sensor_t *s)
{
    LL_GPIO_SetOutputPin(s->cs_port, s->cs_pin);
}
