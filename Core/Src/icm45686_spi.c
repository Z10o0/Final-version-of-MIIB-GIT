/* =============================================================================
 * icm45686_spi.c
 *
 * DMA-управляемый опрос 18 датчиков ICM-45686 на трёх шинах SPI.
 * Контроллер: STM32H723ZGT6, D-Cache включён, AXI SRAM D1 (0x24000000).
 *
 * Шины и DMA:
 *   SPI1  RX→DMA1/Stream2   TX→DMA1/Stream3   датчики 0..5
 *   SPI5  RX→DMA2/Stream2   TX→DMA2/Stream3   датчики 6..11
 *   SPI4  RX→DMA2/Stream0   TX→DMA2/Stream1   датчики 12..17
 *
 * Запрещено: HAL. Разрешено: LL-драйверы, прямые регистры.
 *
 * ─── АРХИТЕКТУРА ЗАВЕРШЕНИЯ ОБМЕНА ─────────────────────────────────────────
 *  DMA RX TC  → ICM_NextSensor():
 *      Выключает DMA-запросы и stream. CS НЕ трогает.
 *      Разрешает прерывание EOT периферии SPI.
 *  SPI EOT IRQ → ICM_OnSpiEot():
 *      CS HIGH → SPE=0 → Invalidate кэша → запуск следующего датчика.
 *
 *  Блокирующих циклов в ISR нет. CS поднимается только после
 *  подтверждённого физического окончания TSIZE-транзакции по SCK.
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

static void    ICM_ClearDmaFlags      (const ICM_Bus_t *bus);
static void    ICM_CleanDmaBuffer     (uint8_t *buf, uint32_t size);
static void    ICM_InvalidateDmaBuffer(uint8_t *buf, uint32_t size);

static void    ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx);
static void    ICM_NextSensor  (ICM_Bus_t *bus);
static void    ICM_OnSpiEot    (ICM_Bus_t *bus);
static void    ICM_FinishBus   (ICM_Bus_t *bus);
static void    ICM_MarkFault   (ICM_Sensor_t *s);

/* ===========================================================================
 *  DMA-буферы в AXI SRAM D1 (0x24000000)
 *
 *  Выравнивание 32 байта = размер кэш-линии Cortex-M7.
 *  Без выравнивания SCB_Clean/Invalidate может затронуть соседние данные.
 * ========================================================================== */

/* Приёмные буферы: [шина][датчик][байты].
 * [b][s][0] = мусор (байт адреса FIFO_DATA), полезные данные с [b][s][1]. */
uint8_t g_fifo_data[ICM_SPI_BUS_COUNT]
                   [ICM_SENSORS_PER_BUS]
                   [ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

/* TX-буферы: по одному на шину.
 * [0] = адрес FIFO_DATA | READ_BIT; остальные байты = 0xFF (dummy MOSI). */
static uint8_t g_tx_spi1[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

static uint8_t g_tx_spi5[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

static uint8_t g_tx_spi4[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

/* Флаги состояния (volatile — используются в main и ISR). */
volatile uint8_t  g_fifo_batch_ready  = 0U; /* 1 → пакет 18 датчиков готов  */
volatile uint8_t  g_dma_cycle_active  = 0U; /* 1 → DMA-цикл выполняется      */
volatile uint32_t g_sensor_fault_mask = 0U; /* бит N → датчик N неисправен   */
volatile uint32_t g_dma_error_mask    = 0U; /* бит 0/1/2 → ошибка SPI1/5/4  */

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
    .sensors =
    {
        /* idx=0  Датчик 1  PB12 */
        { SPI1, GPIOB, LL_GPIO_PIN_12, 0U, 0U },
        /* idx=1  Датчик 2  PB13 */
        { SPI1, GPIOB, LL_GPIO_PIN_13, 1U, 0U },
        /* idx=2  Датчик 3  PE8  */
        { SPI1, GPIOE, LL_GPIO_PIN_8,  2U, 0U },
        /* idx=3  Датчик 4  PE9  */
        { SPI1, GPIOE, LL_GPIO_PIN_9,  3U, 0U },
        /* idx=4  Датчик 5  PF13 */
        { SPI1, GPIOF, LL_GPIO_PIN_13, 4U, 0U },
        /* idx=5  Датчик 6  PF14 */
        { SPI1, GPIOF, LL_GPIO_PIN_14, 5U, 0U }
    }
};

ICM_Bus_t g_bus_spi5 =
{
    .spi           = SPI5,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .tx_buf        = g_tx_spi5,
    .sensors =
    {
        /* idx=0  Датчик 7  PE14 */
        { SPI5, GPIOE, LL_GPIO_PIN_14, 6U, 0U },
        /* idx=1  Датчик 8  PE15 */
        { SPI5, GPIOE, LL_GPIO_PIN_15, 7U, 0U },
        /* idx=2  Датчик 9  PE7  */
        { SPI5, GPIOE, LL_GPIO_PIN_7,  8U, 0U },
        /* idx=3  Датчик 10 PG1  */
        { SPI5, GPIOG, LL_GPIO_PIN_1,  9U, 0U },
        /* idx=4  Датчик 11 PB0  */
        { SPI5, GPIOB, LL_GPIO_PIN_0,  10U, 0U },
        /* idx=5  Датчик 12 PB1  */
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
    .sensors =
    {
        /* idx=0  Датчик 13 PE10 */
        { SPI4, GPIOE, LL_GPIO_PIN_10, 12U, 0U },
        /* idx=1  Датчик 14 PE11 */
        { SPI4, GPIOE, LL_GPIO_PIN_11, 13U, 0U },
        /* idx=2  Датчик 15 PF15 */
        { SPI4, GPIOF, LL_GPIO_PIN_15, 14U, 0U },
        /* idx=3  Датчик 16 PG0  */
        { SPI4, GPIOG, LL_GPIO_PIN_0,  15U, 0U },
        /* idx=4  Датчик 17 PC4  */
        { SPI4, GPIOC, LL_GPIO_PIN_4,  16U, 0U },
        /* idx=5  Датчик 18 PC5  */
        { SPI4, GPIOC, LL_GPIO_PIN_5,  17U, 0U }
    }
};

/* ===========================================================================
 *  ICM_BusesInit
 *
 *  Инициализирует TX-буферы, переводит все CS в HIGH, сбрасывает автоматы.
 *  Вызывать ОДИН РАЗ до ICM_InitAllSensors. D-Cache должен быть уже включён.
 * ========================================================================== */
void ICM_BusesInit(void)
{
    uint8_t i;

    /* Обнуление приёмных буферов — исключает мусор при первом чтении. */
    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));

    /* TX: весь буфер = 0xFF (MOSI HIGH в dummy-фазе). */
    memset(g_tx_spi1, 0xFFU, sizeof(g_tx_spi1));
    memset(g_tx_spi5, 0xFFU, sizeof(g_tx_spi5));
    memset(g_tx_spi4, 0xFFU, sizeof(g_tx_spi4));

    /* Первый байт TX — команда чтения FIFO_DATA (адрес | флаг READ). */
    g_tx_spi1[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi5[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi4[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    /* Явное присвоение tx_buf — независимость от порядка линковки. */
    g_bus_spi1.tx_buf = g_tx_spi1;
    g_bus_spi5.tx_buf = g_tx_spi5;
    g_bus_spi4.tx_buf = g_tx_spi4;

    /* Все CS → HIGH, сброс fault. */
    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        ICM_CS_High(&g_bus_spi1.sensors[i]);
        ICM_CS_High(&g_bus_spi5.sensors[i]);
        ICM_CS_High(&g_bus_spi4.sensors[i]);

        g_bus_spi1.sensors[i].fault = 0U;
        g_bus_spi5.sensors[i].fault = 0U;
        g_bus_spi4.sensors[i].fault = 0U;
    }

    /* Сброс автоматов шин. */
    g_bus_spi1.current_sensor_idx = 0U;
    g_bus_spi5.current_sensor_idx = 0U;
    g_bus_spi4.current_sensor_idx = 0U;
    g_bus_spi1.transfer_complete  = 0U;
    g_bus_spi5.transfer_complete  = 0U;
    g_bus_spi4.transfer_complete  = 0U;

    g_fifo_batch_ready  = 0U;
    g_dma_cycle_active  = 0U;
    g_sensor_fault_mask = 0U;
    g_dma_error_mask    = 0U;

    /* Flush TX-буферов из D-Cache в SRAM до первой DMA-передачи. */
    ICM_CleanDmaBuffer(g_tx_spi1, sizeof(g_tx_spi1));
    ICM_CleanDmaBuffer(g_tx_spi5, sizeof(g_tx_spi5));
    ICM_CleanDmaBuffer(g_tx_spi4, sizeof(g_tx_spi4));
}

/* ===========================================================================
 *  ICM_WriteReg — блокирующая запись одного регистра
 *
 *  Формат SPI: [ADDR & 0x7F][DATA]  (2 байта, bit7=0 → запись).
 *  STM32H7: TSIZE программируется только при SPE=0 (RM0468 §56.4.7).
 *  Используется ТОЛЬКО при инициализации датчиков (не в рабочем цикле).
 * ========================================================================== */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_SPI_EnsureDisabled(spi);
    LL_SPI_SetTransferSize(spi, 2U);
    /* SSM=1: внутренний NSS HIGH, иначе возможен MODF в master-режиме. */
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, reg & 0x7FU);    /* адрес, bit7=0 → запись */

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 2U);                  /* сливаем RX-FIFO, иначе OVR */

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}
}

/* ===========================================================================
 *  ICM_ReadReg — блокирующее чтение одного регистра
 *
 *  Формат SPI: [ADDR | 0x80][0xFF]  (2 байта, bit7=1 → чтение).
 *  MISO[0] = мусор (во время адресного байта), MISO[1] = значение регистра.
 *  Используется ТОЛЬКО при инициализации.
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
    LL_SPI_TransmitData8(spi, 0xFFU);  /* dummy — 8 тактов SCK для MISO */

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
 *
 *  Официальный TDK метод: BLK_SEL_W → MADDR_W → M_W (три отдельных WriteReg).
 *  После каждой записи M_W необходима задержка ICM45686_IREG_DELAY_US.
 *  BLK_SEL_W сбрасывается в 0x00 по завершении (возврат в MREG1).
 * ========================================================================== */
void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t       addr_h,
                   uint8_t       addr_l,
                   uint8_t       value)
{
    /* Шаг 1: старший байт адреса IREG. */
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);

    /* Шаг 2: младший байт адреса IREG. */
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0, addr_l);

    /* Шаг 3: задержка — датчик должен подготовить IREG к записи. */
    ICM_DelayUs(ICM45686_IREG_DELAY_US);

    /* Шаг 4: запись данных в IREG_DATA. */
    ICM_WriteReg(sensor, ICM45686_REG_IREG_DATA, value);

    /* Шаг 5: задержка после записи (IREG требует ~4–10 мкс). */
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
 *  ICM_InitAllSensors
 *
 *  Официальная последовательность инициализации ICM-45686 (TDK AN-000264):
 *
 *    [Один раз до цикла]
 *    0.  3 мс — power-on delay
 *
 *    [Для каждого датчика]
 *     1. WHO_AM_I   — проверка SPI-связи (0xE9) ДО reset
 *     2. Soft reset — сброс всех регистров в заводское состояние
 *     3. 2 мс       — boot-sequence после reset
 *     4. IREG: IOC_PAD_SCENARIO_OVRD → активация CLKIN на пине 9
 *     5. REG_MISC1  — переключение на внешний клок (OSC_ID = EXT)
 *     6. Polling PLL_RDY — ожидание захвата PLL (таймаут 10 мс)
 *     7. IREG: SMC_CONTROL_0 — timestamp core (ДО PWR_MGMT0)
 *     8. RTC_CONFIG — ODR от внешней clock domain
 *     9. ACCEL/GYRO CONFIG0 — FSR и ODR
 *    10. FIFO config — mode, watermark, timestamp, channels, flush
 *    11. PWR_MGMT0  — Gyro LN + Accel LN
 *    12. 200 мс     — startup delay гироскопа
 *
 *  Возвращает g_sensor_fault_mask (бит N = датчик N не прошёл init).
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

    /* ------------------------------------------------------------------
     * Шаг 0. Power-on delay 3 мс.
     * ICM-45686 требует минимум 3 мс от подачи питания до первой SPI-
     * транзакции для стабилизации внутреннего LDO. Один раз на все датчики.
     * ------------------------------------------------------------------ */
    ICM_DelayMs(3U);

    for (bus_idx = 0U; bus_idx < ICM_SPI_BUS_COUNT; bus_idx++)
    {
        for (sensor_idx = 0U; sensor_idx < ICM_SENSORS_PER_BUS; sensor_idx++)
        {
            ICM_Sensor_t *sensor = &buses[bus_idx]->sensors[sensor_idx];

            /* ==============================================================
             * Шаг 1. WHO_AM_I — проверка связи ДО reset.
             * После power-on датчик отвечает 0xE9 без конфигурации.
             * ============================================================== */
            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I) != ICM45686_WHO_AM_I_VALUE)
            {
                ICM_MarkFault(sensor);
                continue;
            }

            /* ==============================================================
             * Шаг 2. Программный reset (SOFT_RESET_CONFIG в REG_MISC2).
             * Возвращает все пользовательские регистры в заводские значения.
             * ============================================================== */
            ICM_WriteReg(sensor, ICM45686_REG_REG_MISC2, ICM45686_SOFT_RESET);

            /* ==============================================================
             * Шаг 3. 2 мс задержка после reset.
             * ICM45686_RESET_DELAY_US = 2000 (2 мс).
             * ============================================================== */
            ICM_DelayUs(ICM45686_RESET_DELAY_US);

            /* ==============================================================
             * Шаг 4. IREG: IOC_PAD_SCENARIO_OVRD → CLKIN на пине 9.
             *
             * Используется официальный TDK метод доступа к IREG через
             * BLK_SEL_W → MADDR_W → M_W (три отдельных SPI-транзакции).
             *
             * ICM45686_CLKIN_ENABLE_VAL = 0x06:
             *   bit[2:1] = PAD_SCENARIO_OVRD = 0x03 → CLKIN mode
             *   bit[0]   = PAD_SCENARIO_OVRD_EN = 1  → override активен
             * ============================================================== */
            //ICM_WriteIReg(sensor,
            //              ICM45686_IREG_TOP1_ADDR_H,
            //              ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L,
            //              ICM45686_CLKIN_ENABLE_VAL);

            /* ==============================================================
             * Шаг 5. REG_MISC1: переключение на внешний клок.
             * Выполняется ПОСЛЕ IREG-конфигурации пина — датчик должен
             * знать, что пин 9 настроен как CLKIN.
             *
             * INT1_CONFIG1 НЕ трогаем: INT1_STATUS1 читается независимо
             * от маски прерываний.
             * ============================================================== */
            //ICM_WriteReg(sensor,
            //             ICM45686_REG_INT1_CONFIG1,
            //             ICM45686_INT1_PLL_RDY_EN);   /* было: ICM45686_INT1_PLLRDY_EN */

             /* ==============================================================
             * Шаг 6. Polling PLL_RDY в INT1_STATUS1[0].
             *
             * Типовое время захвата PLL = 3–5 мс при 32.768 кГц CLKIN.
             * Шаг опроса: 1 мс. Таймаут: ICM45686_PLL_TIMEOUT_US / 1000.
             * INT1_STATUS1 — регистр read-clear, читать не чаще 1 раза
             * за итерацию.
             * ============================================================== */
            //timeout = ICM45686_PLL_TIMEOUT_US / 1000U;  /* итерации по 1 мс */
            //status  = 0U;

            //do
            //{
                /* Задержка ПЕРЕД чтением: при первой итерации PLL
                 * физически не может быть готов (< 1 мс после OSC_ID). */
            //    ICM_DelayMs(1U);
            //    status = ICM_ReadReg(sensor, ICM45686_REG_INT1_STATUS1);

            //    if ((status & ICM45686_INT1_STATUS_PLL_RDY) != 0U)
            //    {
            //       break;
            //   }

            //    if (timeout > 0U)
           //    {
            //        timeout--;
            //    }
            //}
            //while (timeout != 0U);

            //if ((status & ICM45686_INT1_STATUS_PLL_RDY) == 0U)
            //{
                /* PLL не захватил внешний клок за 10 мс. */
             //   ICM_MarkFault(sensor);
             //   continue;
            //}

            /* ==============================================================
             * Шаг 7. IREG: SMC_CONTROL_0 — настройка timestamp core.
             * IPREG_TOP1 offset 0x58.
             * ICM45686_SMC_CONTROL_0_VALUE: bit[0]=TMST_EN=1.
             * Обязательно ДО PWR_MGMT0.
             * ============================================================== */
            ICM_WriteIReg(sensor,
                          ICM45686_IREG_TOP1_ADDR_H,
                          ICM45686_IREG_SMC_CONTROL_0_L,
                          ICM45686_SMC_CONTROL_0_VALUE);

            /* ==============================================================
             * Шаг 8. RTC_CONFIG: RTC_MODE_EN (bit5).
             * ODR генерируется от внешней 32.768 кГц clock domain.
             * ============================================================== */
            //ICM_WriteReg(sensor,
            //             ICM45686_REG_RTC_CONFIG,
            //             ICM45686_RTC_MODE_EN);

            /* ==============================================================
             * Шаг 9. FSR и ODR.
             * Значения из icm45686_config.h. Для смены на 6400 Гц —
             * изменить только ICM_GYRO_ODR_VALUE и ICM_ACCEL_ODR_VALUE.
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

            /* 10а. Режим Stream (старые данные перезаписываются) + 2 KiB. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG0,
                         ICM45686_FIFO_MODE_STREAM | ICM45686_FIFO_DEPTH_2K);

            /* 10б. Watermark LOW байт. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG10,           /* было: FIFO_CONFIG1_0 */
                         (uint8_t)(ICM_FIFO_WATERMARK_BYTES & 0x00FFU));

            /* 10б. Watermark HIGH байт. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG11,           /* было: FIFO_CONFIG1_1 */
                         (uint8_t)((ICM_FIFO_WATERMARK_BYTES >> 8U) & 0x00FFU));

            /* 10в. Timestamp в FIFO-пакете. Compression выкл → 16 байт/пакет. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG4,
                         ICM45686_FIFO_TMST_FSYNC_EN);

            /* 10г. Каналы accel + gyro (без IF_EN — сначала настройка). */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN);

            /* 10д. FIFO_IF_EN — включение интерфейса FIFO. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN |
                         ICM45686_FIFO_IF_EN);

            /* 10е. Flush — очистка FIFO перед началом накопления. */
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
             * Шаг 12. Startup delay 200 мс (LN-режим, DS §3.1).
             * ============================================================== */
            ICM_DelayMs(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}

/* ===========================================================================
 *  ICM_StartBurstRead
 *
 *  Запуск DMA-цикла опроса. Вызывается из ISR TIM6 (~3.125 мс = 320 Гц
 *  при ODR=3200 Гц и 10 пакетах в FIFO).
 *
 *  Последовательность шин: SPI1 → SPI5 → SPI4 (последовательно, не параллельно —
 *  экономия шины AHB, предсказуемый latency).
 * ========================================================================== */
void ICM_StartBurstRead(void)
{
    uint8_t first;

    /* Не запускаем новый цикл, если предыдущий ещё не обработан main-loop
     * или DMA-цикл уже идёт. */
    if ((g_fifo_batch_ready != 0U) || (g_dma_cycle_active != 0U))
    {
        return;
    }

    g_dma_cycle_active = 1U;
    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;

    first = ICM_FindNextHealthy(&g_bus_spi1, 0U);
    if (first < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(&g_bus_spi1, first);
    }
    else
    {
        /* Все датчики SPI1 неисправны — сразу переходим к SPI5. */
        ICM_FinishBus(&g_bus_spi1);
    }
}

/* Обёртка для вызова из TIM6 ISR (совместимость с main.c). */
void ICM_StartBurstRead_SPI1(void)
{
    ICM_StartBurstRead();
}

/* ===========================================================================
 *  ISR-обработчики DMA RX Transfer Complete
 *  Вызываются из stm32h7xx_it.c при TC на RX-стриме.
 * ========================================================================== */
void ICM_DMA_RxComplete_SPI1(void) { ICM_NextSensor(&g_bus_spi1); }
void ICM_DMA_RxComplete_SPI5(void) { ICM_NextSensor(&g_bus_spi5); }
void ICM_DMA_RxComplete_SPI4(void) { ICM_NextSensor(&g_bus_spi4); }

/* ===========================================================================
 *  ISR-обработчики DMA Transfer Error
 *  Ставят бит ошибки и продвигают автомат (не зависаем на мёртвом датчике).
 * ========================================================================== */
void ICM_DMA_Error_SPI1(void)
{
    g_dma_error_mask |= (1UL << 0U);
    ICM_NextSensor(&g_bus_spi1);
}

void ICM_DMA_Error_SPI5(void)
{
    g_dma_error_mask |= (1UL << 1U);
    ICM_NextSensor(&g_bus_spi5);
}

void ICM_DMA_Error_SPI4(void)
{
    g_dma_error_mask |= (1UL << 2U);
    ICM_NextSensor(&g_bus_spi4);
}

/* ===========================================================================
 *  ISR-обработчики SPI EOT (End Of Transfer)
 *  Вызываются из SPI1/4/5_IRQHandler в stm32h7xx_it.c.
 * ========================================================================== */
void ICM_SPI_Eot_SPI1(void) { ICM_OnSpiEot(&g_bus_spi1); }
void ICM_SPI_Eot_SPI5(void) { ICM_OnSpiEot(&g_bus_spi5); }
void ICM_SPI_Eot_SPI4(void) { ICM_OnSpiEot(&g_bus_spi4); }

/* ===========================================================================
 *  ICM_StartBusRead (static)
 *
 *  Запуск DMA full-duplex чтения FIFO одного датчика.
 *
 *  Порядок (критичен!):
 *   1. Выключить stream (если ещё включены).
 *   2. Сбросить флаги DMA.
 *   3. Invalidate RX (CPU не перетрёт SRAM) + Clean TX (DMA читает SRAM).
 *   4. Программировать адреса/длину DMA.
 *   5. Разрешить TC/TE IRQ на RX-stream.
 *   6. Запретить EOT IRQ (включит ICM_NextSensor после DMA TC).
 *   7. CS LOW → SPE=0 → TSIZE → enable streams → DMA req → SPE=1 → CSTART.
 *
 *  SPE=0 обязателен перед SetTransferSize (RM0468 §56.4.7).
 *  DMA streams включаются ДО SPE=1, чтобы не потерять первые байты.
 * ========================================================================== */
static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx)
{
    ICM_Sensor_t *sensor = &bus->sensors[idx];
    uint8_t       bus_idx = ICM_BusIndex(bus);
    uint8_t      *rx_buf  = g_fifo_data[bus_idx][idx];

    /* 1. Выключить DMA-streams (если остались включёнными). */
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_rx) != 0U) {}
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_tx) != 0U) {}

    /* 2. Сброс TC/HT/TE/DME/FE. */
    ICM_ClearDmaFlags(bus);

    /* 3. Кэш: RX Invalidate (до DMA) + TX Clean (DMA читает SRAM). */
    ICM_InvalidateDmaBuffer(rx_buf, ICM_FIFO_DMA_BUF_SIZE);
    ICM_CleanDmaBuffer(bus->tx_buf, ICM_FIFO_DMA_BUF_SIZE);

    /* 4. RX: SPI DR → rx_buf. */
    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_rx,
                            LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_rx, (uint32_t)rx_buf);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_rx, ICM_FIFO_DMA_BUF_SIZE);

    /* TX: tx_buf → SPI DR. */
    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_tx,
                            LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_tx, (uint32_t)bus->tx_buf);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_tx, ICM_FIFO_DMA_BUF_SIZE);

    /* 5. TC + TE IRQ только на RX-stream. TX-stream крутит SCK, его TC не нужен. */
    LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream_rx);

    /* 6. EOT IRQ запрещён до DMA TC (включит ICM_NextSensor). */
    LL_SPI_DisableIT_EOT(bus->spi);

    /* Запомнить текущий датчик. */
    bus->current_sensor_idx = idx;

    /* 7. CS LOW до старта SPI. */
    ICM_CS_Low(sensor);

    /* SPE=0 обязателен для SetTransferSize (RM0468 §56.4.7). */
    ICM_SPI_EnsureDisabled(bus->spi);
    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);
    LL_SPI_SetInternalSSLevel(bus->spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(bus->spi);
    LL_SPI_ClearFlag_TXTF(bus->spi);

    /* DMA streams ДО SPE=1 — иначе первые байты могут быть потеряны. */
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_tx);

    /* DMA-запросы SPI. */
    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    /* SPI enable + CSTART. DSB — барьер до старта периферии. */
    LL_SPI_Enable(bus->spi);
    __DSB();
    LL_SPI_StartMasterTransfer(bus->spi);
}

/* ===========================================================================
 *  ICM_NextSensor (static)
 *
 *  Вызывается из ISR DMA RX TC/TE.
 *  DMA RX TC означает: все байты уже в RXDR / SRAM.
 *  Но SPI ещё может докручивать последние такты SCK.
 *
 *  Действия:
 *   1. Выключить DMA-запросы SPI и DMA-streams.
 *   2. Разрешить EOT IRQ SPI (CS остаётся LOW).
 *   3. Если EOT уже выставлен — сразу ICM_OnSpiEot (race condition).
 *
 *  CS НЕ поднимается здесь — только в ICM_OnSpiEot после EOT.
 * ========================================================================== */
static void ICM_NextSensor(ICM_Bus_t *bus)
{
    /* Выключить DMA-запросы SPI (SPI больше не запрашивает DMA). */
    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);

    /* Выключить DMA-streams. */
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    /* Разрешить EOT IRQ. CS остаётся LOW до ICM_OnSpiEot. */
    LL_SPI_EnableIT_EOT(bus->spi);

    /* Race: EOT мог выставиться между DMA TC и EnableIT_EOT.
     * Если флаг уже активен — обрабатываем сразу (IRQ не придёт). */
    if (LL_SPI_IsActiveFlag_EOT(bus->spi) != 0U)
    {
        ICM_OnSpiEot(bus);
    }
}

/* ===========================================================================
 *  ICM_OnSpiEot (static)
 *
 *  Вызывается из SPIx_IRQHandler при EOT (End Of Transfer = TSIZE байт
 *  физически ушли по SCK).
 *
 *  Действия:
 *   1. Проверить EOT (защита от ложных IRQ / OVR).
 *   2. Запретить EOT IRQ, сбросить флаги.
 *   3. CS HIGH — транзакция завершена.
 *   4. SPE=0 — SPI готов к следующему SetTransferSize.
 *   5. DSB — барьер.
 *   6. Invalidate RX-буфера — CPU читает актуальные данные из SRAM.
 *   7. Следующий здоровый датчик или ICM_FinishBus.
 *
 *  Блокирующих циклов нет. Безопасно для ISR.
 * ========================================================================== */
static void ICM_OnSpiEot(ICM_Bus_t *bus)
{
    uint8_t prev;
    uint8_t bus_idx;
    uint8_t next;

    /* Защита: EOT должен быть активен. Иначе — ложный IRQ (OVR и т.п.). */
    if (LL_SPI_IsActiveFlag_EOT(bus->spi) == 0U)
    {
        return;
    }

    /* Запретить EOT IRQ + сброс флагов. */
    LL_SPI_DisableIT_EOT(bus->spi);
    LL_SPI_ClearFlag_EOT(bus->spi);
    LL_SPI_ClearFlag_TXTF(bus->spi);

    /* TSIZE байт ушли по SCK — CS можно поднимать. */
    prev = bus->current_sensor_idx;
    ICM_CS_High(&bus->sensors[prev]);

    /* SPE=0 — SPI готов к SetTransferSize следующего датчика. */
    LL_SPI_Disable(bus->spi);
    __DSB();

    /* Invalidate RX-буфера: между DMA RX TC и EOT D-Cache мог устареть.
     * CPU должен читать из SRAM. */
    bus_idx = ICM_BusIndex(bus);
    ICM_InvalidateDmaBuffer(g_fifo_data[bus_idx][prev], ICM_FIFO_DMA_BUF_SIZE);

    /* Следующий здоровый датчик на этой шине. */
    next = ICM_FindNextHealthy(bus, (uint8_t)(prev + 1U));
    if (next < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(bus, next);
    }
    else
    {
        ICM_FinishBus(bus);
    }
}

/* ===========================================================================
 *  ICM_FinishBus (static)
 *
 *  Завершение опроса одной шины. Цепочка: SPI1 → SPI5 → SPI4.
 *  После SPI4: g_fifo_batch_ready = 1 (main-loop парсит и шлёт UART).
 * ========================================================================== */
static void ICM_FinishBus(ICM_Bus_t *bus)
{
    uint8_t first;

    bus->transfer_complete = 1U;

    if (bus == &g_bus_spi1)
    {
        /* SPI1 готов → запускаем SPI5. */
        first = ICM_FindNextHealthy(&g_bus_spi5, 0U);
        if (first < ICM_SENSORS_PER_BUS)
        {
            ICM_StartBusRead(&g_bus_spi5, first);
        }
        else
        {
            ICM_FinishBus(&g_bus_spi5);
        }
        return;
    }

    if (bus == &g_bus_spi5)
    {
        /* SPI5 готов → запускаем SPI4. */
        first = ICM_FindNextHealthy(&g_bus_spi4, 0U);
        if (first < ICM_SENSORS_PER_BUS)
        {
            ICM_StartBusRead(&g_bus_spi4, first);
        }
        else
        {
            ICM_FinishBus(&g_bus_spi4);
        }
        return;
    }

    /* -------------------------------------------------------------------
     * SPI4 готов → все 18 датчиков опрошены.
     * main-loop увидит g_fifo_batch_ready и вызовет ICM_ParseAllFIFO +
     * UART_SendBatch. Здесь UART НЕ вызываем (ISR должен быть коротким).
     * ------------------------------------------------------------------- */
    g_dma_cycle_active = 0U;
    g_fifo_batch_ready = 1U;
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

/* Выключает SPI если включён. Блокирует до фактического сброса SPE.
 * Допустимо при инициализации и в ICM_StartBusRead. В ISR не использовать. */
static void ICM_SPI_EnsureDisabled(SPI_TypeDef *spi)
{
    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);
        while (LL_SPI_IsEnabled(spi) != 0U) {}
    }
}

/* Блокирующее ожидание EOT. Использовать ТОЛЬКО в блокирующих функциях
 * (WriteReg/ReadReg при init). В ISR запрещено. */
static void ICM_SPI_WaitEOT(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

/* Сброс RX FIFO: N байт отбрасываются. Используется после блокирующих
 * операций write/read во избежание флага OVR. */
static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t n)
{
    while (n != 0U)
    {
        while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
        (void)LL_SPI_ReceiveData8(spi);
        n--;
    }
}

static uint8_t ICM_BusIndex(const ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1) { return 0U; }
    if (bus == &g_bus_spi5) { return 1U; }
    return 2U;
}

/* Линейный поиск первого исправного датчика начиная с from.
 * Возвращает ICM_SENSORS_PER_BUS если все датчики неисправны. */
static uint8_t ICM_FindNextHealthy(const ICM_Bus_t *bus, uint8_t from)
{
    uint8_t i;

    for (i = from; i < ICM_SENSORS_PER_BUS; i++)
    {
        if (bus->sensors[i].fault == 0U)
        {
            return i;
        }
    }

    return ICM_SENSORS_PER_BUS;
}

/* ===========================================================================
 *  ICM_ClearDmaFlags
 *
 *  Сброс TC/HT/TE/DME/FE для RX и TX стримов шины.
 *  Ветвление по указателю: SPI5 и SPI4 оба на DMA2, но разные стримы.
 * ========================================================================== */
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

/* ===========================================================================
 *  ICM_CleanDmaBuffer
 *
 *  Записывает кэш-линии из D-Cache в SRAM (Clean).
 *  Вызывать перед DMA TX: DMA читает из SRAM, а не из кэша CPU.
 *  Адрес и размер выравниваются до границы 32 байт (кэш-линия Cortex-M7).
 * ========================================================================== */
static void ICM_CleanDmaBuffer(uint8_t *buf, uint32_t size)
{
    uint32_t start = (uint32_t)buf & ~31UL;
    uint32_t end   = ((uint32_t)buf + size + 31UL) & ~31UL;

    /* Проверка: буфер должен лежать в D1 AXI SRAM (0x24000000..0x2407FFFF). */
    if ((start < 0x24000000UL) || (end > 0x24080000UL) || (end <= start))
    {
        return;
    }

    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    __DSB();
    __ISB();
}

/* ===========================================================================
 *  ICM_InvalidateDmaBuffer
 *
 *  Инвалидирует кэш-линии (Invalidate).
 *  Вызывать:
 *    - ДО DMA RX (в ICM_StartBusRead): write-back CPU не перетрёт данные DMA.
 *    - ПОСЛЕ DMA RX (в ICM_OnSpiEot): CPU читает актуальные данные из SRAM.
 * ========================================================================== */
static void ICM_InvalidateDmaBuffer(uint8_t *buf, uint32_t size)
{
    uint32_t addr       = (uint32_t)buf & ~31UL;
    uint32_t aligned_sz = (size + 31U + ((uint32_t)buf - addr)) & ~31UL;

    SCB_InvalidateDCache_by_Addr((uint32_t *)addr, (int32_t)aligned_sz);
    __DSB();
    __ISB();
}

/* ===========================================================================
 *  ICM_DelayUs
 *
 *  Программная задержка. Используется ТОЛЬКО при инициализации.
 *  Точность зависит от SystemCoreClock (550 МГц → ~1 итерация ≈ 3 такта).
 *  Для точных задержек в рабочем коде использовать DWT CYCCNT.
 * ========================================================================== */
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
