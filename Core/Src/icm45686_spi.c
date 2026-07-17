/* =============================================================================
 * icm45686_spi.c
 *
 * DMA-управляемый опрос 18 датчиков ICM-45686 на трёх шинах SPI.
 * Контроллер: STM32H723ZGT6, D-Cache включён, AXI SRAM D1.
 *
 * Шины и DMA:
 *   SPI1  RX→DMA1/Stream2   TX→DMA1/Stream3   датчики 0..5
 *   SPI5  RX→DMA2/Stream2   TX→DMA2/Stream3   датчики 6..11
 *   SPI4  RX→DMA2/Stream0   TX→DMA2/Stream1   датчики 12..17
 *
 * Запрещено: HAL. Разрешено: LL-драйверы, прямые регистры.
 *
 * Порядок инициализации каждого датчика (официальная схема ICM-45686):
 *   3ms boot → WHO_AM_I → reset → 2ms → CLKIN → PLL → SMC → RTC →
 *   ODR/FS → FIFO → flush → PWR LN → 200ms startup
 * =============================================================================
 */

#include "icm45686_spi.h"
#include <string.h>

/* ============================================================================
 *  Прототипы локальных функций
 * ============================================================================ */

static void    ICM_DelayUs(uint32_t us);
static void    ICM_DelayMs(uint32_t ms);

static void    ICM_CS_Low (const ICM_Sensor_t *s);
static void    ICM_CS_High(const ICM_Sensor_t *s);

static void    ICM_SPI_EnsureDisabled(SPI_TypeDef *spi);
static void    ICM_SPI_WaitEOT       (SPI_TypeDef *spi);
static void    ICM_SPI_DrainRx       (SPI_TypeDef *spi, uint32_t n);
static void    ICM_SPI_StopDma       (SPI_TypeDef *spi);

static uint8_t ICM_BusIndex        (const ICM_Bus_t *bus);
static uint8_t ICM_FindNextHealthy (const ICM_Bus_t *bus, uint8_t from);

static void    ICM_ClearDmaFlags      (const ICM_Bus_t *bus);
static void    ICM_CleanDmaBuffer     (uint8_t *buf, uint32_t size);
static void    ICM_InvalidateDmaBuffer(uint8_t *buf, uint32_t size);

static void    ICM_StartBusRead (ICM_Bus_t *bus, uint8_t idx);
static void    ICM_NextSensor   (ICM_Bus_t *bus);
static void    ICM_FinishBus    (ICM_Bus_t *bus);
static void    ICM_MarkFault    (ICM_Sensor_t *s);

/* ============================================================================
 *  DMA-буферы в AXI SRAM D1 (0x24000000)
 *
 *  Выравнивание 32 байта = размер кэш-линии Cortex-M7.
 *  Без выравнивания SCB_Clean/Invalidate может затронуть чужие данные.
 * ============================================================================ */

/* Приёмные буферы: [шина][датчик][байты].
 * [b][s][0] = мусор (байт адреса FIFO_DATA).
 * Полезные данные: [b][s][1 .. ICM_FIFO_DMA_BUF_SIZE-1]. */
uint8_t g_fifo_data[ICM_SPI_BUS_COUNT]
                   [ICM_SENSORS_PER_BUS]
                   [ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

/* TX-буферы: один на шину.
 * [0] = адрес FIFO_DATA | READ_BIT; остальные байты = 0xFF (dummy). */
static uint8_t g_tx_spi1[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

static uint8_t g_tx_spi5[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

static uint8_t g_tx_spi4[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1"), aligned(32)));

/* Флаги состояния, видимые из main и ISR. */
volatile uint8_t  g_fifo_batch_ready  = 0U; /* 1 → пакет 18 датчиков готов   */
volatile uint8_t  g_dma_cycle_active  = 0U; /* 1 → DMA-цикл выполняется       */
volatile uint32_t g_sensor_fault_mask = 0U; /* бит N → датчик N не отвечает   */
volatile uint32_t g_dma_error_mask    = 0U; /* бит 0/1/2 → ошибка SPI1/5/4   */

/* ============================================================================
 *  Описание шин
 *
 *  global sensor_id:
 *    SPI1  0..5   CS36/CS35/CS33/CS34/CS31/CS32  (PB12,PB13,PE8,PE9,PF13,PF14)
 *    SPI5  6..11  CS29/CS30/CS27/CS28/CS25/CS26  (PE14,PE15,PE7,PG1,PB0,PB1)
 *    SPI4  12..17 CS23/CS24/CS22/CS21/CS19/CS20  (PE10,PE11,PF15,PG0,PC4,PC5)
 * ============================================================================ */

ICM_Bus_t g_bus_spi1 =
{
    .spi           = SPI1,
    .dma           = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .tx_buf        = g_tx_spi1,
    .sensors =
    {
        { SPI1, CS36_GPIO_Port, CS36_Pin,  0U, 0U }, /* PB12 */
        { SPI1, CS35_GPIO_Port, CS35_Pin,  1U, 0U }, /* PB13 */
        { SPI1, CS33_GPIO_Port, CS33_Pin,  2U, 0U }, /* PE8  */
        { SPI1, CS34_GPIO_Port, CS34_Pin,  3U, 0U }, /* PE9  */
        { SPI1, CS31_GPIO_Port, CS31_Pin,  4U, 0U }, /* PF13 */
        { SPI1, CS32_GPIO_Port, CS32_Pin,  5U, 0U }  /* PF14 */
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
        { SPI5, CS29_GPIO_Port, CS29_Pin,  6U, 0U }, /* PE14 */
        { SPI5, CS30_GPIO_Port, CS30_Pin,  7U, 0U }, /* PE15 */
        { SPI5, CS27_GPIO_Port, CS27_Pin,  8U, 0U }, /* PE7  */
        { SPI5, CS28_GPIO_Port, CS28_Pin,  9U, 0U }, /* PG1  */
        { SPI5, CS25_GPIO_Port, CS25_Pin, 10U, 0U }, /* PB0  */
        { SPI5, CS26_GPIO_Port, CS26_Pin, 11U, 0U }  /* PB1  */
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
        { SPI4, CS23_GPIO_Port, CS23_Pin, 12U, 0U }, /* PE10 */
        { SPI4, CS24_GPIO_Port, CS24_Pin, 13U, 0U }, /* PE11 */
        { SPI4, CS22_GPIO_Port, CS22_Pin, 14U, 0U }, /* PF15 */
        { SPI4, CS21_GPIO_Port, CS21_Pin, 15U, 0U }, /* PG0  */
        { SPI4, CS19_GPIO_Port, CS19_Pin, 16U, 0U }, /* PC4  */
        { SPI4, CS20_GPIO_Port, CS20_Pin, 17U, 0U }  /* PC5  */
    }
};

/* ============================================================================
 *  ICM_BusesInit
 *
 *  Инициализирует TX-буферы и переводит все CS в HIGH.
 *  Вызывать ОДИН РАЗ до ICM_InitAllSensors.
 *  D-Cache должен быть уже включён.
 * ============================================================================ */
void ICM_BusesInit(void)
{
    uint8_t i;

    /* Обнуление приёмных буферов (исключает мусор при первом чтении). */
    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));

    /* TX: весь буфер = 0xFF (MOSI HIGH в dummy-фазе). */
    memset(g_tx_spi1, 0xFFU, sizeof(g_tx_spi1));
    memset(g_tx_spi5, 0xFFU, sizeof(g_tx_spi5));
    memset(g_tx_spi4, 0xFFU, sizeof(g_tx_spi4));

    /* Первый байт TX — команда чтения FIFO_DATA. */
    g_tx_spi1[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi5[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi4[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    /* Указатели tx_buf уже инициализированы в определении структур,
     * но явно дублируем для независимости от порядка линковки. */
    g_bus_spi1.tx_buf = g_tx_spi1;
    g_bus_spi5.tx_buf = g_tx_spi5;
    g_bus_spi4.tx_buf = g_tx_spi4;

    /* Все CS → HIGH (неактивное состояние).
     * Сброс fault: датчики считаются исправными до init. */
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

    /* Записываем TX-буферы из кэша в SRAM до первой DMA-передачи. */
    ICM_CleanDmaBuffer(g_tx_spi1, sizeof(g_tx_spi1));
    ICM_CleanDmaBuffer(g_tx_spi5, sizeof(g_tx_spi5));
    ICM_CleanDmaBuffer(g_tx_spi4, sizeof(g_tx_spi4));
}

/* ============================================================================
 *  ICM_WriteReg — блокирующая запись одного регистра
 *
 *  Формат SPI: [ADDR & 0x7F][DATA]  (2 байта, bit7=0 → запись).
 *  STM32H7: TSIZE программируется только при SPE=0 (RM0468 §56.4.7).
 * ============================================================================ */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_SPI_EnsureDisabled(spi);

    LL_SPI_SetTransferSize(spi, 2U);
    /* SSM=1: внутренний NSS должен быть HIGH, иначе возможен MODF. */
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, reg & 0x7FU);          /* адрес, bit7=0 */

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, value);                 /* данные */

    ICM_SPI_WaitEOT(spi);

    /* Full-duplex: два принятых байта сливаем во избежание OVR. */
    ICM_SPI_DrainRx(spi, 2U);

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}
}

/* ============================================================================
 *  ICM_ReadReg — блокирующее чтение одного регистра
 *
 *  Формат SPI: [ADDR | 0x80][0xFF]  (2 байта, bit7=1 → чтение).
 *  MISO[0]=мусор, MISO[1]=значение регистра.
 * ============================================================================ */
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
    ICM_DelayMs(3U);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, (reg & 0x7FU) | ICM45686_SPI_READ_BIT);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, 0xFFU);  /* dummy — генерирует 8 тактов SCK */

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

/* ============================================================================
 *  ICM_WriteIReg — блокирующая запись Internal Register (IREG)
 *
 *  IREG требует ОДНОЙ burst-транзакции: [0x7C][ADDR_H][ADDR_L][DATA].
 *  Нельзя разбивать на несколько WriteReg — датчик считает адрес
 *  только при непрерывной CS-LOW транзакции.
 *  После записи обязательна задержка ICM45686_IREG_DELAY_US (~10 мкс).
 * ============================================================================ */
void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t       addr_h,
                   uint8_t       addr_l,
                   uint8_t       value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_SPI_EnsureDisabled(spi);

    LL_SPI_SetTransferSize(spi, 4U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    /* Байт 1: адрес регистра IREG_ADDR_15_8 (запись, bit7=0). */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, ICM45686_REG_IREG_ADDR_15_8 & 0x7FU);

    /* Байт 2: старший байт IREG-адреса. */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, addr_h);

    /* Байт 3: младший байт IREG-адреса. */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, addr_l);

    /* Байт 4: записываемое значение. */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 4U);

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}

    /* Датчик обновляет IREG-регистр за ~4–10 мкс. */
    ICM_DelayUs(ICM45686_IREG_DELAY_US);
}

/* ============================================================================
 *  ICM_ReadIReg — блокирующее чтение Internal Register (IREG)
 *
 *  Записываем адрес двумя WriteReg (допустимо для чтения),
 *  выдерживаем задержку, читаем через IREG_DATA.
 * ============================================================================ */
uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t       addr_h,
                     uint8_t       addr_l)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);
    ICM_DelayUs(ICM45686_IREG_DELAY_US);
    return ICM_ReadReg(sensor, ICM45686_REG_IREG_DATA);
}

/* ============================================================================
 *  ICM_InitAllSensors
 *
 *  Официальная последовательность инициализации ICM-45686 (TDK AN-000264):
 *
 *    [Один раз до цикла]
 *    3 мс — стабилизация питания после power-on
 *
 *    [Для каждого датчика]
 *     1. WHO_AM_I        — проверка живости ДО reset (0xE9)
 *     2. Soft reset      — сброс всех регистров в заводское состояние
 *     3. 2 мс            — boot-sequence после reset
 *     4. IREG CLKIN      — BLK_SEL_W → MADDR_W → M_W (официальный TDK метод)
 *     5. MISC1 OSC_ID    — переключение на внешний клок ПОСЛЕ IREG
 *     6. Polling PLL_RDY — ожидание захвата PLL (таймаут 10 мс, шаг 1 мс)
 *     7. IREG SMC_CONTROL_0 — timestamp core ДО PWR_MGMT0
 *     8. RTC_CONFIG      — ODR от внешней clock domain
 *     9. ACCEL/GYRO CONFIG0 — FSR и ODR
 *    10. FIFO            — mode → watermark → timestamp → channels → IF_EN → flush
 *    11. PWR_MGMT0       — Gyro LN + Accel LN
 *    12. 200 мс          — startup delay гироскопа
 *
 *  Возвращает g_sensor_fault_mask (бит N = датчик N не прошёл init).
 * ============================================================================ */
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
     * ICM-45686 требует минимум 3 мс от подачи питания до первой
     * SPI-транзакции для стабилизации внутреннего LDO.
     * Выполняется ОДИН РАЗ перед обходом всех датчиков.
     * ------------------------------------------------------------------ */
    ICM_DelayMs(3U);

    for (bus_idx = 0U; bus_idx < ICM_SPI_BUS_COUNT; bus_idx++)
    {
        for (sensor_idx = 0U; sensor_idx < ICM_SENSORS_PER_BUS; sensor_idx++)
        {
            ICM_Sensor_t *sensor = &buses[bus_idx]->sensors[sensor_idx];

            /* ==============================================================
             * Шаг 1. WHO_AM_I — проверка SPI-связи ДО reset.
             *
             * После power-on датчик отвечает 0xE9 без какой-либо
             * предварительной конфигурации. Если не отвечает —
             * нет смысла выполнять остальные шаги.
             * ============================================================== */
            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I) != ICM45686_WHO_AM_I_VALUE)
            {
                ICM_MarkFault(sensor);
                continue;
            }

            /* ==============================================================
             * Шаг 2. Программный reset (SOFT_RESET_CONFIG в REG_MISC2).
             *
             * Возвращает все пользовательские регистры в заводские значения.
             * Не сбрасывает OTP и тримминг.
             * ============================================================== */
            ICM_WriteReg(sensor, ICM45686_REG_REG_MISC2, ICM45686_SOFT_RESET);

            /* ==============================================================
             * Шаг 3. Задержка 2 мс после reset.
             *
             * ICM45686_RESET_DELAY_US = 2000 → эквивалент 2 мс.
             * Датчик выполняет внутренний boot и загружает OTP.
             * ============================================================== */
            ICM_DelayUs(ICM45686_RESET_DELAY_US);

            /* ==============================================================
             * Шаг 4. IREG: IOC_PAD_SCENARIO_OVRD → активация CLKIN на пине 9.
             *
             * ВНИМАНИЕ: используется официальный TDK метод доступа к IREG
             * через регистры BLK_SEL_W (0x79) → MADDR_W (0x7A) → M_W (0x7B).
             * Burst-транзакция через IREG_ADDR_15_8 (0x7C) не работает
             * корректно после reset — датчик остаётся в банке MREG1.
             *
             * Целевой регистр: IPREG_TOP1 блок (addr_h = ICM45686_IREG_TOP1_ADDR_H)
             *   offset ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L = 0x30
             *   значение ICM45686_CLKIN_ENABLE_VAL = 0x06
             *     bit[2:1] = PAD_SCENARIO_OVRD = 0x03 → CLKIN mode
             *     bit[0]   = PAD_SCENARIO_OVRD_EN = 1  → override активен
             * ============================================================== */

            /* 4а. Выбираем блок IPREG_TOP1 для записи. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_BLK_SEL_W,
                         ICM45686_IREG_TOP1_ADDR_H);

            /* 4б. Указываем offset регистра внутри блока. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_MADDR_W,
                         ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L);

            /* 4в. Записываем значение. После записи M_W датчик
             *     обновляет IREG-регистр за ~4–10 мкс. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_M_W,
                         ICM45686_CLKIN_ENABLE_VAL);

            /* 4г. Задержка — ждём физического обновления IREG. */
            ICM_DelayUs(ICM45686_IREG_DELAY_US);

            /* 4д. Сброс BLK_SEL_W в 0 — возврат в MREG1.
             *     Без этого последующие обычные WriteReg могут
             *     попасть в неверный банк. */
            ICM_WriteReg(sensor, ICM45686_REG_BLK_SEL_W, 0x00U);

            /* ==============================================================
             * Шаг 5. REG_MISC1: переключение на внешний клок (OSC_ID).
             *
             * Выполняется ПОСЛЕ IREG-конфигурации пина — датчик должен
             * сначала знать что пин 9 настроен как CLKIN, и только потом
             * переключить OSC_ID на внешний источник.
             *
             * ICM45686_OSC_ID_OVRD_EXT_CLK = 0x08
             *   bit[3:0] = OSC_ID = 0x08 → внешний 32.768 кГц CLKIN
             *
             * INT1_CONFIG1 НЕ трогаем: INT1_STATUS1 читается независимо
             * от маски прерываний — маска нужна только для вывода на пин.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_REG_MISC1,
                         ICM45686_OSC_ID_OVRD_EXT_CLK);

            /* ==============================================================
             * Шаг 6. Polling PLL_RDY в INT1_STATUS1[0].
             *
             * Типовое время захвата PLL = 3–5 мс при 32.768 кГц CLKIN.
             * Шаг опроса: 1 мс — компромисс между скоростью и нагрузкой.
             * Таймаут: ICM45686_PLL_TIMEOUT_US / 1000 итераций (= 10 итераций).
             *
             * INT1_STATUS1 — регистр read-clear: читать не чаще 1 раза
             * за итерацию, иначе флаг будет очищен до его фиксации.
             * ============================================================== */
            timeout = ICM45686_PLL_TIMEOUT_US / 1000U;  /* итерации по 1 мс */
            status  = 0U;

            do
            {
                /* Задержка ПЕРЕД чтением: при первой итерации PLL
                 * физически не может быть готов (< 1 мс после OSC_ID). */
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

            if ((status & ICM45686_INT1_STATUS_PLL_RDY) == 0U)
            {
                /* PLL не захватил внешний клок за 10 мс.
                 * Возможные причины: нет сигнала на пине CLKIN,
                 * IREG не записался, ошибочный OSC_ID. */
                ICM_MarkFault(sensor);
                continue;
            }

            /* ==============================================================
             * Шаг 7. IREG: SMC_CONTROL_0 — настройка timestamp core.
             *
             * IPREG_TOP1 offset ICM45686_IREG_SMC_CONTROL_0_L = 0x58
             * ICM45686_SMC_CONTROL_0_VALUE:
             *   bit[4] = ACCEL_LP_CLK_SEL = 0 → RC oscillator в LP
             *   bit[0] = TMST_EN = 1          → timestamp включён
             *
             * Обязательно ДО PWR_MGMT0 — иначе timestamp не синхронизируется
             * с данными в FIFO.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_BLK_SEL_W,
                         ICM45686_IREG_TOP1_ADDR_H);

            ICM_WriteReg(sensor,
                         ICM45686_REG_MADDR_W,
                         ICM45686_IREG_SMC_CONTROL_0_L);

            ICM_WriteReg(sensor,
                         ICM45686_REG_M_W,
                         ICM45686_SMC_CONTROL_0_VALUE);

            ICM_DelayUs(ICM45686_IREG_DELAY_US);

            /* Сброс банка. */
            ICM_WriteReg(sensor, ICM45686_REG_BLK_SEL_W, 0x00U);

            /* ==============================================================
             * Шаг 8. RTC_CONFIG: RTC_MODE_EN (bit5 = 1).
             *
             * Переводит ODR-генератор в режим работы от внешней
             * 32.768 кГц clock domain (CLKIN).
             * Без этого ODR будет от внутреннего RC — нестабильно.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_RTC_CONFIG,
                         ICM45686_RTC_MODE_EN);

            /* ==============================================================
             * Шаг 9. FSR и ODR для акселерометра и гироскопа.
             *
             * Значения из icm45686_config.h (меняй только там):
             *   ICM_ACCEL_FS_VALUE  — диапазон (по умолчанию ±16G)
             *   ICM_ACCEL_ODR_VALUE — частота (по умолчанию 3200 Гц)
             *   ICM_GYRO_FS_VALUE   — диапазон (по умолчанию ±2000 dps)
             *   ICM_GYRO_ODR_VALUE  — частота (по умолчанию 3200 Гц)
             *
             * Для перехода на 6400 Гц: изменить ODR-константы в config.h,
             * этот файл трогать не нужно.
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

            /* 10а. FIFO_CONFIG0: режим Stream (при переполнении —
             *       старые данные затираются) + глубина 2 KiB. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG0,
                         ICM45686_FIFO_MODE_STREAM | ICM45686_FIFO_DEPTH_2K);

            /* 10б. Watermark — LOW байт обязательно первым.
             *       ICM_FIFO_WATERMARK_BYTES = 10 пакетов × 16 байт = 160.
             *       При достижении watermark датчик выставляет FIFO_FULL/WM
             *       в INT1_STATUS. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG10,
                         (uint8_t)(ICM_FIFO_WATERMARK_BYTES & 0x00FFU));

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG11,
                         (uint8_t)((ICM_FIFO_WATERMARK_BYTES >> 8U) & 0x00FFU));

            /* 10в. FIFO_CONFIG4: timestamp в каждом FIFO-пакете.
             *       Compression отключена → фиксированный размер пакета 16 байт.
             *       При включении compression размер пакета станет переменным
             *       и DMA-логику нужно будет переделывать. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG4,
                         ICM45686_FIFO_TMST_FSYNC_EN);

            /* 10г. FIFO_CONFIG3: каналы accel + gyro БЕЗ FIFO_IF_EN.
             *       По даташиту §14.18: IF_EN должен быть установлен
             *       отдельной транзакцией ПОСЛЕ выбора каналов. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN);

            /* 10д. FIFO_CONFIG3: добавляем FIFO_IF_EN.
             *       Активирует FIFO-интерфейс — с этого момента данные
             *       начинают накапливаться в FIFO. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN |
                         ICM45686_FIFO_GYRO_EN  |
                         ICM45686_FIFO_IF_EN);

            /* 10е. FIFO flush — сброс всех накопленных данных.
             *       Без flush первый DMA-пакет может содержать данные
             *       от предыдущего сеанса работы или мусор после reset. */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG2,
                         ICM45686_FIFO_FLUSH);

            /* ==============================================================
             * Шаг 11. PWR_MGMT0: включение Gyro + Accel в Low-Noise режим.
             *
             * Это последний шаг конфигурации — датчик начинает измерения
             * сразу после записи. Все предыдущие регистры должны быть
             * настроены до этой точки.
             * ============================================================== */
            ICM_WriteReg(sensor,
                         ICM45686_REG_PWR_MGMT0,
                         ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN);

            /* ==============================================================
             * Шаг 12. Startup delay 200 мс.
             *
             * Гироскоп ICM-45686 в LN-режиме достигает стабильного
             * выхода через ~200 мс (даташит §3.1, Gyro startup time).
             * До истечения этого времени данные в FIFO невалидны.
             * ============================================================== */
            ICM_DelayMs(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}

/* ============================================================================
 *  ICM_StartBurstRead
 *
 *  Точка запуска DMA-цикла. Вызывается из ISR таймера TIM6 каждые ~3.125 мс.
 *  Запускает каскад: SPI1 → SPI5 → SPI4, каждый датчик через DMA.
 *  Повторный вызов до завершения предыдущего цикла игнорируется.
 * ============================================================================ */
void ICM_StartBurstRead(void)
{
    uint8_t first;

    /* Защита от наложения: предыдущий пакет не забрал main или цикл активен. */
    if ((g_fifo_batch_ready != 0U) || (g_dma_cycle_active != 0U))
    {
        return;
    }

    g_dma_cycle_active           = 1U;
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
        /* Все датчики SPI1 неисправны — переходим к SPI5. */
        ICM_FinishBus(&g_bus_spi1);
    }
}

/* Алиас для вызова из ISR таймера. */
void ICM_StartBurstRead_SPI1(void)
{
    ICM_StartBurstRead();
}

/* ============================================================================
 *  ISR-обработчики: завершение DMA RX
 *
 *  Вызываются из stm32h7xx_it.c при TC (Transfer Complete) RX-стрима.
 * ============================================================================ */
void ICM_DMA_RxComplete_SPI1(void) { ICM_NextSensor(&g_bus_spi1); }
void ICM_DMA_RxComplete_SPI5(void) { ICM_NextSensor(&g_bus_spi5); }
void ICM_DMA_RxComplete_SPI4(void) { ICM_NextSensor(&g_bus_spi4); }

/* ============================================================================
 *  ISR-обработчики: ошибка DMA
 *
 *  Бит 0=SPI1, 1=SPI5, 2=SPI4.
 *  Даже при ошибке продолжаем каскад (датчик уже имеет плохие данные,
 *  но остальные шины продолжают работу).
 * ============================================================================ */
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

/* ============================================================================
 *  ICM_StartBusRead  [static]
 *
 *  Запускает DMA full-duplex для одного датчика:
 *    1. Останавливаем стримы.
 *    2. Сбрасываем флаги.
 *    3. Кэш: Invalidate RX перед DMA, Clean TX перед DMA.
 *    4. Настраиваем адреса и длины.
 *    5. Включаем прерывания TC+TE на RX.
 *    6. CS LOW → TSIZE → DMA enable → DMA-запросы → SPI enable → START.
 *
 *  КРИТИЧНО: SPE=0 перед SetTransferSize (STM32H7 RM0468 §56.4.7).
 *  КРИТИЧНО: DMA стримы стартуют ДО SPI enable.
 * ============================================================================ */
static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx)
{
    ICM_Sensor_t *sensor    = &bus->sensors[idx];
    uint8_t       bus_idx   = ICM_BusIndex(bus);
    uint8_t      *rx_buf    = g_fifo_data[bus_idx][idx];

    /* Останавливаем стримы и ждём фактической остановки. */
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_rx) != 0U) {}
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_tx) != 0U) {}

    /* Сброс всех флагов предыдущей передачи. */
    ICM_ClearDmaFlags(bus);

    /* Кэш-синхронизация:
     *   RX Invalidate — CPU читает SRAM после DMA, а не устаревший кэш.
     *   TX Clean      — DMA читает из SRAM актуальные данные из кэша CPU. */
    ICM_InvalidateDmaBuffer(rx_buf,      ICM_FIFO_DMA_BUF_SIZE);
    ICM_CleanDmaBuffer(bus->tx_buf,      ICM_FIFO_DMA_BUF_SIZE);

    /* Настройка RX: SPI_DR → rx_buf. */
    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_rx,
                            LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_rx,
                            (uint32_t)rx_buf);
    LL_DMA_SetDataLength(bus->dma, bus->dma_stream_rx,
                         ICM_FIFO_DMA_BUF_SIZE);

    /* Настройка TX: tx_buf → SPI_DR. */
    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_tx,
                            LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_tx,
                            (uint32_t)bus->tx_buf);
    LL_DMA_SetDataLength(bus->dma, bus->dma_stream_tx,
                         ICM_FIFO_DMA_BUF_SIZE);

    /* Прерывания только на RX-стриме: TC (переход к следующему датчику)
     * и TE (обработка ошибки). TX-прерывания не нужны. */
    LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream_rx);

    /* Запоминаем текущий датчик. */
    bus->current_sensor_idx = idx;

    /* CS LOW перед включением SPI. */
    ICM_CS_Low(sensor);

    /* SPE=0 обязателен перед TSIZE. */
    ICM_SPI_EnsureDisabled(bus->spi);
    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);

    /* DMA стартуют ДО SPI — иначе первые байты могут быть потеряны. */
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_tx);

    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    LL_SPI_Enable(bus->spi);
    LL_SPI_StartMasterTransfer(bus->spi);
}

/* ============================================================================
 *  ICM_NextSensor  [static, вызывается из ISR]
 *
 *  Завершает текущую DMA-передачу и запускает следующий датчик или
 *  передаёт управление ICM_FinishBus.
 * ============================================================================ */
static void ICM_NextSensor(ICM_Bus_t *bus)
{
    uint8_t prev    = bus->current_sensor_idx;
    uint8_t bus_idx = ICM_BusIndex(bus);
    uint8_t next;

    /* Завершаем SPI: EOT → отключаем DMA-запросы → SPE=0. */
    ICM_SPI_StopDma(bus->spi);

    /* Останавливаем стримы. */
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    /* CS HIGH — освобождаем датчик. */
    ICM_CS_High(&bus->sensors[prev]);

    /* Invalidate кэша RX-буфера: CPU читает SRAM, а не кэш. */
    ICM_InvalidateDmaBuffer(g_fifo_data[bus_idx][prev], ICM_FIFO_DMA_BUF_SIZE);

    /* Ищем следующий исправный датчик. */
    next = ICM_FindNextHealthy(bus, prev + 1U);

    if (next < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(bus, next);
    }
    else
    {
        ICM_FinishBus(bus);
    }
}

/* ============================================================================
 *  ICM_FinishBus  [static]
 *
 *  Цепочка шин: SPI1 → SPI5 → SPI4 → установка флага g_fifo_batch_ready.
 * ============================================================================ */
static void ICM_FinishBus(ICM_Bus_t *bus)
{
    uint8_t first;

    bus->transfer_complete = 1U;

    if (bus == &g_bus_spi1)
    {
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

    /* SPI4 завершена — все 18 датчиков опрошены. */
    g_dma_cycle_active = 0U;
    g_fifo_batch_ready = 1U;
}

/* ============================================================================
 *  Служебные функции
 * ============================================================================ */

/* Помечает датчик неисправным и устанавливает соответствующий бит. */
static void ICM_MarkFault(ICM_Sensor_t *s)
{
    s->fault             = 1U;
    g_sensor_fault_mask |= (1UL << s->sensor_id);
}

/* CS LOW: начало транзакции. */
static void ICM_CS_Low(const ICM_Sensor_t *s)
{
    LL_GPIO_ResetOutputPin(s->cs_port, s->cs_pin);
}

/* CS HIGH: конец транзакции. */
static void ICM_CS_High(const ICM_Sensor_t *s)
{
    LL_GPIO_SetOutputPin(s->cs_port, s->cs_pin);
}

/* Выключает SPI если он включён. TSIZE нельзя писать при SPE=1. */
static void ICM_SPI_EnsureDisabled(SPI_TypeDef *spi)
{
    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);
        while (LL_SPI_IsEnabled(spi) != 0U) {}
    }
}

/* Ожидание физического окончания передачи (все байты ушли по SCK).
 * EOT и TXTF сбрасываются после ожидания. */
static void ICM_SPI_WaitEOT(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

/* Сброс RX FIFO: извлекаем и отбрасываем N байт.
 * Необходимо после блокирующих операций во избежание OVR. */
static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t n)
{
    while (n != 0U)
    {
        while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
        (void)LL_SPI_ReceiveData8(spi);
        n--;
    }
}

/* Завершение DMA-передачи: EOT → отключение DMA-запросов → SPE=0. */
static void ICM_SPI_StopDma(SPI_TypeDef *spi)
{
    ICM_SPI_WaitEOT(spi);
    LL_SPI_DisableDMAReq_RX(spi);
    LL_SPI_DisableDMAReq_TX(spi);
    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}
}

/* Возвращает индекс шины: 0=SPI1, 1=SPI5, 2=SPI4. */
static uint8_t ICM_BusIndex(const ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1) { return 0U; }
    if (bus == &g_bus_spi5) { return 1U; }
    return 2U;
}

/* Линейный поиск первого исправного датчика начиная с from.
 * Возвращает ICM_SENSORS_PER_BUS если все неисправны. */
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

/* ============================================================================
 *  ICM_ClearDmaFlags
 *
 *  Сброс TC/HT/TE/DME/FE для RX и TX стримов шины.
 *  Ветвление строго по указателю на структуру, т.к. SPI5 и SPI4 оба
 *  используют DMA2, но на разных стримах.
 * ============================================================================ */
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

/* ============================================================================
 *  ICM_CleanDmaBuffer
 *
 *  Записывает кэш-линии в SRAM перед DMA TX.
 *  Адрес и размер выравниваются вниз/вверх до границы 32 байт.
 * ============================================================================ */
static void ICM_CleanDmaBuffer(uint8_t *buf, uint32_t size)
{
    /* Граница кэш-линии Cortex-M7 = 32 байта. */
    uint32_t start = (uint32_t)buf & ~31UL;          /* выровнять вниз  */
    uint32_t end   = ((uint32_t)buf + size + 31UL) & ~31UL; /* выровнять вверх */

    /* Проверка: адрес должен лежать в D1 SRAM (0x24000000..0x2407FFFF). */
    if ((start < 0x24000000UL) || (end > 0x24080000UL) || (end <= start))
    {
        /* Буфер вне AXI SRAM — кэш-операция опасна, пропускаем.
         * В продакшне заменить на Error_Handler() с логированием адреса. */
        return;
    }

    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    __DSB();
    __ISB();
}


/* ============================================================================
 *  ICM_InvalidateDmaBuffer
 *
 *  Инвалидирует кэш-линии:
 *   - ДО DMA RX (запуск): чтобы write-back CPU не перетёр данные DMA.
 *   - ПОСЛЕ DMA RX (ICM_NextSensor): CPU читает SRAM, а не кэш.
 * ============================================================================ */
static void ICM_InvalidateDmaBuffer(uint8_t *buf, uint32_t size)
{
    uint32_t addr        = (uint32_t)buf & ~31UL;
    uint32_t aligned_sz  = (size + 31U + ((uint32_t)buf - addr)) & ~31UL;

    SCB_InvalidateDCache_by_Addr((uint32_t *)addr, (int32_t)aligned_sz);
    __DSB();
    __ISB();
}

/* ============================================================================
 *  ICM_DelayUs
 *
 *  Программная задержка на основе счётчика тактов ядра.
 *  Используется ТОЛЬКО при инициализации.
 *  При 550 МГц: ~1 итерация ≈ 3 такта → погрешность < 1%.
 * ============================================================================ */
static void ICM_DelayUs(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;

    while (cycles > 3U)
    {
        __NOP();
        cycles -= 3U;
    }
}

/* Задержка в миллисекундах через ICM_DelayUs. */
static void ICM_DelayMs(uint32_t ms)
{
    while (ms != 0U)
    {
        ICM_DelayUs(1000U);
        ms--;
    }
}
