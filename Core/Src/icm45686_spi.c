/*
 * MIIB ICM-45686 SPI/DMA acquisition.
 *
 * Sensor ODR и FIFO geometry задаются только через config.h.
 * TIM6 output acquisition: 400 Гц.
 * TIM7 watchdog: 1 кГц.
 *
 * Шесть шин работают параллельно.
 * На каждой шине последовательно обслуживаются шесть датчиков.
 *
 * SPI1...SPI5 DMA buffers: .RAM_D2.
 * SPI6 BDMA buffers: .RAM_D3 / SRAM4.
 *
 * Существующая FSM BUS_* сохранена.
 * Парсинг, усреднение и построение UART-кадра здесь не выполняются.
 *
 * g_fifo_resync_required является sticky diagnostic flag.
 * Полный блокирующий FIFO/register resync из ISR не выполняется.
 */

#include "icm45686_spi.h"

#include <stdint.h>
#include <string.h>

#define ICM45686_INT1_STATUS0_RESET_DONE  (1U << 7)
#define ICM45686_ACCEL_LP_CLK_SEL         (1U << 4)

static void ICM_DelayUs(uint32_t us);
static void ICM_DelayMs(uint32_t ms);

static void ICM_CS_Low(const ICM_Sensor_t *s);
static void ICM_CS_High(const ICM_Sensor_t *s);

static void ICM_SPI_EnsureDisabled(SPI_TypeDef *spi);
static void ICM_SPI_WaitEOT(SPI_TypeDef *spi);
static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t n);

static uint8_t ICM_FindNextHealthy(const ICM_Bus_t *bus, uint8_t from);

static void ICM_ClearDmaFlags(const ICM_Bus_t *bus);
static void ICM_WaitIRegReady(void);

static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx);
static void ICM_OnDmaRxComplete(ICM_Bus_t *bus);
static void ICM_AdvanceSensor(ICM_Bus_t *bus);
static void ICM_OnSpiEot(ICM_Bus_t *bus);

static void ICM_FinishBus(ICM_Bus_t *bus);
static void ICM_TryCompleteBatch(void);

static void ICM_MarkFault(ICM_Sensor_t *s);
static void ICM_RecoverBus(ICM_Bus_t *bus);

static uint8_t ICM_BusTimedOut(const ICM_Bus_t *bus);
static void ICM_ServiceReintegration(ICM_Bus_t *bus);

static void ICM_BusesInit_PrecomputeDescriptors(
    ICM_Bus_t *bus,
    uint8_t bus_idx);

static uint32_t ICM_DWT_ElapsedUs(uint32_t cyc_start);
static void ICM_SetEvent(uint32_t mask);

#define ICM_SENSOR_MAX_FAULTS_LOCAL     3U
#define ICM_REINTEGRATION_CYCLES_LOCAL  500U

/* Таймаут относится к одной DMA-транзакции датчика. */
#define ICM_DMA_TIMEOUT_US_LOCAL        600U

volatile uint8_t g_fifo_resync_required = 0U;

_Static_assert(ICM_FIFO_DMA_BUF_SIZE <= UINT16_MAX,
               "DMA descriptor length field is too small");

volatile uint32_t g_icm_events = 0U;
icm_profile_t g_icm_profile;

/*
 * Размерность сохранена из исходного проекта.
 * SPI6 не использует g_fifo_data[5], но этот резерв остаётся.
 */
uint8_t g_fifo_data
    [ICM_SPI_BUS_COUNT]
    [ICM_SENSORS_PER_BUS]
    [ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

uint8_t g_fifo_data_spi6
    [ICM_SENSORS_PER_BUS]
    [ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D3"), aligned(32)));

static uint8_t g_tx_spi1[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi5[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi4[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi2[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi3[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi6[ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D3"), aligned(32)));

volatile uint8_t g_fifo_batch_ready = 0U;
volatile uint8_t g_dma_cycle_active = 0U;

volatile uint64_t g_sensor_fault_mask = 0U;

volatile uint32_t g_dma_error_mask = 0U;
volatile uint32_t g_tim6_skip_count = 0U;

volatile uint32_t g_clk_ok_mask = 0U;
volatile uint32_t g_clk_fail_mask = 0U;

/* Нижняя плата: global sensor IDs 0...17. */

ICM_Bus_t g_bus_spi1 =
{
    .spi = SPI1,
    .dma = DMA1,
    .bdma = NULL,
    .is_bdma = 0U,

    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,

    .tx_buf = g_tx_spi1,
    .state = BUS_IDLE,
    .eot_handled = 0U,

    .sensors =
    {
        { SPI1, GPIOB, LL_GPIO_PIN_12, 0U, 0U },
        { SPI1, GPIOB, LL_GPIO_PIN_13, 1U, 0U },
        { SPI1, GPIOE, LL_GPIO_PIN_8,  2U, 0U },
        { SPI1, GPIOE, LL_GPIO_PIN_9,  3U, 0U },
        { SPI1, GPIOF, LL_GPIO_PIN_13, 4U, 0U },
        { SPI1, GPIOF, LL_GPIO_PIN_14, 5U, 0U }
    }
};

ICM_Bus_t g_bus_spi5 =
{
    .spi = SPI5,
    .dma = DMA2,
    .bdma = NULL,
    .is_bdma = 0U,

    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,

    .tx_buf = g_tx_spi5,
    .state = BUS_IDLE,
    .eot_handled = 0U,

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
    .spi = SPI4,
    .dma = DMA2,
    .bdma = NULL,
    .is_bdma = 0U,

    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,

    .tx_buf = g_tx_spi4,
    .state = BUS_IDLE,
    .eot_handled = 0U,

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

/* Верхняя плата: global sensor IDs 18...35. */

ICM_Bus_t g_bus_spi3 =
{
    .spi = SPI3,
    .dma = DMA1,
    .bdma = NULL,
    .is_bdma = 0U,

    .dma_stream_rx = LL_DMA_STREAM_6,
    .dma_stream_tx = LL_DMA_STREAM_7,

    .tx_buf = g_tx_spi3,
    .state = BUS_IDLE,
    .eot_handled = 0U,

    .sensors =
    {
        { SPI3, GPIOD, LL_GPIO_PIN_0, 24U, 0U },
        { SPI3, GPIOD, LL_GPIO_PIN_1, 25U, 0U },
        { SPI3, GPIOD, LL_GPIO_PIN_6, 26U, 0U },
        { SPI3, GPIOD, LL_GPIO_PIN_7, 27U, 0U },
        { SPI3, GPIOB, LL_GPIO_PIN_8, 28U, 0U },
        { SPI3, GPIOE, LL_GPIO_PIN_0, 29U, 0U }
    }
};

ICM_Bus_t g_bus_spi2 =
{
    .spi = SPI2,
    .dma = DMA1,
    .bdma = NULL,
    .is_bdma = 0U,

    .dma_stream_rx = LL_DMA_STREAM_4,
    .dma_stream_tx = LL_DMA_STREAM_5,

    .tx_buf = g_tx_spi2,
    .state = BUS_IDLE,
    .eot_handled = 0U,

    .sensors =
    {
        { SPI2, GPIOA, LL_GPIO_PIN_9,  18U, 0U },
        { SPI2, GPIOA, LL_GPIO_PIN_10, 19U, 0U },
        { SPI2, GPIOD, LL_GPIO_PIN_4,  20U, 0U },
        { SPI2, GPIOD, LL_GPIO_PIN_5,  21U, 0U },
        { SPI2, GPIOB, LL_GPIO_PIN_3,  22U, 0U },
        { SPI2, GPIOB, LL_GPIO_PIN_4,  23U, 0U }
    }
};

ICM_Bus_t g_bus_spi6 =
{
    .spi = SPI6,
    .dma = NULL,
    .bdma = BDMA,
    .is_bdma = 1U,

    .dma_stream_rx = LL_BDMA_CHANNEL_0,
    .dma_stream_tx = LL_BDMA_CHANNEL_1,

    .tx_buf = g_tx_spi6,
    .state = BUS_IDLE,
    .eot_handled = 0U,

    .sensors =
    {
        { SPI6, GPIOA, LL_GPIO_PIN_8,  30U, 0U },
        { SPI6, GPIOC, LL_GPIO_PIN_9,  31U, 0U },
        { SPI6, GPIOD, LL_GPIO_PIN_2,  32U, 0U },
        { SPI6, GPIOD, LL_GPIO_PIN_3,  33U, 0U },
        { SPI6, GPIOG, LL_GPIO_PIN_9,  34U, 0U },
        { SPI6, GPIOG, LL_GPIO_PIN_15, 35U, 0U }
    }
};

static void ICM_BusesInit_PrecomputeDescriptors(
    ICM_Bus_t *bus,
    uint8_t bus_idx)
{
    uint8_t i;

    if (bus_idx >= (ICM_SPI_BUS_COUNT - 1U))
    {
        return;
    }

    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        bus->dma_desc[i].rx_mem_addr =
            (uint32_t)g_fifo_data[bus_idx][i];

        bus->dma_desc[i].tx_mem_addr =
            (uint32_t)bus->tx_buf;

        bus->dma_desc[i].length =
            (uint16_t)ICM_FIFO_DMA_BUF_SIZE;
    }
}

void ICM_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void ICM_BusesInit(void)
{
    uint8_t i;

    ICM_DWT_Init();

    g_bus_spi1.tx_buf = g_tx_spi1;
    g_bus_spi5.tx_buf = g_tx_spi5;
    g_bus_spi4.tx_buf = g_tx_spi4;

    g_bus_spi2.tx_buf = g_tx_spi2;
    g_bus_spi3.tx_buf = g_tx_spi3;
    g_bus_spi6.tx_buf = g_tx_spi6;

    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));
    memset(g_fifo_data_spi6, 0x00U, sizeof(g_fifo_data_spi6));

    memset(g_tx_spi1, 0xFFU, sizeof(g_tx_spi1));
    memset(g_tx_spi5, 0xFFU, sizeof(g_tx_spi5));
    memset(g_tx_spi4, 0xFFU, sizeof(g_tx_spi4));

    memset(g_tx_spi2, 0xFFU, sizeof(g_tx_spi2));
    memset(g_tx_spi3, 0xFFU, sizeof(g_tx_spi3));
    memset(g_tx_spi6, 0xFFU, sizeof(g_tx_spi6));

    g_tx_spi1[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi5[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi4[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    g_tx_spi2[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi3[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi6[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        ICM_CS_High(&g_bus_spi1.sensors[i]);
        ICM_CS_High(&g_bus_spi5.sensors[i]);
        ICM_CS_High(&g_bus_spi4.sensors[i]);

        ICM_CS_High(&g_bus_spi2.sensors[i]);
        ICM_CS_High(&g_bus_spi3.sensors[i]);
        ICM_CS_High(&g_bus_spi6.sensors[i]);

        g_bus_spi1.sensors[i].fault = 0U;
        g_bus_spi5.sensors[i].fault = 0U;
        g_bus_spi4.sensors[i].fault = 0U;

        g_bus_spi2.sensors[i].fault = 0U;
        g_bus_spi3.sensors[i].fault = 0U;
        g_bus_spi6.sensors[i].fault = 0U;

        g_bus_spi1.sensors[i].fault_count = 0U;
        g_bus_spi5.sensors[i].fault_count = 0U;
        g_bus_spi4.sensors[i].fault_count = 0U;

        g_bus_spi2.sensors[i].fault_count = 0U;
        g_bus_spi3.sensors[i].fault_count = 0U;
        g_bus_spi6.sensors[i].fault_count = 0U;

        g_bus_spi1.sensors[i].reint_countdown = 0U;
        g_bus_spi5.sensors[i].reint_countdown = 0U;
        g_bus_spi4.sensors[i].reint_countdown = 0U;

        g_bus_spi2.sensors[i].reint_countdown = 0U;
        g_bus_spi3.sensors[i].reint_countdown = 0U;
        g_bus_spi6.sensors[i].reint_countdown = 0U;
    }

    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi1, 0U);
    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi5, 1U);
    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi4, 2U);

    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi2, 3U);
    ICM_BusesInit_PrecomputeDescriptors(&g_bus_spi3, 4U);

    /* SPI6 descriptors используют отдельный D3/SRAM4 buffer. */
    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        g_bus_spi6.dma_desc[i].rx_mem_addr =
            (uint32_t)g_fifo_data_spi6[i];

        g_bus_spi6.dma_desc[i].tx_mem_addr =
            (uint32_t)g_bus_spi6.tx_buf;

        g_bus_spi6.dma_desc[i].length =
            (uint16_t)ICM_FIFO_DMA_BUF_SIZE;
    }

    g_bus_spi1.state = BUS_IDLE;
    g_bus_spi5.state = BUS_IDLE;
    g_bus_spi4.state = BUS_IDLE;

    g_bus_spi2.state = BUS_IDLE;
    g_bus_spi3.state = BUS_IDLE;
    g_bus_spi6.state = BUS_IDLE;

    g_bus_spi1.current_sensor_idx = 0U;
    g_bus_spi5.current_sensor_idx = 0U;
    g_bus_spi4.current_sensor_idx = 0U;

    g_bus_spi2.current_sensor_idx = 0U;
    g_bus_spi3.current_sensor_idx = 0U;
    g_bus_spi6.current_sensor_idx = 0U;

    g_bus_spi1.timeout_count = 0U;
    g_bus_spi5.timeout_count = 0U;
    g_bus_spi4.timeout_count = 0U;

    g_bus_spi2.timeout_count = 0U;
    g_bus_spi3.timeout_count = 0U;
    g_bus_spi6.timeout_count = 0U;

    g_bus_spi1.dma_error_count = 0U;
    g_bus_spi5.dma_error_count = 0U;
    g_bus_spi4.dma_error_count = 0U;

    g_bus_spi2.dma_error_count = 0U;
    g_bus_spi3.dma_error_count = 0U;
    g_bus_spi6.dma_error_count = 0U;

    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;

    g_bus_spi2.transfer_complete = 0U;
    g_bus_spi3.transfer_complete = 0U;
    g_bus_spi6.transfer_complete = 0U;

    g_bus_spi1.eot_handled = 0U;
    g_bus_spi5.eot_handled = 0U;
    g_bus_spi4.eot_handled = 0U;

    g_bus_spi2.eot_handled = 0U;
    g_bus_spi3.eot_handled = 0U;
    g_bus_spi6.eot_handled = 0U;

    g_fifo_resync_required = 0U;

    g_icm_events = 0U;
    g_fifo_batch_ready = 0U;
    g_dma_cycle_active = 0U;

    g_sensor_fault_mask = 0U;
    g_dma_error_mask = 0U;
    g_tim6_skip_count = 0U;

    g_clk_ok_mask = 0U;
    g_clk_fail_mask = 0U;

    memset(&g_icm_profile, 0, sizeof(g_icm_profile));
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

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, reg & 0x7FU);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 2U);

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);

    while (LL_SPI_IsEnabled(spi) != 0U)
    {
    }
}

uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg)
{
    SPI_TypeDef *spi = sensor->spi;

    uint8_t dummy;
    uint8_t result;

    ICM_SPI_EnsureDisabled(spi);

    LL_SPI_SetTransferSize(spi, 2U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(
        spi,
        (reg & 0x7FU) | ICM45686_SPI_READ_BIT);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, 0xFFU);

    ICM_SPI_WaitEOT(spi);

    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U)
    {
    }

    dummy = LL_SPI_ReceiveData8(spi);
    (void)dummy;

    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U)
    {
    }

    result = LL_SPI_ReceiveData8(spi);

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);

    while (LL_SPI_IsEnabled(spi) != 0U)
    {
    }

    return result;
}

static void ICM_WaitIRegReady(void)
{
    ICM_DelayUs(10U);
}

void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t addr_h,
                   uint8_t addr_l,
                   uint8_t value)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0, addr_l);

    ICM_WaitIRegReady();

    ICM_WriteReg(sensor, ICM45686_REG_IREG_DATA, value);

    ICM_WaitIRegReady();
}

uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t addr_h,
                     uint8_t addr_l)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0, addr_l);

    ICM_WaitIRegReady();

    return ICM_ReadReg(sensor, ICM45686_REG_IREG_DATA);
}

static void ICM_WriteIRegBurst(ICM_Sensor_t *sensor,
                               uint8_t addr_h,
                               uint8_t addr_l,
                               uint8_t value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_SPI_EnsureDisabled(spi);

    LL_SPI_SetTransferSize(spi, 5U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);

    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(
        spi,
        ICM45686_REG_IREG_ADDR_15_8 & 0x7FU);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, addr_h);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, addr_l);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, ICM45686_REG_IREG_DATA & 0x7FU);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 5U);

    ICM_CS_High(sensor);

    LL_SPI_Disable(spi);

    while (LL_SPI_IsEnabled(spi) != 0U)
    {
    }

    ICM_DelayUs(10U);
}

uint64_t ICM_InitAllSensors(void)
{
    ICM_Bus_t * const buses[ICM_SPI_BUS_COUNT] =
    {
        &g_bus_spi1,
        &g_bus_spi5,
        &g_bus_spi4,
        &g_bus_spi2,
        &g_bus_spi3,
        &g_bus_spi6
    };

    uint8_t bus_idx;
    uint8_t sensor_idx;
    uint8_t reg_val;

    uint32_t timeout;

    g_sensor_fault_mask = 0U;
    g_clk_ok_mask = 0U;
    g_clk_fail_mask = 0U;

    for (bus_idx = 0U; bus_idx < ICM_SPI_BUS_COUNT; bus_idx++)
    {
        for (sensor_idx = 0U;
             sensor_idx < ICM_SENSORS_PER_BUS;
             sensor_idx++)
        {
            ICM_Sensor_t *sensor =
                &buses[bus_idx]->sensors[sensor_idx];

            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I) !=
                ICM45686_WHO_AM_I_VALUE)
            {
                ICM_MarkFault(sensor);
                continue;
            }

            ICM_WriteReg(
                sensor,
                ICM45686_REG_REG_MISC2,
                ICM45686_MISC2_SOFT_RST);

            timeout = 1000U;

            do
            {
                ICM_DelayUs(10U);

                reg_val = ICM_ReadReg(
                    sensor,
                    ICM45686_REG_INT1_STATUS0);

                timeout--;
            }
            while (((reg_val & ICM45686_INT1_STATUS0_RESET_DONE) == 0U) &&
                   (timeout != 0U));

            if (timeout == 0U)
            {
                ICM_MarkFault(sensor);
                continue;
            }

            /* Исходная настройка Big Endian сохранена. */
            ICM_WriteIRegBurst(sensor, 0xA2U, 0x67U, 0x02U);

            reg_val = ICM_ReadReg(
                sensor,
                ICM45686_REG_IOC_PAD_AUX_OVRD);

            reg_val |= ICM45686_AUX1_ENABLE_OVRD;
            reg_val &= ~ICM45686_AUX1_ENABLE_OVRD_VAL;

            ICM_WriteReg(
                sensor,
                ICM45686_REG_IOC_PAD_AUX_OVRD,
                reg_val);

            reg_val = ICM_ReadReg(
                sensor,
                ICM45686_REG_IOC_PAD_SCENARIO_OVRD);

            reg_val |= ICM45686_INT2_CFG_OVRD_EN;

            reg_val =
                (reg_val & ~0x03U) |
                ICM45686_INT2_CFG_CLKIN_VAL;

            ICM_WriteReg(
                sensor,
                ICM45686_REG_IOC_PAD_SCENARIO_OVRD,
                reg_val);

            ICM_WriteReg(
                sensor,
                ICM45686_REG_RTC_CONFIG,
                ICM45686_RTC_ALIGN_EN | ICM45686_RTC_MODE_EN);

            reg_val = ICM_ReadIReg(
                sensor,
                ICM45686_IREG_I3C_STC_CFG_H,
                ICM45686_IREG_I3C_STC_CFG_L);

            reg_val &= ~ICM45686_I3C_STC_MODE_BIT;

            ICM_WriteIRegBurst(
                sensor,
                ICM45686_IREG_I3C_STC_CFG_H,
                ICM45686_IREG_I3C_STC_CFG_L,
                reg_val);

            reg_val = ICM_ReadIReg(
                sensor,
                ICM45686_IREG_ACCEL_SRC_CTRL_H,
                ICM45686_IREG_ACCEL_SRC_CTRL_L);

            reg_val =
                (reg_val & ~0x03U) |
                ICM45686_ACCEL_SRC_FIR_INTERP;

            ICM_WriteIRegBurst(
                sensor,
                ICM45686_IREG_ACCEL_SRC_CTRL_H,
                ICM45686_IREG_ACCEL_SRC_CTRL_L,
                reg_val);

            reg_val = ICM_ReadIReg(
                sensor,
                ICM45686_IREG_GYRO_SRC_CTRL_H,
                ICM45686_IREG_GYRO_SRC_CTRL_L);

            reg_val =
                (reg_val & ~ICM45686_GYRO_SRC_CTRL_MASK) |
                (ICM45686_GYRO_SRC_FIR_INTERP <<
                 ICM45686_GYRO_SRC_CTRL_SHIFT);

            ICM_WriteIRegBurst(
                sensor,
                ICM45686_IREG_GYRO_SRC_CTRL_H,
                ICM45686_IREG_GYRO_SRC_CTRL_L,
                reg_val);

            /*
             * ODR и FS задаются макросами config.h.
             * Никаких локальных ODR-кодов здесь не добавляется.
             */
            ICM_WriteReg(
                sensor,
                ICM45686_REG_ACCEL_CONFIG0,
                ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE);

            ICM_WriteReg(
                sensor,
                ICM45686_REG_GYRO_CONFIG0,
                ICM_GYRO_FS_VALUE | ICM_GYRO_ODR_VALUE);

            ICM_WriteReg(
                sensor,
                ICM45686_REG_PWR_MGMT0,
                ICM45686_PWR_GYRO_MODE_LN |
                ICM45686_PWR_ACCEL_MODE_LN);

            ICM_DelayUs(500U);

            reg_val = ICM_ReadIReg(
                sensor,
                ICM45686_IREG_SMC_CONTROL_0_H,
                ICM45686_IREG_SMC_CONTROL_0_L);

            reg_val |= ICM45686_TMST_EN;
            reg_val &= ~ICM45686_ACCEL_LP_CLK_SEL;

            ICM_WriteIRegBurst(
                sensor,
                ICM45686_IREG_SMC_CONTROL_0_H,
                ICM45686_IREG_SMC_CONTROL_0_L,
                reg_val);

            ICM_DelayUs(500U);

            /* Исходная настройка FIFO timestamp сохранена. */
            ICM_WriteReg(
                sensor,
                0x23U,
                (1U << 3) | (0U << 2));

            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG3,
                0x00U);

            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG2,
                ICM45686_FIFO_FLUSH);

            ICM_DelayUs(100U);

            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG0,
                ICM45686_FIFO_MODE_STREAM |
                ICM45686_FIFO_DEPTH_MAX);

            /* Watermark вычисляется из общего FIFO payload size. */
            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG1_0,
                (uint8_t)(ICM_FIFO_WATERMARK_BYTES & 0x00FFU));

            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG1_1,
                (uint8_t)((ICM_FIFO_WATERMARK_BYTES >> 8U) & 0x00FFU));

            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG4,
                ICM45686_FIFO_TMST_FSYNC_EN);

            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG3,
                ICM45686_FIFO_ACCEL_EN |
                ICM45686_FIFO_GYRO_EN |
                ICM45686_FIFO_HIRES_EN);

            ICM_WriteReg(
                sensor,
                ICM45686_REG_FIFO_CONFIG3,
                ICM45686_FIFO_ACCEL_EN |
                ICM45686_FIFO_GYRO_EN |
                ICM45686_FIFO_HIRES_EN |
                ICM45686_FIFO_IF_EN);

            ICM_DelayMs(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}

void ICM_StartBurstRead(void)
{
    uint8_t first1;
    uint8_t first5;
    uint8_t first4;

    uint8_t first2;
    uint8_t first3;
    uint8_t first6;

    g_icm_profile.tim6_total++;

    if ((g_bus_spi1.state != BUS_IDLE) ||
        (g_bus_spi5.state != BUS_IDLE) ||
        (g_bus_spi4.state != BUS_IDLE) ||
        (g_bus_spi2.state != BUS_IDLE) ||
        (g_bus_spi3.state != BUS_IDLE) ||
        (g_bus_spi6.state != BUS_IDLE))
    {
        g_tim6_skip_count++;
        g_icm_profile.frame_skip_count++;

        ICM_SetEvent(ICM_EVT_FRAME_SKIP);
        return;
    }

    g_icm_profile.acq_start_cyc = DWT->CYCCNT;

    g_dma_cycle_active = 1U;
    g_fifo_batch_ready = 0U;

    first1 = ICM_FindNextHealthy(&g_bus_spi1, 0U);
    first5 = ICM_FindNextHealthy(&g_bus_spi5, 0U);
    first4 = ICM_FindNextHealthy(&g_bus_spi4, 0U);

    first2 = ICM_FindNextHealthy(&g_bus_spi2, 0U);
    first3 = ICM_FindNextHealthy(&g_bus_spi3, 0U);
    first6 = ICM_FindNextHealthy(&g_bus_spi6, 0U);

    if (first1 < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(&g_bus_spi1, first1);
    }
    else
    {
        g_bus_spi1.state = BUS_COMPLETE;
        ICM_FinishBus(&g_bus_spi1);
    }

    if (first5 < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(&g_bus_spi5, first5);
    }
    else
    {
        g_bus_spi5.state = BUS_COMPLETE;
        ICM_FinishBus(&g_bus_spi5);
    }

    if (first4 < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(&g_bus_spi4, first4);
    }
    else
    {
        g_bus_spi4.state = BUS_COMPLETE;
        ICM_FinishBus(&g_bus_spi4);
    }

    if (first2 < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(&g_bus_spi2, first2);
    }
    else
    {
        g_bus_spi2.state = BUS_COMPLETE;
        ICM_FinishBus(&g_bus_spi2);
    }

    if (first3 < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(&g_bus_spi3, first3);
    }
    else
    {
        g_bus_spi3.state = BUS_COMPLETE;
        ICM_FinishBus(&g_bus_spi3);
    }

    if (first6 < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(&g_bus_spi6, first6);
    }
    else
    {
        g_bus_spi6.state = BUS_COMPLETE;
        ICM_FinishBus(&g_bus_spi6);
    }
}

void ICM_StartBurstRead_SPI1(void)
{
    ICM_StartBurstRead();
}

void ICM_DMA_RxComplete_SPI1(void)
{
    ICM_OnDmaRxComplete(&g_bus_spi1);
}

void ICM_DMA_RxComplete_SPI5(void)
{
    ICM_OnDmaRxComplete(&g_bus_spi5);
}

void ICM_DMA_RxComplete_SPI4(void)
{
    ICM_OnDmaRxComplete(&g_bus_spi4);
}

void ICM_DMA_RxComplete_SPI2(void)
{
    ICM_OnDmaRxComplete(&g_bus_spi2);
}

void ICM_DMA_RxComplete_SPI3(void)
{
    ICM_OnDmaRxComplete(&g_bus_spi3);
}

void ICM_DMA_RxComplete_SPI6(void)
{
    ICM_OnDmaRxComplete(&g_bus_spi6);
}

void ICM_DMA_Error_SPI1(void)
{
    g_dma_error_mask |= (1UL << 0U);

    g_bus_spi1.dma_error_count++;
    g_bus_spi1.state = BUS_ERROR;

    ICM_RecoverBus(&g_bus_spi1);
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

void ICM_DMA_Error_SPI2(void)
{
    g_dma_error_mask |= (1UL << 3U);

    g_bus_spi2.dma_error_count++;
    g_bus_spi2.state = BUS_ERROR;

    ICM_RecoverBus(&g_bus_spi2);
}

void ICM_DMA_Error_SPI3(void)
{
    g_dma_error_mask |= (1UL << 4U);

    g_bus_spi3.dma_error_count++;
    g_bus_spi3.state = BUS_ERROR;

    ICM_RecoverBus(&g_bus_spi3);
}

void ICM_DMA_Error_SPI6(void)
{
    g_dma_error_mask |= (1UL << 5U);

    g_bus_spi6.dma_error_count++;
    g_bus_spi6.state = BUS_ERROR;

    ICM_RecoverBus(&g_bus_spi6);
}

void ICM_SPI_Eot_SPI1(void)
{
    ICM_OnSpiEot(&g_bus_spi1);
}

void ICM_SPI_Eot_SPI5(void)
{
    ICM_OnSpiEot(&g_bus_spi5);
}

void ICM_SPI_Eot_SPI4(void)
{
    ICM_OnSpiEot(&g_bus_spi4);
}

void ICM_SPI_Eot_SPI2(void)
{
    ICM_OnSpiEot(&g_bus_spi2);
}

void ICM_SPI_Eot_SPI3(void)
{
    ICM_OnSpiEot(&g_bus_spi3);
}

void ICM_SPI_Eot_SPI6(void)
{
    ICM_OnSpiEot(&g_bus_spi6);
}

static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx)
{
    ICM_Sensor_t *sensor;
    const icm_dma_desc_t *desc;

    uint32_t spin;

    if (idx >= ICM_SENSORS_PER_BUS)
    {
        return;
    }

    sensor = &bus->sensors[idx];
    desc = &bus->dma_desc[idx];

    bus->state = BUS_START_SENSOR;
    bus->current_sensor_idx = idx;
    bus->eot_handled = 0U;

    if (bus->is_bdma)
    {
        LL_BDMA_DisableChannel(bus->bdma, bus->dma_stream_rx);
        LL_BDMA_DisableChannel(bus->bdma, bus->dma_stream_tx);

        spin = 2000U;

        while ((LL_BDMA_IsEnabledChannel(
                    bus->bdma, bus->dma_stream_rx) != 0U) &&
               (spin != 0U))
        {
            spin--;
        }

        spin = 2000U;

        while ((LL_BDMA_IsEnabledChannel(
                    bus->bdma, bus->dma_stream_tx) != 0U) &&
               (spin != 0U))
        {
            spin--;
        }

        if ((LL_BDMA_IsEnabledChannel(
                 bus->bdma, bus->dma_stream_rx) != 0U) ||
            (LL_BDMA_IsEnabledChannel(
                 bus->bdma, bus->dma_stream_tx) != 0U))
        {
            bus->state = BUS_ERROR;
            ICM_RecoverBus(bus);
            return;
        }
    }
    else
    {
        LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
        LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

        spin = 2000U;

        while ((LL_DMA_IsEnabledStream(
                    bus->dma, bus->dma_stream_rx) != 0U) &&
               (spin != 0U))
        {
            spin--;
        }

        spin = 2000U;

        while ((LL_DMA_IsEnabledStream(
                    bus->dma, bus->dma_stream_tx) != 0U) &&
               (spin != 0U))
        {
            spin--;
        }

        if ((LL_DMA_IsEnabledStream(
                 bus->dma, bus->dma_stream_rx) != 0U) ||
            (LL_DMA_IsEnabledStream(
                 bus->dma, bus->dma_stream_tx) != 0U))
        {
            bus->state = BUS_ERROR;
            ICM_RecoverBus(bus);
            return;
        }
    }

    ICM_ClearDmaFlags(bus);

    /*
     * TX-шаблон подготовлен при init.
     * Размеры и адреса берутся из precomputed descriptor.
     */
    if (bus->is_bdma)
    {
        LL_BDMA_SetPeriphAddress(
            bus->bdma,
            bus->dma_stream_rx,
            LL_SPI_DMA_GetRxRegAddr(bus->spi));

        LL_BDMA_SetMemoryAddress(
            bus->bdma,
            bus->dma_stream_rx,
            desc->rx_mem_addr);

        LL_BDMA_SetDataLength(
            bus->bdma,
            bus->dma_stream_rx,
            desc->length);

        LL_BDMA_SetPeriphAddress(
            bus->bdma,
            bus->dma_stream_tx,
            LL_SPI_DMA_GetTxRegAddr(bus->spi));

        LL_BDMA_SetMemoryAddress(
            bus->bdma,
            bus->dma_stream_tx,
            desc->tx_mem_addr);

        LL_BDMA_SetDataLength(
            bus->bdma,
            bus->dma_stream_tx,
            desc->length);

        LL_BDMA_EnableIT_TC(bus->bdma, bus->dma_stream_rx);
        LL_BDMA_EnableIT_TE(bus->bdma, bus->dma_stream_rx);
    }
    else
    {
        LL_DMA_SetPeriphAddress(
            bus->dma,
            bus->dma_stream_rx,
            LL_SPI_DMA_GetRxRegAddr(bus->spi));

        LL_DMA_SetMemoryAddress(
            bus->dma,
            bus->dma_stream_rx,
            desc->rx_mem_addr);

        LL_DMA_SetDataLength(
            bus->dma,
            bus->dma_stream_rx,
            desc->length);

        LL_DMA_SetPeriphAddress(
            bus->dma,
            bus->dma_stream_tx,
            LL_SPI_DMA_GetTxRegAddr(bus->spi));

        LL_DMA_SetMemoryAddress(
            bus->dma,
            bus->dma_stream_tx,
            desc->tx_mem_addr);

        LL_DMA_SetDataLength(
            bus->dma,
            bus->dma_stream_tx,
            desc->length);

        LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream_rx);
        LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream_rx);
    }

    LL_SPI_DisableIT_EOT(bus->spi);

    ICM_CS_Low(sensor);
    ICM_DelayUs(2U);

    ICM_SPI_EnsureDisabled(bus->spi);

    while (LL_SPI_IsActiveFlag_RXP(bus->spi) != 0U)
    {
        (void)LL_SPI_ReceiveData8(bus->spi);
    }

    WRITE_REG(bus->spi->IFCR, 0x0FF8U);

    LL_SPI_SetTransferSize(bus->spi, desc->length);
    LL_SPI_SetInternalSSLevel(bus->spi, LL_SPI_SS_LEVEL_HIGH);

    if (bus->is_bdma)
    {
        LL_BDMA_EnableChannel(bus->bdma, bus->dma_stream_rx);
        LL_BDMA_EnableChannel(bus->bdma, bus->dma_stream_tx);
    }
    else
    {
        LL_DMA_EnableStream(bus->dma, bus->dma_stream_rx);
        LL_DMA_EnableStream(bus->dma, bus->dma_stream_tx);
    }

    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    LL_SPI_Enable(bus->spi);

    __DSB();

    LL_SPI_StartMasterTransfer(bus->spi);

    bus->dma_start_cyc = DWT->CYCCNT;
    bus->state = BUS_DMA_ACTIVE;
}

static void ICM_AdvanceSensor(ICM_Bus_t *bus)
{
    uint8_t prev_idx;
    uint8_t next_idx;

    prev_idx = bus->current_sensor_idx;

    if (prev_idx >= ICM_SENSORS_PER_BUS)
    {
        bus->state = BUS_COMPLETE;
        ICM_FinishBus(bus);
        return;
    }

    ICM_CS_High(&bus->sensors[prev_idx]);

    bus->sensors[prev_idx].fault_count = 0U;

    LL_SPI_Disable(bus->spi);

    __DSB();

    bus->state = BUS_NEXT_SENSOR;

    next_idx = ICM_FindNextHealthy(
        bus,
        (uint8_t)(prev_idx + 1U));

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

static void ICM_OnDmaRxComplete(ICM_Bus_t *bus)
{
    if (bus->state != BUS_DMA_ACTIVE)
    {
        bus->dma_error_count++;
        bus->state = BUS_ERROR;

        ICM_RecoverBus(bus);
        return;
    }

    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);

    if (bus->is_bdma)
    {
        LL_BDMA_DisableChannel(bus->bdma, bus->dma_stream_rx);
        LL_BDMA_DisableChannel(bus->bdma, bus->dma_stream_tx);
    }
    else
    {
        LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
        LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);
    }

    /* Fast path: SPI EOT уже готов после DMA RX TC. */
    if (LL_SPI_IsActiveFlag_EOT(bus->spi) != 0U)
    {
        LL_SPI_ClearFlag_EOT(bus->spi);
        LL_SPI_ClearFlag_TXTF(bus->spi);

        WRITE_REG(bus->spi->IFCR, 0x0FF8U);

        ICM_AdvanceSensor(bus);
        return;
    }

    /* Fallback: ждём EOT через SPI IRQ. */
    bus->state = BUS_WAIT_EOT;

    LL_SPI_EnableIT_EOT(bus->spi);
}

static void ICM_OnSpiEot(ICM_Bus_t *bus)
{
    if (LL_SPI_IsActiveFlag_EOT(bus->spi) == 0U)
    {
        return;
    }

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

    ICM_AdvanceSensor(bus);
}

static void ICM_FinishBus(ICM_Bus_t *bus)
{
    bus->transfer_complete = 1U;

    ICM_TryCompleteBatch();
}

static void ICM_TryCompleteBatch(void)
{
    if ((g_bus_spi1.state == BUS_COMPLETE) &&
        (g_bus_spi5.state == BUS_COMPLETE) &&
        (g_bus_spi4.state == BUS_COMPLETE) &&
        (g_bus_spi2.state == BUS_COMPLETE) &&
        (g_bus_spi3.state == BUS_COMPLETE) &&
        (g_bus_spi6.state == BUS_COMPLETE))
    {
        g_icm_profile.acq_end_cyc = DWT->CYCCNT;

        g_icm_profile.acq_lat_last_us =
            ICM_DWT_ElapsedUs(g_icm_profile.acq_start_cyc);

        if (g_icm_profile.acq_lat_last_us >
            g_icm_profile.acq_lat_max_us)
        {
            g_icm_profile.acq_lat_max_us =
                g_icm_profile.acq_lat_last_us;
        }

        g_icm_profile.batch_count++;

        g_bus_spi1.state = BUS_IDLE;
        g_bus_spi5.state = BUS_IDLE;
        g_bus_spi4.state = BUS_IDLE;

        g_bus_spi2.state = BUS_IDLE;
        g_bus_spi3.state = BUS_IDLE;
        g_bus_spi6.state = BUS_IDLE;

        g_dma_cycle_active = 0U;
        g_fifo_batch_ready = 1U;

        ICM_SetEvent(ICM_EVT_BATCH_READY);
    }
}

static void ICM_MarkFault(ICM_Sensor_t *s)
{
    s->fault = 1U;

    g_sensor_fault_mask |= (1ULL << s->sensor_id);
}

static void ICM_RecoverBus(ICM_Bus_t *bus)
{
    uint8_t idx = bus->current_sensor_idx;
    uint8_t next_idx;

    /*
     * Диагностика необходимости полной FIFO-ресинхронизации.
     * Здесь не выполняется register/FIFO flush.
     */
    g_fifo_resync_required = 1U;

    if (idx < ICM_SENSORS_PER_BUS)
    {
        ICM_CS_High(&bus->sensors[idx]);

        bus->sensors[idx].fault_count++;

        if (bus->sensors[idx].fault_count >=
            ICM_SENSOR_MAX_FAULTS_LOCAL)
        {
            ICM_MarkFault(&bus->sensors[idx]);

            bus->sensors[idx].reint_countdown =
                ICM_REINTEGRATION_CYCLES_LOCAL;
        }
    }

    ICM_SPI_EnsureDisabled(bus->spi);

    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);

    if (bus->is_bdma)
    {
        LL_BDMA_DisableChannel(bus->bdma, bus->dma_stream_rx);
        LL_BDMA_DisableChannel(bus->bdma, bus->dma_stream_tx);
    }
    else
    {
        LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
        LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);
    }

    ICM_ClearDmaFlags(bus);

    LL_SPI_DisableIT_EOT(bus->spi);
    LL_SPI_ClearFlag_EOT(bus->spi);

    if (bus == &g_bus_spi1)
    {
        ICM_SetEvent(ICM_EVT_BUS0_FAULT);
    }
    else if (bus == &g_bus_spi5)
    {
        ICM_SetEvent(ICM_EVT_BUS1_FAULT);
    }
    else if (bus == &g_bus_spi4)
    {
        ICM_SetEvent(ICM_EVT_BUS2_FAULT);
    }
    else if (bus == &g_bus_spi2)
    {
        ICM_SetEvent(ICM_EVT_BUS3_FAULT);
    }
    else if (bus == &g_bus_spi3)
    {
        ICM_SetEvent(ICM_EVT_BUS4_FAULT);
    }
    else
    {
        ICM_SetEvent(ICM_EVT_BUS5_FAULT);
    }

    next_idx = ICM_FindNextHealthy(
        bus,
        (uint8_t)(idx + 1U));

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

/* TIM7 watchdog 1 кГц; output acquisition 400 Гц. */
static uint8_t ICM_BusTimedOut(const ICM_Bus_t *bus)
{
    if ((bus->state == BUS_DMA_ACTIVE) ||
        (bus->state == BUS_WAIT_EOT))
    {
        if (ICM_DWT_ElapsedUs(bus->dma_start_cyc) >
            ICM_DMA_TIMEOUT_US_LOCAL)
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

        if ((s->fault != 0U) &&
            (s->reint_countdown != 0U))
        {
            s->reint_countdown--;

            if (s->reint_countdown == 0U)
            {
                s->fault = 0U;
                s->fault_count = 0U;
            }
        }
    }
}

void ICM_WatchdogTick(void)
{
    ICM_Bus_t *buses[ICM_SPI_BUS_COUNT] =
    {
        &g_bus_spi1,
        &g_bus_spi5,
        &g_bus_spi4,
        &g_bus_spi2,
        &g_bus_spi3,
        &g_bus_spi6
    };

    uint8_t i;

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

/*
 * Atomic event bitmap.
 * ISR устанавливает биты, main loop атомарно потребляет их.
 */
static void ICM_SetEvent(uint32_t mask)
{
    uint32_t prev;

    if ((mask & (ICM_EVT_FRAME_SKIP | ICM_EVT_DMA_TIMEOUT)) != 0U)
    {
        g_fifo_resync_required = 1U;
    }

    do
    {
        prev = __LDREXW((uint32_t *)&g_icm_events);
    }
    while (__STREXW(
               prev | mask,
               (uint32_t *)&g_icm_events) != 0U);

    __DMB();
}

uint32_t ICM_ConsumeEvents(void)
{
    uint32_t events;

    do
    {
        events = __LDREXW((uint32_t *)&g_icm_events);
    }
    while (__STREXW(
               0U,
               (uint32_t *)&g_icm_events) != 0U);

    __DMB();

    return events;
}

static uint32_t ICM_DWT_ElapsedUs(uint32_t cyc_start)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t delta = now - cyc_start;

    return delta / (SystemCoreClock / 1000000UL);
}


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

static void ICM_ClearDmaFlags(const ICM_Bus_t *bus)
{
    if (bus->is_bdma)
    {
        LL_BDMA_ClearFlag_TC0(BDMA);
        LL_BDMA_ClearFlag_HT0(BDMA);
        LL_BDMA_ClearFlag_TE0(BDMA);

        LL_BDMA_ClearFlag_TC1(BDMA);
        LL_BDMA_ClearFlag_HT1(BDMA);
        LL_BDMA_ClearFlag_TE1(BDMA);

        return;
    }

    if (bus == &g_bus_spi1)
    {
        LL_DMA_ClearFlag_TC2(DMA1);
        LL_DMA_ClearFlag_HT2(DMA1);
        LL_DMA_ClearFlag_TE2(DMA1);
        LL_DMA_ClearFlag_DME2(DMA1);
        LL_DMA_ClearFlag_FE2(DMA1);

        LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_ClearFlag_HT3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1);
        LL_DMA_ClearFlag_DME3(DMA1);
        LL_DMA_ClearFlag_FE3(DMA1);
    }
    else if (bus == &g_bus_spi5)
    {
        LL_DMA_ClearFlag_TC2(DMA2);
        LL_DMA_ClearFlag_HT2(DMA2);
        LL_DMA_ClearFlag_TE2(DMA2);
        LL_DMA_ClearFlag_DME2(DMA2);
        LL_DMA_ClearFlag_FE2(DMA2);

        LL_DMA_ClearFlag_TC3(DMA2);
        LL_DMA_ClearFlag_HT3(DMA2);
        LL_DMA_ClearFlag_TE3(DMA2);
        LL_DMA_ClearFlag_DME3(DMA2);
        LL_DMA_ClearFlag_FE3(DMA2);
    }
    else if (bus == &g_bus_spi4)
    {
        LL_DMA_ClearFlag_TC0(DMA2);
        LL_DMA_ClearFlag_HT0(DMA2);
        LL_DMA_ClearFlag_TE0(DMA2);
        LL_DMA_ClearFlag_DME0(DMA2);
        LL_DMA_ClearFlag_FE0(DMA2);

        LL_DMA_ClearFlag_TC1(DMA2);
        LL_DMA_ClearFlag_HT1(DMA2);
        LL_DMA_ClearFlag_TE1(DMA2);
        LL_DMA_ClearFlag_DME1(DMA2);
        LL_DMA_ClearFlag_FE1(DMA2);
    }
    else if (bus == &g_bus_spi2)
    {
        LL_DMA_ClearFlag_TC4(DMA1);
        LL_DMA_ClearFlag_HT4(DMA1);
        LL_DMA_ClearFlag_TE4(DMA1);
        LL_DMA_ClearFlag_DME4(DMA1);
        LL_DMA_ClearFlag_FE4(DMA1);

        LL_DMA_ClearFlag_TC5(DMA1);
        LL_DMA_ClearFlag_HT5(DMA1);
        LL_DMA_ClearFlag_TE5(DMA1);
        LL_DMA_ClearFlag_DME5(DMA1);
        LL_DMA_ClearFlag_FE5(DMA1);
    }
    else if (bus == &g_bus_spi3)
    {
        LL_DMA_ClearFlag_TC6(DMA1);
        LL_DMA_ClearFlag_HT6(DMA1);
        LL_DMA_ClearFlag_TE6(DMA1);
        LL_DMA_ClearFlag_DME6(DMA1);
        LL_DMA_ClearFlag_FE6(DMA1);

        LL_DMA_ClearFlag_TC7(DMA1);
        LL_DMA_ClearFlag_HT7(DMA1);
        LL_DMA_ClearFlag_TE7(DMA1);
        LL_DMA_ClearFlag_DME7(DMA1);
        LL_DMA_ClearFlag_FE7(DMA1);
    }
}

static void ICM_SPI_EnsureDisabled(SPI_TypeDef *spi)
{
    uint32_t spin = 5000U;

    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);

        while ((LL_SPI_IsEnabled(spi) != 0U) &&
               (spin != 0U))
        {
            spin--;
        }
    }
}

static void ICM_SPI_WaitEOT(SPI_TypeDef *spi)
{
    uint32_t spin = 100000U;

    while ((LL_SPI_IsActiveFlag_EOT(spi) == 0U) &&
           (spin != 0U))
    {
        spin--;
    }

    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t n)
{
    uint32_t spin;

    while (n != 0U)
    {
        spin = 100000U;

        while ((LL_SPI_IsActiveFlag_RXP(spi) == 0U) &&
               (spin != 0U))
        {
            spin--;
        }

        (void)LL_SPI_ReceiveData8(spi);

        n--;
    }
}

static void ICM_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;

    uint32_t cycles =
        (SystemCoreClock / 1000000UL) * us;

    while ((DWT->CYCCNT - start) < cycles)
    {
        __NOP();
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

static void ICM_CS_Low(const ICM_Sensor_t *s)
{
    LL_GPIO_ResetOutputPin(s->cs_port, s->cs_pin);
}

static void ICM_CS_High(const ICM_Sensor_t *s)
{
    LL_GPIO_SetOutputPin(s->cs_port, s->cs_pin);
}
