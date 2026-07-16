#include "icm45686_spi.h"

#include <string.h>

/* ========================================================================== */
/*                       Локальные прототипы                                  */
/* ========================================================================== */

static void ICM_DelayUs(uint32_t delay_us);
static void ICM_DelayMs(uint32_t delay_ms);

static void ICM_CS_Low(const ICM_Sensor_t *sensor);
static void ICM_CS_High(const ICM_Sensor_t *sensor);

static void ICM_SPI_WaitEOT(SPI_TypeDef *spi);
static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t bytes);
static void ICM_SPI_StopDma(SPI_TypeDef *spi);

static uint8_t ICM_BusIndex(const ICM_Bus_t *bus);
static uint8_t ICM_FindNextHealthy(const ICM_Bus_t *bus, uint8_t start_idx);

static void ICM_ClearDmaFlags(const ICM_Bus_t *bus);
static void ICM_InvalidateDmaBuffer(uint8_t *buffer, uint32_t size);
static void ICM_CleanDmaBuffer(uint8_t *buffer, uint32_t size);

static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t sensor_idx);
static void ICM_NextSensor(ICM_Bus_t *bus);
static void ICM_FinishBus(ICM_Bus_t *bus);
static void ICM_MarkSensorFault(ICM_Sensor_t *sensor);

/* ========================================================================== */
/*                       DMA-совместимые буферы                               */
/* ========================================================================== */

/*
 * AXI SRAM (D1): 0x24000000.
 * Передача DMA и кэш Cortex-M7 синхронизируются через Clean/Invalidate.
 */
uint8_t g_fifo_data[ICM_SPI_BUS_COUNT]
                   [ICM_SENSORS_PER_BUS]
                   [ICM_FIFO_DMA_BUF_SIZE]
                   __attribute__((section(".RAM_D1")))
                   __attribute__((aligned(32)));

static uint8_t g_tx_spi1[ICM_FIFO_DMA_BUF_SIZE]
                   __attribute__((section(".RAM_D1")))
                   __attribute__((aligned(32)));

static uint8_t g_tx_spi5[ICM_FIFO_DMA_BUF_SIZE]
                   __attribute__((section(".RAM_D1")))
                   __attribute__((aligned(32)));

static uint8_t g_tx_spi4[ICM_FIFO_DMA_BUF_SIZE]
                   __attribute__((section(".RAM_D1")))
                   __attribute__((aligned(32)));

volatile uint8_t g_fifo_batch_ready = 0U;
volatile uint8_t g_dma_cycle_active = 0U;
volatile uint32_t g_sensor_fault_mask = 0U;
volatile uint32_t g_dma_error_mask = 0U;

/* ========================================================================== */
/*                       Описание физических шин                              */
/* ========================================================================== */

/*
 * SPI1:
 * sensor 0..5:
 * PB12, PB13, PE8, PE9, PF13, PF14.
 */
ICM_Bus_t g_bus_spi1 =
{
    .spi = SPI1,
    .dma = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
	.tx_buf = g_tx_spi1,

    .sensors =
    {
        { SPI1, CS36_GPIO_Port, CS36_Pin, 0U, 0U },
        { SPI1, CS35_GPIO_Port, CS35_Pin, 1U, 0U },
        { SPI1, CS33_GPIO_Port, CS33_Pin, 2U, 0U },
        { SPI1, CS34_GPIO_Port, CS34_Pin, 3U, 0U },
        { SPI1, CS31_GPIO_Port, CS31_Pin, 4U, 0U },
        { SPI1, CS32_GPIO_Port, CS32_Pin, 5U, 0U }
    }
};

/*
 * SPI5:
 * sensor 6..11:
 * PE14, PE15, PE7, PG1, PB0, PB1.
 */
ICM_Bus_t g_bus_spi5 =
{
    .spi = SPI5,
    .dma = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
	.tx_buf = g_tx_spi5,

    .sensors =
    {
        { SPI5, CS29_GPIO_Port, CS29_Pin, 6U, 0U },
        { SPI5, CS30_GPIO_Port, CS30_Pin, 7U, 0U },
        { SPI5, CS27_GPIO_Port, CS27_Pin, 8U, 0U },
        { SPI5, CS28_GPIO_Port, CS28_Pin, 9U, 0U },
        { SPI5, CS25_GPIO_Port, CS25_Pin, 10U, 0U },
        { SPI5, CS26_GPIO_Port, CS26_Pin, 11U, 0U }
    }
};

/*
 * SPI4:
 * sensor 12..17:
 * PE10, PE11, PF15, PG0, PC4, PC5.
 */
ICM_Bus_t g_bus_spi4 =
{
    .spi = SPI4,
    .dma = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,
	.tx_buf = g_tx_spi4,

    .sensors =
    {
        { SPI4, CS23_GPIO_Port, CS23_Pin, 12U, 0U },
        { SPI4, CS24_GPIO_Port, CS24_Pin, 13U, 0U },
        { SPI4, CS22_GPIO_Port, CS22_Pin, 14U, 0U },
        { SPI4, CS21_GPIO_Port, CS21_Pin, 15U, 0U },
        { SPI4, CS19_GPIO_Port, CS19_Pin, 16U, 0U },
        { SPI4, CS20_GPIO_Port, CS20_Pin, 17U, 0U }
    }
};

/* ========================================================================== */
/*                       Инициализация                                        */
/* ========================================================================== */

void ICM_BusesInit(void)
{
    uint8_t i;

    memset(g_fifo_data, 0, sizeof(g_fifo_data));

    memset(g_tx_spi1, 0xFF, sizeof(g_tx_spi1));
    memset(g_tx_spi5, 0xFF, sizeof(g_tx_spi5));
    memset(g_tx_spi4, 0xFF, sizeof(g_tx_spi4));

    g_tx_spi1[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi5[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi4[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    g_bus_spi1.tx_buf = g_tx_spi1;
    g_bus_spi5.tx_buf = g_tx_spi5;
    g_bus_spi4.tx_buf = g_tx_spi4;

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
    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;

    g_fifo_batch_ready = 0U;
    g_dma_cycle_active = 0U;
    g_sensor_fault_mask = 0U;
    g_dma_error_mask = 0U;

    /* Теперь адреса в 0x2400xxxx — Clean безопасен, если D-Cache ON */
    ICM_CleanDmaBuffer(g_tx_spi1, sizeof(g_tx_spi1));
    ICM_CleanDmaBuffer(g_tx_spi5, sizeof(g_tx_spi5));
    ICM_CleanDmaBuffer(g_tx_spi4, sizeof(g_tx_spi4));
}

/* ========================================================================== */
/*                       Блокирующий SPI                                      */
/* ========================================================================== */

void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t value)
{
    SPI_TypeDef *spi = sensor->spi;

    /*
     * В начале транзакции SPI обязан быть выключен:
     * TSIZE на STM32H7 корректно программируется при SPE = 0.
     */
    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);

        while (LL_SPI_IsEnabled(spi) != 0U)
        {
        }
    }

    /* 2 байта: адрес регистра + записываемое значение. */
    LL_SPI_SetTransferSize(spi, 2U);

    /* Software NSS: внутренний NSS master должен быть HIGH. */
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);

    /* Выбираем конкретный датчик. */
    ICM_CS_Low(sensor);

    /* Включаем SPI: при CPOL=0 SCK переходит в idle LOW. */
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    /* Байта №1: адрес записи, MSB = 0. */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }
    LL_SPI_TransmitData8(spi, reg & 0x7FU);

    /* Байт №2: значение регистра. */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }
    LL_SPI_TransmitData8(spi, value);

    /* Все 2 байта физически ушли по SCK. */
    ICM_SPI_WaitEOT(spi);

    /*
     * SPI full-duplex: параллельно отправленным двум байтам
     * в RX FIFO поступили два ненужных байта.
     * Их необходимо извлечь, чтобы не получить OVR.
     */
    ICM_SPI_DrainRx(spi, 2U);

    /* Сначала завершение с датчиком, потом выключение SPI. */
    ICM_CS_High(sensor);

    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_Disable(spi);

    while (LL_SPI_IsEnabled(spi) != 0U)
    {
    }
}

/*
 * Чтение одного регистра ICM-45686 по SPI.
 *
 * Формат обмена:
 * MOSI: [ 1 | A6..A0 ] [ 0xFF ]
 * MISO: [    discard  ] [ data ]
 *
 * Для ICM-45686 CS удерживается LOW на протяжении всех 16 тактов.
 */
uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg)
{
    SPI_TypeDef *spi = sensor->spi;
    uint8_t discard;
    uint8_t result;

    /*
     * На STM32H7 поле TSIZE программируется только при SPE = 0.
     * Обычно SPI уже выключен после прошлой транзакции, но проверка
     * делает функцию независимой от предыдущего состояния шины.
     */
    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);

        while (LL_SPI_IsEnabled(spi) != 0U)
        {
        }
    }

    /*
     * SSM=1: внутренний NSS должен быть в неактивном HIGH-состоянии,
     * иначе master может получить MODF.
     * Это НЕ физический CS датчика.
     */
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);

    /* Очистить признак окончания предыдущей транзакции. */
    LL_SPI_ClearFlag_EOT(spi);

    /*
     * Два байта: read-address и dummy-байт для получения ответа.
     * TSIZE задаётся до установки SPE.
     */
    LL_SPI_SetTransferSize(spi, 2U);

    /* Выбор ровно одного ICM-45686. */
    ICM_CS_Low(sensor);

    /*
     * CPOL=0: после включения SPI SCK удерживается в LOW до CSTART.
     * CPHA=0: ICM-45686 считывает MOSI на rising edge.
     */
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    /* Первый байт: bit7=1 означает операцию чтения. */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, (reg & 0x7FU) | ICM45686_SPI_READ_BIT);

    /*
     * Второй байт не имеет значения для датчика: он нужен только,
     * чтобы выдать дополнительные 8 импульсов SCK и принять data.
     */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U)
    {
    }

    LL_SPI_TransmitData8(spi, 0xFFU);

    /* Ожидание физического окончания всех 16 тактов. */
    ICM_SPI_WaitEOT(spi);

    /*
     * После отправки каждого байта SPI принял один байт:
     * 1-й — неинформативен, 2-й — содержимое запрошенного регистра.
     */
    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U)
    {
    }

    discard = LL_SPI_ReceiveData8(spi);

    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U)
    {
    }

    result = LL_SPI_ReceiveData8(spi);

    (void)discard;

    /* Закрытие транзакции строго после считывания RX FIFO. */
    ICM_CS_High(sensor);

    LL_SPI_ClearFlag_EOT(spi);

    LL_SPI_Disable(spi);

    while (LL_SPI_IsEnabled(spi) != 0U)
    {
    }

    return result;
}

/*
 * IREG write обязательно является одной SPI burst-write операцией:
 * [0x7C][ADDR_H][ADDR_L][DATA].
 */
void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t addr_h,
                   uint8_t addr_l,
                   uint8_t value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_CS_Low(sensor);

    LL_SPI_SetTransferSize(spi, 4U);
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, ICM45686_REG_IREG_ADDR_15_8);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, addr_h);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, addr_l);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 4U);

    ICM_CS_High(sensor);
    LL_SPI_Disable(spi);

    ICM_DelayUs(ICM45686_IREG_DELAY_US);
}

uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t addr_h,
                     uint8_t addr_l)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0, addr_l);

    ICM_DelayUs(ICM45686_IREG_DELAY_US);

    return ICM_ReadReg(sensor, ICM45686_REG_IREG_DATA);
}

/* ========================================================================== */
/*                       Инициализация ICM-45686                              */
/* ========================================================================== */

uint32_t ICM_InitAllSensors(void)
{
    ICM_Bus_t *const buses[ICM_SPI_BUS_COUNT] =
    {
        &g_bus_spi1,
        &g_bus_spi5,
        &g_bus_spi4
    };

    uint8_t bus_idx;
    uint8_t sensor_idx;

    g_sensor_fault_mask = 0U;

    for (bus_idx = 0U; bus_idx < ICM_SPI_BUS_COUNT; bus_idx++)
    {
        for (sensor_idx = 0U;
             sensor_idx < ICM_SENSORS_PER_BUS;
             sensor_idx++)
        {
            ICM_Sensor_t *sensor = &buses[bus_idx]->sensors[sensor_idx];
            uint8_t status;
            uint32_t timeout;



            /* 2. WHO_AM_I */
            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I)
                != ICM45686_WHO_AM_I_VALUE)
            {
                ICM_MarkSensorFault(sensor);
                continue;
            }


            /* 1. Программный reset */
            ICM_WriteReg(sensor,
                         ICM45686_REG_REG_MISC2,
                         ICM45686_SOFT_RESET);

            ICM_DelayUs(ICM45686_RESET_DELAY_US);

            /*
             * 3. Pin 9 переводится в CLKIN.
             * Общий внешний сигнал 32.768 кГц уже должен присутствовать.
             */
            ICM_WriteIReg(sensor,
                           ICM45686_IREG_TOP1_ADDR_H,
                           ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L,
                           ICM45686_CLKIN_ENABLE_VAL);

            /*
             * 4. Включаем статус PLL ready и выбираем внешний clock source.
             */
            ICM_WriteReg(sensor,
                         ICM45686_REG_INT1_CONFIG1,
                         ICM45686_INT1_PLL_RDY_EN);

            ICM_WriteReg(sensor,
                         ICM45686_REG_REG_MISC1,
                         ICM45686_OSC_ID_OVRD_EXT_CLK);

            /*
             * 5. Ожидание захвата PLL.
             * INT1_STATUS1 read-clear, поэтому читаем в polling цикле.
             */
            timeout = ICM45686_PLL_TIMEOUT_US;

            do
            {
                status = ICM_ReadReg(sensor, ICM45686_REG_INT1_STATUS1);

                if ((status & ICM45686_INT1_STATUS_PLL_RDY) != 0U)
                {
                    break;
                }

                ICM_DelayUs(10U);
                timeout -= 10U;
            }
            while (timeout != 0U);

            if ((status & ICM45686_INT1_STATUS_PLL_RDY) == 0U)
            {
                ICM_MarkSensorFault(sensor);
                continue;
            }

            /*
             * 6. Timestamp core и выбор clock.
             * Должно быть выполнено до PWR_MGMT0.
             */
            ICM_WriteIReg(sensor,
                           ICM45686_IREG_TOP1_ADDR_H,
                           ICM45686_IREG_SMC_CONTROL_0_L,
                           ICM45686_SMC_CONTROL_0_VALUE);

            /* 7. RTC_MODE — работа ODR от внешней clock domain */
            ICM_WriteReg(sensor,
                         ICM45686_REG_RTC_CONFIG,
                         ICM45686_RTC_MODE_EN);

            /* 8. FS и ODR */
            ICM_WriteReg(sensor,
                         ICM45686_REG_ACCEL_CONFIG0,
                         ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE);

            ICM_WriteReg(sensor,
                         ICM45686_REG_GYRO_CONFIG0,
                         ICM_GYRO_FS_VALUE | ICM_GYRO_ODR_VALUE);

            /*
             * 9. FIFO конфигурируется строго до активации interface.
             * FIFO depth: 2 KiB; stream mode.
             */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG0,
                         ICM45686_FIFO_MODE_STREAM |
                         ICM45686_FIFO_DEPTH_2K);

            /*
             * Watermark: строго сначала low byte, затем high byte.
             */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG10,
                         (uint8_t)(ICM_FIFO_WATERMARK_BYTES & 0xFFU));

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG11,
                         (uint8_t)((ICM_FIFO_WATERMARK_BYTES >> 8) & 0xFFU));

            /*
             * Timestamp в FIFO.
             * FIFO compression выключена: необходим фиксированный размер пакета.
             */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG4,
                         ICM45686_FIFO_TMST_FSYNC_EN);

            /*
             * 10. Сначала channels, затем FIFO interface.
             */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN |
                         ICM45686_FIFO_GYRO_EN);

            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN |
                         ICM45686_FIFO_GYRO_EN |
                         ICM45686_FIFO_IF_EN);

            /* 11. Очистка FIFO перед стартом измерений */
            ICM_WriteReg(sensor,
                         ICM45686_REG_FIFO_CONFIG2,
                         ICM45686_FIFO_FLUSH);

            /* 12. Включение gyro и accel в Low-Noise */
            ICM_WriteReg(sensor,
                         ICM45686_REG_PWR_MGMT0,
                         ICM45686_PWR_GYRO_MODE_LN |
                         ICM45686_PWR_ACCEL_MODE_LN);

            ICM_DelayMs(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}

/* ========================================================================== */
/*                       DMA FIFO cascade                                     */
/* ========================================================================== */

void ICM_StartBurstRead(void)
{
    uint8_t first;

    /*
     * Новая пачка не запускается, если main-loop ещё не забрал предыдущую.
     */
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
        ICM_FinishBus(&g_bus_spi1);
    }
}

void ICM_StartBurstRead_SPI1(void)
{
    ICM_StartBurstRead();
}

void ICM_DMA_RxComplete_SPI1(void)
{
    ICM_NextSensor(&g_bus_spi1);
}

void ICM_DMA_RxComplete_SPI5(void)
{
    ICM_NextSensor(&g_bus_spi5);
}

void ICM_DMA_RxComplete_SPI4(void)
{
    ICM_NextSensor(&g_bus_spi4);
}

void ICM_DMA_Error_SPI1(void)
{
    g_dma_error_mask |= (1UL << 0);
    ICM_NextSensor(&g_bus_spi1);
}

void ICM_DMA_Error_SPI5(void)
{
    g_dma_error_mask |= (1UL << 1);
    ICM_NextSensor(&g_bus_spi5);
}

void ICM_DMA_Error_SPI4(void)
{
    g_dma_error_mask |= (1UL << 2);
    ICM_NextSensor(&g_bus_spi4);
}

static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t sensor_idx)
{
    ICM_Sensor_t *sensor = &bus->sensors[sensor_idx];
    uint8_t bus_idx = ICM_BusIndex(bus);
    uint8_t *rx_buffer = g_fifo_data[bus_idx][sensor_idx];

    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_rx) != 0U) {}
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_tx) != 0U) {}

    ICM_ClearDmaFlags(bus);

    /*
     * RX buffer invalidation до DMA исключает write-back старой cache line
     * после начала записи DMA.
     */
    ICM_InvalidateDmaBuffer(rx_buffer, ICM_FIFO_DMA_BUF_SIZE);
    ICM_CleanDmaBuffer(bus->tx_buf, ICM_FIFO_DMA_BUF_SIZE);

    LL_DMA_SetPeriphAddress(bus->dma,
                            bus->dma_stream_rx,
                            LL_SPI_DMA_GetRxRegAddr(bus->spi));

    LL_DMA_SetMemoryAddress(bus->dma,
                            bus->dma_stream_rx,
                            (uint32_t)rx_buffer);

    LL_DMA_SetDataLength(bus->dma,
                         bus->dma_stream_rx,
                         ICM_FIFO_DMA_BUF_SIZE);

    LL_DMA_SetPeriphAddress(bus->dma,
                            bus->dma_stream_tx,
                            LL_SPI_DMA_GetTxRegAddr(bus->spi));

    LL_DMA_SetMemoryAddress(bus->dma,
                            bus->dma_stream_tx,
                            (uint32_t)bus->tx_buf);

    LL_DMA_SetDataLength(bus->dma,
                         bus->dma_stream_tx,
                         ICM_FIFO_DMA_BUF_SIZE);

    LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream_rx);

    bus->current_sensor_idx = sensor_idx;

    ICM_CS_Low(sensor);

    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);

    LL_DMA_EnableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_tx);

    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    LL_SPI_Enable(bus->spi);
    LL_SPI_StartMasterTransfer(bus->spi);
}

static void ICM_NextSensor(ICM_Bus_t *bus)
{
    uint8_t previous = bus->current_sensor_idx;
    uint8_t next;
    uint8_t bus_idx = ICM_BusIndex(bus);

    ICM_SPI_StopDma(bus->spi);

    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    ICM_CS_High(&bus->sensors[previous]);

    /*
     * После DMA CPU должен отбросить кэшированные строки и читать SRAM.
     */
    ICM_InvalidateDmaBuffer(g_fifo_data[bus_idx][previous],
                            ICM_FIFO_DMA_BUF_SIZE);

    next = ICM_FindNextHealthy(bus, previous + 1U);

    if (next < ICM_SENSORS_PER_BUS)
    {
        ICM_StartBusRead(bus, next);
    }
    else
    {
        ICM_FinishBus(bus);
    }
}

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

    g_dma_cycle_active = 0U;
    g_fifo_batch_ready = 1U;
}

/* ========================================================================== */
/*                       Служебные функции                                    */
/* ========================================================================== */

static void ICM_MarkSensorFault(ICM_Sensor_t *sensor)
{
    sensor->fault = 1U;
    g_sensor_fault_mask |= (1UL << sensor->sensor_id);
}

static void ICM_CS_Low(const ICM_Sensor_t *sensor)
{
    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);
}

static void ICM_CS_High(const ICM_Sensor_t *sensor)
{
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);
}

static void ICM_SPI_WaitEOT(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}

    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t bytes)
{
    while (bytes != 0U)
    {
        while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}

        (void)LL_SPI_ReceiveData8(spi);
        bytes--;
    }
}

static void ICM_SPI_StopDma(SPI_TypeDef *spi)
{
    ICM_SPI_WaitEOT(spi);

    LL_SPI_DisableDMAReq_RX(spi);
    LL_SPI_DisableDMAReq_TX(spi);

    LL_SPI_Disable(spi);
}

static uint8_t ICM_BusIndex(const ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1)
    {
        return 0U;
    }

    if (bus == &g_bus_spi5)
    {
        return 1U;
    }

    return 2U;
}

static uint8_t ICM_FindNextHealthy(const ICM_Bus_t *bus, uint8_t start_idx)
{
    uint8_t i;

    for (i = start_idx; i < ICM_SENSORS_PER_BUS; i++)
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
    if (bus->dma == DMA1)
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
    else
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
}

static void ICM_CleanDmaBuffer(uint8_t *buffer, uint32_t size)
{
    uint32_t address = ((uint32_t)buffer) & ~31UL;
    uint32_t length = (size + 31U + ((uint32_t)buffer - address)) & ~31UL;

    SCB_CleanDCache_by_Addr((uint32_t *)address, (int32_t)length);
    __DSB();
    __ISB();
}

static void ICM_InvalidateDmaBuffer(uint8_t *buffer, uint32_t size)
{
    uint32_t address = ((uint32_t)buffer) & ~31UL;
    uint32_t length = (size + 31U + ((uint32_t)buffer - address)) & ~31UL;

    SCB_InvalidateDCache_by_Addr((uint32_t *)address, (int32_t)length);
    __DSB();
    __ISB();
}

static void ICM_DelayUs(uint32_t delay_us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * delay_us;

    while (cycles > 3U)
    {
        __NOP();
        cycles -= 3U;
    }
}

static void ICM_DelayMs(uint32_t delay_ms)
{
    while (delay_ms != 0U)
    {
        ICM_DelayUs(1000U);
        delay_ms--;
    }
}
