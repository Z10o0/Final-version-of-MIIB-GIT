/**
 * @file    icm45686_spi.c
 * @brief   Реализация DMA-транспорта для 18 датчиков ICM-45686.
 *
 *          Архитектура каскадного DMA-опроса (3 шины):
 *
 *          TIM6 UPDATE IRQ (каждые 3.125 мс при 3200 Гц)
 *             │
 *             ▼
 *          ICM_StartBurstRead()  ← единая точка входа (обёртка над _SPI1)
 *             │ запускает DMA на шине SPI1, датчик[0]
 *             ▼
 *          DMA1_Stream2 TC ISR → ICM_DMA_RxComplete_SPI1()
 *             │ опросил датчик[n] — переключить CS, запустить датчик[n+1]
 *             │ если n==5 (все 6 готовы) → освободить SPI1, запустить SPI4
 *             ▼
 *          DMA2_Stream0 TC ISR → ICM_DMA_RxComplete_SPI4()
 *             │ аналогично SPI1, по завершению → запустить SPI5
 *             ▼
 *          DMA2_Stream2 TC ISR → ICM_DMA_RxComplete_SPI5()
 *             │ после последнего датчика SPI5 → g_fifo_batch_ready = 1
 *             ▼
 *          main-loop: ICM_ParseAllFIFO() → UART_SendBatch()
 *
 *          ВАЖНО: DMA1/DMA2 не имеют доступа к ITCM/DTCM.
 *          Буферы g_fifo_data размещены в SRAM D2 (AXI_SRAM).
 *          В linker-script по умолчанию глобальные переменные идут
 *          туда автоматически; если нет — добавить секцию .noinit в D2.
 *
 *          Блокирующая инициализация (ICM_WriteReg/ICM_ReadReg) использует
 *          простой polling SPI — допустимо только при старте системы.
 */

#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include "icm45686_config.h"
#include "main.h"
#include <string.h>

/* ================================================================
 * Внутренние прототипы
 * ================================================================ */
static void     BusBurstRead_Start(ICM_Bus_t *bus, uint8_t sensor_idx);
static void     BusBurstRead_Next(ICM_Bus_t *bus, uint8_t prev_idx);
static void     SPI_WaitTX(SPI_TypeDef *spi);
static void     SPI_WaitRX(SPI_TypeDef *spi);
static void     SPI_Enable(SPI_TypeDef *spi);
static void     SPI_Disable(SPI_TypeDef *spi);
static void     Delay_ms(uint32_t ms);

/* ================================================================
 * Сырые RX-буферы FIFO: [шина 0/1/2][датчик 0..5][байт]
 * Размещать в памяти доступной DMA (не DTCM/ITCM).
 * При STM32H7 в CubeIDE попадают в AXI SRAM — это нормально для DMA1/DMA2.
 * ================================================================ */
uint8_t g_fifo_data[3][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((aligned(4)));

/* ================================================================
 * Флаг готовности пачки данных (все 3 шины завершили опрос)
 * ================================================================ */
volatile uint8_t g_fifo_batch_ready = 0U;

/* ================================================================
 * Счётчик завершённых шин за текущий цикл
 * Инкрементируется в ISR каждой шины; при ==3 выставляем g_fifo_batch_ready
 * ================================================================ */
static volatile uint8_t g_buses_done_cnt = 0U;

/* ================================================================
 * Дескрипторы шин
 * Порядок датчиков: sensor_id — глобальный номер 0..17
 * SPI1 → датчики  0.. 5 (bus_idx = 0)
 * SPI4 → датчики  6..11 (bus_idx = 1)
 * SPI5 → датчики 12..17 (bus_idx = 2)
 * CS-пины соответствуют таблице соединений из Raspinovka-podkliucheniia-plat.docx
 * ================================================================ */
ICM_Bus_t g_bus_spi1 = {
    .spi           = SPI1,
    .dma           = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .current_sensor_idx = 0,
    .transfer_complete  = 0,
    .sensors = {
        /* IMU1 */ { SPI1, CS_IMU1_GPIO_Port, CS_IMU1_Pin, 0U },
        /* IMU2 */ { SPI1, CS_IMU2_GPIO_Port, CS_IMU2_Pin, 1U },
        /* IMU3 */ { SPI1, GPIOD,             CS_IMU3_Pin, 2U },
        /* IMU4 */ { SPI1, GPIOD,             CS_IMU4_Pin, 3U },
        /* IMU5 */ { SPI1, GPIOG,             CS_IMU5_Pin, 4U },
        /* IMU6 */ { SPI1, GPIOG,             CS_IMU6_Pin, 5U },
    }
};

ICM_Bus_t g_bus_spi4 = {
    .spi           = SPI4,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,
    .current_sensor_idx = 0,
    .transfer_complete  = 0,
    .sensors = {
        /* IMU7  */ { SPI4, GPIOE, CS27_Pin, 6U  },
        /* IMU8  */ { SPI4, GPIOB, CS25_Pin, 7U  },
        /* IMU9  */ { SPI4, GPIOB, CS26_Pin, 8U  },
        /* IMU10 */ { SPI4, GPIOE, CS33_Pin, 9U  },
        /* IMU11 */ { SPI4, GPIOE, CS34_Pin, 10U },
        /* IMU12 */ { SPI4, GPIOG, CS28_Pin, 11U },
    }
};

ICM_Bus_t g_bus_spi5 = {
    .spi           = SPI5,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .current_sensor_idx = 0,
    .transfer_complete  = 0,
    .sensors = {
        /* IMU13 */ { SPI5, GPIOF, CS31_Pin, 12U },
        /* IMU14 */ { SPI5, GPIOF, CS32_Pin, 13U },
        /* IMU15 */ { SPI5, GPIOE, CS29_Pin, 14U },
        /* IMU16 */ { SPI5, GPIOE, CS30_Pin, 15U },
        /* IMU17 */ { SPI5, GPIOE, CS23_Pin, 16U },
        /* IMU18 */ { SPI5, GPIOE, CS24_Pin, 17U },
    }
};

/* ================================================================
 * Вспомогательные inline-функции
 * ================================================================ */

/** @brief Ждать освобождения TXP (TX FIFO не полный) */
static inline void SPI_WaitTX(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
}

/** @brief Ждать RXP (RX FIFO не пустой) */
static inline void SPI_WaitRX(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
}

/** @brief Включить и запустить SPI */
static inline void SPI_Enable(SPI_TypeDef *spi)
{
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);   /* необходимо для STM32H7 SPI master */
}

/** @brief Дождаться конца транзакции и выключить SPI */
static inline void SPI_Disable(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
    LL_SPI_Disable(spi);
}

/** @brief Простая задержка в мс (используется только при инициализации) */
static void Delay_ms(uint32_t ms)
{
    /* TIM7 сконфигурирован как free-running 16-бит счётчик без прескалера.
     * При 550 МГц SYSCLK → APB1 = 275 МГц → TIM7 clock = 275 МГц
     * Одна единица TIM7 = 1/275 мкс ≈ 3.6 нс.
     * Для 1 мс нужно 275000 тиков.
     * Используем цикл по 16-бит переполнениям.
     */
    uint32_t ticks = ms * 275000UL;
    uint32_t t0 = LL_TIM_GetCounter(TIM7);
    while ((LL_TIM_GetCounter(TIM7) - t0) < ticks)
    {
        /* Простое ожидание — допустимо только при инициализации */
    }
}

/* ================================================================
 * ICM_BusesInit — инициализация структур шин
 * ================================================================ */
void ICM_BusesInit(void)
{
    uint8_t s;

    /* Взвести все CS в HIGH (неактивно) */
    for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
    {
        LL_GPIO_SetOutputPin(g_bus_spi1.sensors[s].cs_port,
                             g_bus_spi1.sensors[s].cs_pin);
        LL_GPIO_SetOutputPin(g_bus_spi4.sensors[s].cs_port,
                             g_bus_spi4.sensors[s].cs_pin);
        LL_GPIO_SetOutputPin(g_bus_spi5.sensors[s].cs_port,
                             g_bus_spi5.sensors[s].cs_pin);
    }

    /* Подготовить TX-буферы: первый байт — команда чтения FIFO-DATA */
    memset(g_bus_spi1.tx_buf, 0xFFU, sizeof(g_bus_spi1.tx_buf));
    memset(g_bus_spi4.tx_buf, 0xFFU, sizeof(g_bus_spi4.tx_buf));
    memset(g_bus_spi5.tx_buf, 0xFFU, sizeof(g_bus_spi5.tx_buf));

    /* Первый байт TX = адрес FIFO_DATA с битом READ */
    g_bus_spi1.tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_bus_spi4.tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_bus_spi5.tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    g_fifo_batch_ready = 0U;
    g_buses_done_cnt   = 0U;
}

/* ================================================================
 * ICM_WriteReg — блокирующая запись регистра (только для init)
 * ================================================================ */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t val)
{
    SPI_TypeDef *spi = sensor->spi;

    /* Установить размер транзакции = 2 байта */
    LL_SPI_SetTransferSize(spi, 2U);
    SPI_Enable(spi);

    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);  /* CS LOW */

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, reg & 0x7FU);  /* Бит7=0 → запись */

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, val);

    SPI_Disable(spi);
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);   /* CS HIGH */
}

/* ================================================================
 * ICM_ReadReg — блокирующее чтение регистра (только для init)
 * ================================================================ */
uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg)
{
    uint8_t dummy;
    uint8_t result;
    SPI_TypeDef *spi = sensor->spi;

    LL_SPI_SetTransferSize(spi, 2U);
    SPI_Enable(spi);

    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);  /* CS LOW */

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, reg | ICM45686_SPI_READ_BIT);   /* Бит7=1 → чтение */

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, 0xFFU);  /* Dummy-байт для получения ответа */

    SPI_Disable(spi);
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);   /* CS HIGH */

    /* Прочитать два байта из RX FIFO (адресный dummy + данные) */
    SPI_WaitRX(spi);
    dummy  = LL_SPI_ReceiveData8(spi);  /* Первый байт — echo адреса, игнорируем */
    (void)dummy;

    SPI_WaitRX(spi);
    result = LL_SPI_ReceiveData8(spi);  /* Второй байт — данные */

    return result;
}

/* ================================================================
 * ICM_InitAllSensors — инициализация всех 18 датчиков
 * ================================================================ */
uint8_t ICM_InitAllSensors(void)
{
    ICM_Bus_t   *buses[3]  = { &g_bus_spi1, &g_bus_spi4, &g_bus_spi5 };
    uint8_t      b, s;
    uint8_t      whoami;
    ICM_Sensor_t *sensor;
    uint8_t      pwr_val;
    uint8_t      fifo_cfg;

    for (b = 0U; b < 3U; b++)
    {
        for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
        {
            sensor = &buses[b]->sensors[s];

            /* 1. Сброс устройства (бит 1 = SOFT_RESET_CONFIG) */
            ICM_WriteReg(sensor, ICM45686_REG_DEVICE_CONFIG, 0x01U);
            Delay_ms(ICM45686_RESET_DELAY_MS);

            /* 2. Проверка WHO_AM_I */
            whoami = ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I);
            if (whoami != ICM_WHOAMI_EXPECTED)
            {
                /* Возвращаем глобальный номер датчика (1-based) при ошибке */
                return (uint8_t)(sensor->sensor_id + 1U);
            }

            /* 3. Выбор банка 0 */
            ICM_WriteReg(sensor, ICM45686_REG_BANK_SEL, 0x00U);

            /* 4. Конфигурация ODR и диапазона акселерометра */
            ICM_WriteReg(sensor, ICM45686_REG_ACCEL_CONFIG0,
                         (uint8_t)(ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE));

            /* 5. Конфигурация ODR и диапазона гироскопа */
            ICM_WriteReg(sensor, ICM45686_REG_GYRO_CONFIG0,
                         (uint8_t)(ICM_GYRO_FS_VALUE | ICM_GYRO_ODR_VALUE));

            /* 6. Настройка FIFO: режим STREAM, гироскоп + акселерометр + темп + timestamp */
            fifo_cfg = (uint8_t)(ICM45686_FIFO_MODE_STREAM |
                                 ICM45686_FIFO_SEL_GYRO    |
                                 ICM45686_FIFO_SEL_ACCEL   |
                                 ICM45686_FIFO_SEL_TEMP    |
                                 ICM45686_FIFO_SEL_TMST);
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG0, fifo_cfg);

            /* 7. Порог прерывания FIFO: не используем аппаратное прерывание,
             *    но настраиваем WM на случай расширения до 6400 Гц */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_0,
                         (uint8_t)((ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES) & 0xFFU));
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_1,
                         (uint8_t)(((ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES) >> 8U) & 0x0FU));

            /* 8. Включение гироскопа (LN режим) + акселерометра (LN режим) */
            pwr_val = (uint8_t)(ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN);
            ICM_WriteReg(sensor, ICM45686_REG_PWR_MGMT0, pwr_val);

            /* 9. Ожидание готовности датчика */
            Delay_ms(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return 0U;  /* Успех */
}

/* ================================================================
 * BusBurstRead_Start — запуск DMA-чтения для датчика sensor_idx на шине bus
 * ================================================================ */
static void BusBurstRead_Start(ICM_Bus_t *bus, uint8_t sensor_idx)
{
    ICM_Sensor_t *sensor = &bus->sensors[sensor_idx];
    DMA_TypeDef  *dma    = bus->dma;
    uint32_t      rx_st  = bus->dma_stream_rx;
    uint32_t      tx_st  = bus->dma_stream_tx;

    /* Выбрать индекс шины для адресации в g_fifo_data */
    uint8_t bus_idx;
    if (bus == &g_bus_spi1)      { bus_idx = 0U; }
    else if (bus == &g_bus_spi4) { bus_idx = 1U; }
    else                          { bus_idx = 2U; }

    /* Остановить поток на случай предыдущего незавершённого цикла */
    LL_DMA_DisableStream(dma, rx_st);
    LL_DMA_DisableStream(dma, tx_st);
    while (LL_DMA_IsEnabledStream(dma, rx_st) != 0U) {}
    while (LL_DMA_IsEnabledStream(dma, tx_st) != 0U) {}

    /* Сброс флагов */
    /* DMA1 */
    if (dma == DMA1)
    {
        if (rx_st == LL_DMA_STREAM_2) { LL_DMA_ClearFlag_TC2(DMA1); LL_DMA_ClearFlag_TE2(DMA1); }
        if (tx_st == LL_DMA_STREAM_3) { LL_DMA_ClearFlag_TC3(DMA1); LL_DMA_ClearFlag_TE3(DMA1); }
        if (rx_st == LL_DMA_STREAM_4) { LL_DMA_ClearFlag_TC4(DMA1); LL_DMA_ClearFlag_TE4(DMA1); }
        if (tx_st == LL_DMA_STREAM_5) { LL_DMA_ClearFlag_TC5(DMA1); LL_DMA_ClearFlag_TE5(DMA1); }
        if (rx_st == LL_DMA_STREAM_6) { LL_DMA_ClearFlag_TC6(DMA1); LL_DMA_ClearFlag_TE6(DMA1); }
        if (tx_st == LL_DMA_STREAM_7) { LL_DMA_ClearFlag_TC7(DMA1); LL_DMA_ClearFlag_TE7(DMA1); }
    }
    else  /* DMA2 */
    {
        if (rx_st == LL_DMA_STREAM_0) { LL_DMA_ClearFlag_TC0(DMA2); LL_DMA_ClearFlag_TE0(DMA2); }
        if (tx_st == LL_DMA_STREAM_1) { LL_DMA_ClearFlag_TC1(DMA2); LL_DMA_ClearFlag_TE1(DMA2); }
        if (rx_st == LL_DMA_STREAM_2) { LL_DMA_ClearFlag_TC2(DMA2); LL_DMA_ClearFlag_TE2(DMA2); }
        if (tx_st == LL_DMA_STREAM_3) { LL_DMA_ClearFlag_TC3(DMA2); LL_DMA_ClearFlag_TE3(DMA2); }
    }

    /* Настройка RX DMA: SPI RX → g_fifo_data[bus_idx][sensor_idx] */
    LL_DMA_SetMemoryAddress(dma, rx_st,
        (uint32_t)g_fifo_data[bus_idx][sensor_idx]);
    LL_DMA_SetPeriphAddress(dma, rx_st,
        LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetDataLength(dma, rx_st, ICM_FIFO_DMA_BUF_SIZE);

    /* Настройка TX DMA: tx_buf → SPI TX */
    LL_DMA_SetMemoryAddress(dma, tx_st,
        (uint32_t)bus->tx_buf);
    LL_DMA_SetPeriphAddress(dma, tx_st,
        LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetDataLength(dma, tx_st, ICM_FIFO_DMA_BUF_SIZE);

    /* Разрешить TC-прерывание только для RX (TX не нужно отслеживать) */
    LL_DMA_EnableIT_TC(dma, rx_st);

    /* CS LOW — активировать выбранный датчик */
    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* Настроить SPI на DMA-режим и задать размер транзакции */
    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);
    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    /* Запустить потоки: сначала RX, затем TX */
    LL_DMA_EnableStream(dma, rx_st);
    LL_DMA_EnableStream(dma, tx_st);

    /* Запустить SPI-транзакцию */
    SPI_Enable(bus->spi);
}

/* ================================================================
 * BusBurstRead_Next — обработка завершения датчика, переход к следующему
 * Вызывается из RX TC ISR каждой шины.
 * ================================================================ */
static void BusBurstRead_Next(ICM_Bus_t *bus, uint8_t prev_idx)
{
    /* CS HIGH — деактивировать завершённый датчик */
    LL_GPIO_SetOutputPin(bus->sensors[prev_idx].cs_port,
                         bus->sensors[prev_idx].cs_pin);

    /* Дождаться конца SPI транзакции и выключить */
    SPI_Disable(bus->spi);
    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);

    if (prev_idx < (ICM_SENSORS_PER_BUS - 1U))
    {
        /* Ещё есть датчики — запустить следующий */
        uint8_t next = prev_idx + 1U;
        bus->current_sensor_idx = next;
        BusBurstRead_Start(bus, next);
    }
    else
    {
        /* Все датчики шины опрошены */
        bus->transfer_complete = 1U;
        bus->current_sensor_idx = 0U;

        /* Инкрементируем счётчик завершённых шин */
        uint8_t cnt = ++g_buses_done_cnt;

        if (bus == &g_bus_spi1)
        {
            /* SPI1 готова → запускаем SPI4 */
            g_bus_spi4.current_sensor_idx = 0U;
            BusBurstRead_Start(&g_bus_spi4, 0U);
        }
        else if (bus == &g_bus_spi4)
        {
            /* SPI4 готова → запускаем SPI5 */
            g_bus_spi5.current_sensor_idx = 0U;
            BusBurstRead_Start(&g_bus_spi5, 0U);
        }
        else
        {
            /* SPI5 готова — все три шины завершены */
            if (cnt >= 3U)
            {
                g_buses_done_cnt   = 0U;  /* Сброс для следующего цикла */
                g_fifo_batch_ready = 1U;  /* Сигнал main-loop */
            }
        }
    }
}

/* ================================================================
 * ICM_StartBurstRead — точка входа из TIM6 ISR
 * (обёртка, совместимая с именами в stm32h7xx_it.c)
 * ================================================================ */
void ICM_StartBurstRead(void)
{
    /* Предохранитель: если предыдущий цикл не завершён — пропустить */
    if (g_fifo_batch_ready != 0U)
    {
        return;  /* main-loop ещё не обработал прошлые данные */
    }

    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;
    g_bus_spi1.current_sensor_idx = 0U;

    BusBurstRead_Start(&g_bus_spi1, 0U);
}

/* ================================================================
 * ICM_StartBurstRead_SPI1 — псевдоним для обратной совместимости
 * ================================================================ */
void ICM_StartBurstRead_SPI1(void)
{
    ICM_StartBurstRead();
}

/* ================================================================
 * ISR-обработчики DMA (вызываются из stm32h7xx_it.c)
 * ================================================================ */

void ICM_DMA_RxComplete_SPI1(void)
{
    BusBurstRead_Next(&g_bus_spi1, g_bus_spi1.current_sensor_idx);
}

void ICM_DMA_RxComplete_SPI4(void)
{
    BusBurstRead_Next(&g_bus_spi4, g_bus_spi4.current_sensor_idx);
}

void ICM_DMA_RxComplete_SPI5(void)
{
    BusBurstRead_Next(&g_bus_spi5, g_bus_spi5.current_sensor_idx);
}
