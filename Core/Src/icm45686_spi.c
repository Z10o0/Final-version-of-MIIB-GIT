/**
 * @file    icm45686_spi.c
 * @brief   Реализация DMA-транспорта для 18 датчиков ICM-45686.
 *
 *          Архитектура каскадного DMA-опроса (3 шины):
 *
 *          TIM6 UPDATE IRQ (каждые 3.125 мс при 3200 Гц)
 *             │
 *             ▼
 *          ICM_StartBurstRead()  ← единая точка входа
 *             │ запускает первый ИСПРАВНЫЙ датчик SPI1
 *             ▼
 *          DMA1_Stream2 TC ISR → ICM_DMA_RxComplete_SPI1()
 *             │ датчик опрошен → следующий исправный датчик шины
 *             │ если шина завершена → запустить SPI4
 *             ▼
 *          DMA2_Stream0 TC ISR → ICM_DMA_RxComplete_SPI4()
 *             │ аналогично → запустить SPI5
 *             ▼
 *          DMA2_Stream2 TC ISR → ICM_DMA_RxComplete_SPI5()
 *             │ после последнего → g_fifo_batch_ready = 1
 *             ▼
 *          main-loop: ICM_ParseAllFIFO() → UART_SendBatch()
 *
 *          Неисправные датчики (fault=1):
 *          - В DMA-каскаде полностью пропускаются (CS не трогается)
 *          - В g_fifo_data их буферы остаются нулями от memset при старте
 *          - ICM_ParseAllFIFO() записывает нули в g_sensor_batches для них
 *
 *          ВАЖНО: DMA1/DMA2 не имеют доступа к ITCM/DTCM.
 *          Буферы g_fifo_data размещены в SRAM D2 (AXI_SRAM).
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
static void     BusFinalize(ICM_Bus_t *bus);
static void     SPI_WaitTX(SPI_TypeDef *spi);
static void     SPI_WaitRX(SPI_TypeDef *spi);
static void     SPI_Enable(SPI_TypeDef *spi);
static void     SPI_Disable(SPI_TypeDef *spi);
static void     Delay_ms(uint32_t ms);
static uint8_t  Bus_FindNextOK(ICM_Bus_t *bus, uint8_t start_idx);

/* ================================================================
 * Сырые RX-буферы FIFO: [шина][датчик][байт]
 * Размещать в памяти доступной DMA (не DTCM/ITCM).
 * ================================================================ */
uint8_t g_fifo_data[3][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((aligned(4)));

/* ================================================================
 * Флаг готовности пачки данных (все 3 шины завершили опрос)
 * ================================================================ */
volatile uint8_t g_fifo_batch_ready = 0U;

/* ================================================================
 * Маска неисправных датчиков: бит N установлен если датчик N не ответил
 * на WHO_AM_I при инициализации. Устанавливается один раз при старте.
 * ================================================================ */
volatile uint32_t g_sensor_fault_mask = 0U;

/* ================================================================
 * Счётчик завершённых шин за текущий цикл
 * ================================================================ */
static volatile uint8_t g_buses_done_cnt = 0U;

/* ================================================================
 * Дескрипторы шин
 * SPI1 → датчики  0.. 5 (bus_idx = 0)
 * SPI4 → датчики  6..11 (bus_idx = 1)
 * SPI5 → датчики 12..17 (bus_idx = 2)
 * ================================================================ */

/* SPI1: датчики CS13..CS18 → PF13,PF14,PE8,PE9,PB13,PB12 */
ICM_Bus_t g_bus_spi1 = {
    .spi = SPI1, .dma = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .sensors = {
        /* CS13 */ { SPI1, CS31_GPIO_Port, CS31_Pin, 0U, 0U },  /* PF13 */
        /* CS14 */ { SPI1, CS32_GPIO_Port, CS32_Pin, 1U, 0U },  /* PF14 */
        /* CS15 */ { SPI1, CS33_GPIO_Port, CS33_Pin, 2U, 0U },  /* PE8  */
        /* CS16 */ { SPI1, CS34_GPIO_Port, CS34_Pin, 3U, 0U },  /* PE9  */
        /* CS17 */ { SPI1, CS35_GPIO_Port, CS35_Pin, 4U, 0U },  /* PB13 */
        /* CS18 */ { SPI1, CS36_GPIO_Port, CS36_Pin, 5U, 0U },  /* PB12 */
    }
};


/* SPI4: датчики CS1..CS6 → PC4,PC5,PG0,PF15,PE10,PE11 */
ICM_Bus_t g_bus_spi4 = {
    .spi = SPI4, .dma = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,
    .sensors = {
        /* CS1  */ { SPI4, CS19_GPIO_Port, CS19_Pin, 6U,  0U }, /* PC4  */
        /* CS2  */ { SPI4, CS20_GPIO_Port, CS20_Pin, 7U,  0U }, /* PC5  */
        /* CS3  */ { SPI4, CS21_GPIO_Port, CS21_Pin, 8U,  0U }, /* PG0  */
        /* CS4  */ { SPI4, CS22_GPIO_Port, CS22_Pin, 9U,  0U }, /* PF15 */
        /* CS5  */ { SPI4, CS23_GPIO_Port, CS23_Pin, 10U, 0U }, /* PE10 */
        /* CS6  */ { SPI4, CS24_GPIO_Port, CS24_Pin, 11U, 0U }, /* PE11 */
    }
};


/* SPI5: датчики CS7..CS12 → PB0,PB1,PE7,PG1,PE14,PE15 */
ICM_Bus_t g_bus_spi5 = {
    .spi = SPI5, .dma = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .sensors = {
        /* CS7  */ { SPI5, CS25_GPIO_Port, CS25_Pin, 12U, 0U }, /* PB0  */
        /* CS8  */ { SPI5, CS26_GPIO_Port, CS26_Pin, 13U, 0U }, /* PB1  */
        /* CS9  */ { SPI5, CS27_GPIO_Port, CS27_Pin, 14U, 0U }, /* PE7  */
        /* CS10 */ { SPI5, CS28_GPIO_Port, CS28_Pin, 15U, 0U }, /* PG1  */
        /* CS11 */ { SPI5, CS29_GPIO_Port, CS29_Pin, 16U, 0U }, /* PE14 */
        /* CS12 */ { SPI5, CS30_GPIO_Port, CS30_Pin, 17U, 0U }, /* PE15 */
    }
};

/* ================================================================
 * Bus_FindNextOK — найти индекс следующего ИСПРАВНОГО датчика на шине
 * Возвращает ICM_SENSORS_PER_BUS если исправных больше нет.
 * ================================================================ */
static uint8_t Bus_FindNextOK(ICM_Bus_t *bus, uint8_t start_idx)
{
    uint8_t i;
    for (i = start_idx; i < ICM_SENSORS_PER_BUS; i++)
    {
        if (bus->sensors[i].fault == 0U)
        {
            return i;
        }
    }
    return ICM_SENSORS_PER_BUS;  /* Нет исправных датчиков */
}

/* ================================================================
 * Вспомогательные inline-функции
 * ================================================================ */

static inline void SPI_WaitTX(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
}

static inline void SPI_WaitRX(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
}

static inline void SPI_Enable(SPI_TypeDef *spi)
{
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);
}

static inline void SPI_Disable(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
    LL_SPI_Disable(spi);
}

static void Delay_ms(uint32_t ms)
{
    /* TIM7: free-running, APB1 = 275 МГц → 275000 тиков/мс */
    uint32_t ticks = ms * 275000UL;
    uint32_t t0    = LL_TIM_GetCounter(TIM7);
    while ((LL_TIM_GetCounter(TIM7) - t0) < ticks) {}
}

/* ================================================================
 * ICM_BusesInit — инициализация структур шин и сброс fault-флагов
 * ================================================================ */
void ICM_BusesInit(void)
{
    uint8_t s;

    /* Взвести все CS в HIGH (неактивно) и сбросить fault */
    for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
    {
        LL_GPIO_SetOutputPin(g_bus_spi1.sensors[s].cs_port, g_bus_spi1.sensors[s].cs_pin);
        LL_GPIO_SetOutputPin(g_bus_spi4.sensors[s].cs_port, g_bus_spi4.sensors[s].cs_pin);
        LL_GPIO_SetOutputPin(g_bus_spi5.sensors[s].cs_port, g_bus_spi5.sensors[s].cs_pin);

        g_bus_spi1.sensors[s].fault = 0U;
        g_bus_spi4.sensors[s].fault = 0U;
        g_bus_spi5.sensors[s].fault = 0U;
    }

    /* Подготовить TX-буферы: первый байт — команда чтения FIFO, остальные = 0xFF dummy */
    memset(g_bus_spi1.tx_buf, 0xFFU, sizeof(g_bus_spi1.tx_buf));
    memset(g_bus_spi4.tx_buf, 0xFFU, sizeof(g_bus_spi4.tx_buf));
    memset(g_bus_spi5.tx_buf, 0xFFU, sizeof(g_bus_spi5.tx_buf));

    g_bus_spi1.tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_bus_spi4.tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_bus_spi5.tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    /* Занулить FIFO-буферы: для fault-датчиков данные останутся нулями */
    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));

    g_fifo_batch_ready  = 0U;
    g_buses_done_cnt    = 0U;
    g_sensor_fault_mask = 0U;
}

/* ================================================================
 * ICM_WriteReg — блокирующая запись регистра (только для init)
 * ================================================================ */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t val)
{
    SPI_TypeDef *spi = sensor->spi;

    LL_SPI_SetTransferSize(spi, 2U);
    SPI_Enable(spi);

    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, reg & 0x7FU);

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, val);

    SPI_Disable(spi);
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);
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

    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, reg | ICM45686_SPI_READ_BIT);

    SPI_WaitTX(spi);
    LL_SPI_TransmitData8(spi, 0xFFU);

    SPI_Disable(spi);
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);

    SPI_WaitRX(spi);
    dummy  = LL_SPI_ReceiveData8(spi);
    (void)dummy;

    SPI_WaitRX(spi);
    result = LL_SPI_ReceiveData8(spi);

    return result;
}

/* ================================================================
 * ICM_InitAllSensors — инициализация всех 18 датчиков.
 *
 * КЛЮЧЕВОЕ ПОВЕДЕНИЕ:
 * При ошибке WHO_AM_I датчик помечается sensor->fault = 1,
 * бит его ID устанавливается в g_sensor_fault_mask,
 * инициализация ПРОДОЛЖАЕТСЯ на следующем датчике.
 * Возвращает маску неисправных датчиков (0 = все исправны).
 * ================================================================ */
uint32_t ICM_InitAllSensors(void)
{
    ICM_Bus_t   *buses[3] = { &g_bus_spi1, &g_bus_spi4, &g_bus_spi5 };
    uint8_t      b, s;
    uint8_t      whoami;
    ICM_Sensor_t *sensor;
    uint8_t      pwr_val;
    uint8_t      fifo_cfg;

    g_sensor_fault_mask = 0U;

    for (b = 0U; b < 3U; b++)
    {
        for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
        {
            sensor = &buses[b]->sensors[s];
            sensor->fault = 0U;  /* Начинаем как исправный */

            /* 1. Программный сброс */
            ICM_WriteReg(sensor, ICM45686_REG_DEVICE_CONFIG, 0x01U);
            Delay_ms(ICM45686_RESET_DELAY_MS);

            /* 2. Проверка WHO_AM_I */
            whoami = ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I);
            if (whoami != ICM_WHOAMI_EXPECTED)
            {
                /* Датчик не ответил — помечаем fault, НЕ останавливаемся */
                sensor->fault = 1U;
                g_sensor_fault_mask |= (1UL << sensor->sensor_id);

                /* Переходим к следующему датчику */
                continue;
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

            /* 7. Порог FIFO watermark (задел под 6400 Гц) */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_0,
                         (uint8_t)((ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES) & 0xFFU));
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_1,
                         (uint8_t)(((ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES) >> 8U) & 0x0FU));

            /* 8. Включение гироскопа (LN) + акселерометра (LN) */
            pwr_val = (uint8_t)(ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN);
            ICM_WriteReg(sensor, ICM45686_REG_PWR_MGMT0, pwr_val);

            /* 9. Ожидание готовности */
            Delay_ms(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;  /* 0 = все исправны */
}

/* ================================================================
 * BusBurstRead_Start — запуск DMA-чтения для датчика sensor_idx на шине
 * Вызывать только для ИСПРАВНЫХ датчиков (fault == 0).
 * ================================================================ */
static void BusBurstRead_Start(ICM_Bus_t *bus, uint8_t sensor_idx)
{
    ICM_Sensor_t *sensor = &bus->sensors[sensor_idx];
    DMA_TypeDef  *dma    = bus->dma;
    uint32_t      rx_st  = bus->dma_stream_rx;
    uint32_t      tx_st  = bus->dma_stream_tx;

    uint8_t bus_idx;
    if (bus == &g_bus_spi1)      { bus_idx = 0U; }
    else if (bus == &g_bus_spi4) { bus_idx = 1U; }
    else                          { bus_idx = 2U; }

    /* Остановить потоки */
    LL_DMA_DisableStream(dma, rx_st);
    LL_DMA_DisableStream(dma, tx_st);
    while (LL_DMA_IsEnabledStream(dma, rx_st) != 0U) {}
    while (LL_DMA_IsEnabledStream(dma, tx_st) != 0U) {}

    /* Сброс флагов */
    if (dma == DMA1)
    {
        if (rx_st == LL_DMA_STREAM_2) { LL_DMA_ClearFlag_TC2(DMA1); LL_DMA_ClearFlag_TE2(DMA1); }
        if (tx_st == LL_DMA_STREAM_3) { LL_DMA_ClearFlag_TC3(DMA1); LL_DMA_ClearFlag_TE3(DMA1); }
    }
    else
    {
        if (rx_st == LL_DMA_STREAM_0) { LL_DMA_ClearFlag_TC0(DMA2); LL_DMA_ClearFlag_TE0(DMA2); }
        if (tx_st == LL_DMA_STREAM_1) { LL_DMA_ClearFlag_TC1(DMA2); LL_DMA_ClearFlag_TE1(DMA2); }
        if (rx_st == LL_DMA_STREAM_2) { LL_DMA_ClearFlag_TC2(DMA2); LL_DMA_ClearFlag_TE2(DMA2); }
        if (tx_st == LL_DMA_STREAM_3) { LL_DMA_ClearFlag_TC3(DMA2); LL_DMA_ClearFlag_TE3(DMA2); }
    }

    /* RX DMA: SPI RX → g_fifo_data[bus_idx][sensor_idx] */
    LL_DMA_SetMemoryAddress(dma, rx_st, (uint32_t)g_fifo_data[bus_idx][sensor_idx]);
    LL_DMA_SetPeriphAddress(dma, rx_st, LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetDataLength(dma, rx_st, ICM_FIFO_DMA_BUF_SIZE);

    /* TX DMA: tx_buf → SPI TX */
    LL_DMA_SetMemoryAddress(dma, tx_st, (uint32_t)bus->tx_buf);
    LL_DMA_SetPeriphAddress(dma, tx_st, LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetDataLength(dma, tx_st, ICM_FIFO_DMA_BUF_SIZE);

    /* Разрешить TC-прерывание только для RX */
    LL_DMA_EnableIT_TC(dma, rx_st);

    /* CS LOW — активировать датчик */
    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* SPI: включить DMA-режим и задать размер */
    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);
    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    /* Запустить: сначала RX, затем TX */
    LL_DMA_EnableStream(dma, rx_st);
    LL_DMA_EnableStream(dma, tx_st);

    SPI_Enable(bus->spi);
}

/* ================================================================
 * BusFinalize — завершение шины: сигнализировать и запустить следующую
 * ================================================================ */
static void BusFinalize(ICM_Bus_t *bus)
{
    bus->transfer_complete  = 1U;
    bus->current_sensor_idx = 0U;

    uint8_t cnt = ++g_buses_done_cnt;

    if (bus == &g_bus_spi1)
    {
        /* SPI1 готова → запускаем первый исправный датчик SPI4 */
        uint8_t first = Bus_FindNextOK(&g_bus_spi4, 0U);
        if (first < ICM_SENSORS_PER_BUS)
        {
            g_bus_spi4.current_sensor_idx = first;
            BusBurstRead_Start(&g_bus_spi4, first);
        }
        else
        {
            /* Все датчики SPI4 неисправны — сразу финализируем */
            BusFinalize(&g_bus_spi4);
        }
    }
    else if (bus == &g_bus_spi4)
    {
        /* SPI4 готова → запускаем первый исправный датчик SPI5 */
        uint8_t first = Bus_FindNextOK(&g_bus_spi5, 0U);
        if (first < ICM_SENSORS_PER_BUS)
        {
            g_bus_spi5.current_sensor_idx = first;
            BusBurstRead_Start(&g_bus_spi5, first);
        }
        else
        {
            /* Все датчики SPI5 неисправны — сразу финализируем */
            BusFinalize(&g_bus_spi5);
        }
    }
    else
    {
        /* SPI5 готова — все три шины завершены */
        if (cnt >= 3U)
        {
            g_buses_done_cnt   = 0U;
            g_fifo_batch_ready = 1U;  /* Сигнал main-loop */
        }
    }
}

/* ================================================================
 * BusBurstRead_Next — обработка завершения датчика, переход к следующему.
 * Вызывается из RX TC ISR.
 * ================================================================ */
static void BusBurstRead_Next(ICM_Bus_t *bus, uint8_t prev_idx)
{
    /* CS HIGH — деактивировать завершённый датчик */
    LL_GPIO_SetOutputPin(bus->sensors[prev_idx].cs_port,
                         bus->sensors[prev_idx].cs_pin);

    SPI_Disable(bus->spi);
    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);

    /* Ищем следующий ИСПРАВНЫЙ датчик после prev_idx */
    uint8_t next = Bus_FindNextOK(bus, prev_idx + 1U);

    if (next < ICM_SENSORS_PER_BUS)
    {
        /* Есть ещё исправные датчики на этой шине */
        bus->current_sensor_idx = next;
        BusBurstRead_Start(bus, next);
    }
    else
    {
        /* Все датчики шины опрошены (или пропущены) */
        BusFinalize(bus);
    }
}

/* ================================================================
 * ICM_StartBurstRead — точка входа из TIM6 ISR
 * ================================================================ */
void ICM_StartBurstRead(void)
{
    /* Предохранитель: предыдущий цикл ещё не обработан main-loop */
    if (g_fifo_batch_ready != 0U)
    {
        return;
    }

    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;

    /* Найти первый исправный датчик SPI1 */
    uint8_t first = Bus_FindNextOK(&g_bus_spi1, 0U);
    if (first < ICM_SENSORS_PER_BUS)
    {
        g_bus_spi1.current_sensor_idx = first;
        BusBurstRead_Start(&g_bus_spi1, first);
    }
    else
    {
        /* Все датчики SPI1 неисправны — сразу финализируем шину */
        BusFinalize(&g_bus_spi1);
    }
}

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
