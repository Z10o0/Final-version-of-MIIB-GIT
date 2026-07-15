/**
 * @file    icm45686_spi.c
 * @brief   Реализация DMA-транспорта для 18 датчиков ICM-45686 на STM32H723ZGT6.
 *
 * ──────────────────────────────────────────────────────────────
 *  АРХИТЕКТУРА КАСКАДНОГО DMA-ОПРОСА (3 шины SPI)
 * ──────────────────────────────────────────────────────────────
 *
 *  TIM6 UPDATE IRQ (каждые ~3.125 мс при ODR=3200 Гц)
 *     │
 *     ▼
 *  ICM_StartBurstRead()              ← единая точка входа
 *     │ запускает первый ИСПРАВНЫЙ датчик SPI1 (CS13..CS18)
 *     ▼
 *  DMA1_Stream2 TC ISR → ICM_DMA_RxComplete_SPI1()
 *     │ по очереди все 6 датчиков SPI1
 *     │ после последнего → запуск первого исправного SPI4
 *     ▼
 *  DMA2_Stream0 TC ISR → ICM_DMA_RxComplete_SPI4()
 *     │ по очереди все 6 датчиков SPI4 (CS1..CS6)
 *     │ после последнего → запуск первого исправного SPI5
 *     ▼
 *  DMA2_Stream2 TC ISR → ICM_DMA_RxComplete_SPI5()
 *     │ по очереди все 6 датчиков SPI5 (CS7..CS12)
 *     │ после последнего → g_fifo_batch_ready = 1
 *     ▼
 *  main-loop: g_fifo_batch_ready → ICM_ParseAllFIFO() → UART_SendBatch()
 *
 * ──────────────────────────────────────────────────────────────
 *  КРИТИЧЕСКИЙ ПОРЯДОК SPI-ТРАНЗАКЦИИ (блокирующий режим):
 *    1. CS LOW
 *    2. LL_SPI_SetTransferSize(N)
 *    3. LL_SPI_Enable() + LL_SPI_StartMasterTransfer()
 *    4. Записать байты через LL_SPI_TransmitData8()
 *    5. Дождаться EOT (LL_SPI_IsActiveFlag_EOT)
 *    6. Считать RX FIFO (LL_SPI_ReceiveData8)
 *    7. CS HIGH
 *    8. LL_SPI_Disable()
 *  Нарушение порядка шагов 1/2/6/7 — частая причина сбоев!
 *
 * ──────────────────────────────────────────────────────────────
 *  ПОРЯДОК IREG (CLKIN-активация):
 *    До PWR_MGMT0 для каждого датчика вызывается:
 *      ICM_WriteIReg(IOC_PAD_SCENARIO_OVRD, 0x06)  → CLKIN enable
 *      ICM_WriteIReg(SMC_CONTROL_0,         0x42)  → RTC_MODE + TMST_EN
 *    Без этих шагов датчик игнорирует внешний CLKIN-сигнал!
 *
 * ──────────────────────────────────────────────────────────────
 *  ОГРАНИЧЕНИЯ ПАМЯТИ DMA:
 *    DMA1/DMA2 НЕ имеют доступа к ITCM (0x00000000) и DTCM (0x20000000).
 *    Все DMA-буферы (g_fifo_data, tx_buf) должны быть в SRAM D2 (AXI).
 *    Для этого разместить переменные в секции .RAM_D2 через линкер-скрипт
 *    или атрибут: __attribute__((section(".RAM_D2"))) __attribute__((aligned(4)))
 *
 * ──────────────────────────────────────────────────────────────
 *  НЕИСПРАВНЫЕ ДАТЧИКИ (fault = 1):
 *    - CS не трогается в DMA-каскаде
 *    - g_fifo_data для fault-датчика остаётся нулями (memset при старте)
 *    - ICM_ParseAllFIFO() записывает нули в g_sensor_batches
 *    - Бит N в g_sensor_fault_mask = 1 если датчик N неисправен
 */

#include "icm45686_spi.h"
#include "icm45686_regs.h"
#include "icm45686_config.h"
#include "main.h"
#include <string.h>

/* ================================================================
 * Внутренние прототипы (не экспортируются)
 * ================================================================ */
static void    BusBurstRead_Start(ICM_Bus_t *bus, uint8_t sensor_idx);
static void    BusBurstRead_Next(ICM_Bus_t *bus, uint8_t prev_idx);
static void    BusFinalize(ICM_Bus_t *bus);
static void    SPI_WaitEOT(SPI_TypeDef *spi);
static void    SPI_EnableDMA(SPI_TypeDef *spi);
static void    SPI_DisableDMA(SPI_TypeDef *spi);
static void    Delay_ms(uint32_t ms);
static uint8_t Bus_FindNextOK(ICM_Bus_t *bus, uint8_t start_idx);
static uint8_t Bus_GetIdx(ICM_Bus_t *bus);

/* ================================================================
 * Сырые RX-буферы FIFO: [шина 0/1/2][датчик 0..5][байт]
 *
 * ОБЯЗАТЕЛЬНО: разместить в AXI SRAM (D2), не в DTCM/ITCM.
 * Если линкер-скрипт настроен на DTCM по умолчанию — добавить
 * атрибут section(".RAM_D2") или перенастроить CubeMX.
 * ================================================================ */
uint8_t g_fifo_data[3][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((aligned(4)));

/* ================================================================
 * Флаг готовности пачки данных.
 * Устанавливается в ISR ICM_DMA_RxComplete_SPI5(), когда все
 * три шины завершили DMA-опрос за текущий тик TIM6.
 * Очищается в main-loop перед ICM_ParseAllFIFO().
 * volatile — доступ из ISR и main-loop.
 * ================================================================ */
volatile uint8_t g_fifo_batch_ready = 0U;

/* ================================================================
 * Маска неисправных датчиков.
 * Бит N = 1 если датчик N не ответил на WHO_AM_I.
 * Записывается один раз в ICM_InitAllSensors().
 * ================================================================ */
volatile uint32_t g_sensor_fault_mask = 0U;

/* ================================================================
 * Счётчик шин, завершивших опрос за текущий цикл TIM6.
 * Инкрементируется в BusFinalize(). Сбрасывается после = 3.
 * volatile — модифицируется в ISR-контексте (через BusFinalize).
 * ================================================================ */
static volatile uint8_t g_buses_done_cnt = 0U;

/* ================================================================
 * ДЕСКРИПТОРЫ ТРЁХ ШИН
 *
 * Соответствие CS-меток (из main.h, генерируется CubeMX) и пинов:
 *
 *  SPI1 (DMA1, Stream2/3):
 *    sensor_id 0  → CS13 = CS31_Pin = PF13
 *    sensor_id 1  → CS14 = CS32_Pin = PF14
 *    sensor_id 2  → CS15 = CS33_Pin = PE8
 *    sensor_id 3  → CS16 = CS34_Pin = PE9
 *    sensor_id 4  → CS17 = CS35_Pin = PB13
 *    sensor_id 5  → CS18 = CS36_Pin = PB12
 *
 *  SPI4 (DMA2, Stream0/1):
 *    sensor_id 6  → CS1  = CS19_Pin = PC4
 *    sensor_id 7  → CS2  = CS20_Pin = PC5
 *    sensor_id 8  → CS3  = CS21_Pin = PG0
 *    sensor_id 9  → CS4  = CS22_Pin = PF15
 *    sensor_id 10 → CS5  = CS23_Pin = PE10
 *    sensor_id 11 → CS6  = CS24_Pin = PE11
 *
 *  SPI5 (DMA2, Stream2/3):
 *    sensor_id 12 → CS7  = CS25_Pin = PB0
 *    sensor_id 13 → CS8  = CS26_Pin = PB1
 *    sensor_id 14 → CS9  = CS27_Pin = PE7
 *    sensor_id 15 → CS10 = CS28_Pin = PG1
 *    sensor_id 16 → CS11 = CS29_Pin = PE14
 *    sensor_id 17 → CS12 = CS30_Pin = PE15
 * ================================================================ */

ICM_Bus_t g_bus_spi1 = {
    .spi           = SPI1,
    .dma           = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .sensors = {
        { SPI1, CS31_GPIO_Port, CS31_Pin,  0U, 0U }, /* PF13 — CS13 */
        { SPI1, CS32_GPIO_Port, CS32_Pin,  1U, 0U }, /* PF14 — CS14 */
        { SPI1, CS33_GPIO_Port, CS33_Pin,  2U, 0U }, /* PE8  — CS15 */
        { SPI1, CS34_GPIO_Port, CS34_Pin,  3U, 0U }, /* PE9  — CS16 */
        { SPI1, CS35_GPIO_Port, CS35_Pin,  4U, 0U }, /* PB13 — CS17 */
        { SPI1, CS36_GPIO_Port, CS36_Pin,  5U, 0U }, /* PB12 — CS18 */
    }
};

ICM_Bus_t g_bus_spi4 = {
    .spi           = SPI4,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,
    .sensors = {
        { SPI4, CS19_GPIO_Port, CS19_Pin,  6U, 0U }, /* PC4  — CS1  */
        { SPI4, CS20_GPIO_Port, CS20_Pin,  7U, 0U }, /* PC5  — CS2  */
        { SPI4, CS21_GPIO_Port, CS21_Pin,  8U, 0U }, /* PG0  — CS3  */
        { SPI4, CS22_GPIO_Port, CS22_Pin,  9U, 0U }, /* PF15 — CS4  */
        { SPI4, CS23_GPIO_Port, CS23_Pin, 10U, 0U }, /* PE10 — CS5  */
        { SPI4, CS24_GPIO_Port, CS24_Pin, 11U, 0U }, /* PE11 — CS6  */
    }
};

ICM_Bus_t g_bus_spi5 = {
    .spi           = SPI5,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .sensors = {
        { SPI5, CS25_GPIO_Port, CS25_Pin, 12U, 0U }, /* PB0  — CS7  */
        { SPI5, CS26_GPIO_Port, CS26_Pin, 13U, 0U }, /* PB1  — CS8  */
        { SPI5, CS27_GPIO_Port, CS27_Pin, 14U, 0U }, /* PE7  — CS9  */
        { SPI5, CS28_GPIO_Port, CS28_Pin, 15U, 0U }, /* PG1  — CS10 */
        { SPI5, CS29_GPIO_Port, CS29_Pin, 16U, 0U }, /* PE14 — CS11 */
        { SPI5, CS30_GPIO_Port, CS30_Pin, 17U, 0U }, /* PE15 — CS12 */
    }
};

/* ================================================================
 * Bus_GetIdx — индекс шины в массиве g_fifo_data
 * SPI1=0, SPI4=1, SPI5=2
 * ================================================================ */
static uint8_t Bus_GetIdx(ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1) { return 0U; }
    if (bus == &g_bus_spi4) { return 1U; }
    return 2U;
}

/* ================================================================
 * Bus_FindNextOK — найти индекс следующего исправного датчика.
 * start_idx — с какого индекса начинать поиск (включительно).
 * Возвращает ICM_SENSORS_PER_BUS если исправных датчиков нет.
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
    return ICM_SENSORS_PER_BUS; /* Все неисправны */
}

/* ================================================================
 * SPI_WaitEOT — ожидание флага End Of Transfer (для блокирующего режима).
 *
 * Используется в ICM_WriteReg / ICM_ReadReg.
 * В DMA-пути НЕ используется (там работаем по TC ISR).
 * ================================================================ */
static inline void SPI_WaitEOT(SPI_TypeDef *spi)
{
    /* EOT = все байты TX отправлены И все байты RX получены */
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

/* ================================================================
 * SPI_EnableDMA — включение SPI в DMA-режиме.
 * Размер транзакции должен быть задан до вызова этой функции.
 * ================================================================ */
static inline void SPI_EnableDMA(SPI_TypeDef *spi)
{
    LL_SPI_EnableDMAReq_RX(spi);
    LL_SPI_EnableDMAReq_TX(spi);
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);
}

/* ================================================================
 * SPI_DisableDMA — отключение SPI после DMA-транзакции.
 *
 * ВАЖНО: вызывать только в ISR-контексте, где EOT уже наступил
 * (DMA TC происходит после последнего байта, EOT — сразу после).
 * Не содержит busy-wait — безопасно вызывать из ISR.
 * ================================================================ */
static inline void SPI_DisableDMA(SPI_TypeDef *spi)
{
    LL_SPI_DisableDMAReq_RX(spi);
    LL_SPI_DisableDMAReq_TX(spi);
    /* Очистить флаги завершения транзакции */
    if (LL_SPI_IsActiveFlag_EOT(spi))
    {
        LL_SPI_ClearFlag_EOT(spi);
        LL_SPI_ClearFlag_TXTF(spi);
    }
    LL_SPI_Disable(spi);
}

/* ================================================================
 * Delay_ms — программная задержка на основе SysTick.
 *
 * Используется только в ICM_InitAllSensors() (не в DMA-цикле).
 * uwTick инкрементируется в SysTick_Handler() каждые 1 мс.
 * Объявлен как extern — генерируется CubeMX в stm32h7xx_it.c.
 * ================================================================ */

static void Delay_ms(uint32_t ms)
{
    /* Задержка через регистры SysTick — без зависимости от HAL (uwTick).
     * SysTick->LOAD = SystemCoreClock/1000 - 1 при стандартной настройке CubeMX.
     * Считаем убывающие тики VAL, обрабатываем переполнение (wrap-around). */
    uint32_t ticks_per_ms = SysTick->LOAD + 1UL;
    uint32_t total_ticks  = ms * ticks_per_ms;
    uint32_t elapsed      = 0UL;
    uint32_t prev_val     = SysTick->VAL;
    uint32_t curr_val;

    while (elapsed < total_ticks)
    {
        curr_val = SysTick->VAL;
        if (curr_val < prev_val)
        {
            elapsed += (prev_val - curr_val);          /* Обычный убывающий счёт */
        }
        else
        {
            elapsed += (prev_val + ticks_per_ms - curr_val); /* Переполнение VAL */
        }
        prev_val = curr_val;
    }
}

/* ================================================================
 * ICM_BusesInit — первичная инициализация структур шин.
 *
 * Выполняет:
 *   1. CS HIGH для всех 18 датчиков (деактивация линий выбора)
 *   2. Сброс fault-флагов всех датчиков
 *   3. Заполнение TX-буферов: [0] = FIFO burst-read команда,
 *      [1..N] = 0xFF (dummy-байты для генерации тактовых импульсов)
 *   4. Обнуление g_fifo_data (fault-датчики останутся нулями навсегда)
 *   5. Сброс управляющих флагов и счётчиков
 *
 * ВЫЗЫВАТЬ ДО ICM_InitAllSensors().
 * ================================================================ */
void ICM_BusesInit(void)
{
    uint8_t s;

    /* ── Взвести все CS HIGH (неактивное состояние) ── */
    for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
    {
        LL_GPIO_SetOutputPin(g_bus_spi1.sensors[s].cs_port,
                             g_bus_spi1.sensors[s].cs_pin);
        LL_GPIO_SetOutputPin(g_bus_spi4.sensors[s].cs_port,
                             g_bus_spi4.sensors[s].cs_pin);
        LL_GPIO_SetOutputPin(g_bus_spi5.sensors[s].cs_port,
                             g_bus_spi5.sensors[s].cs_pin);

        /* Сброс fault-флагов — все датчики считаются исправными до проверки */
        g_bus_spi1.sensors[s].fault = 0U;
        g_bus_spi4.sensors[s].fault = 0U;
        g_bus_spi5.sensors[s].fault = 0U;
    }

    /* ── Подготовить TX-буферы ── */
    /* Байт [0]: команда SPI burst-read FIFO: addr | READ_BIT */
    /* Байты [1..N]: 0xFF — dummy для генерации SCK при чтении RX */
    memset(g_bus_spi1.tx_buf, 0xFFU, sizeof(g_bus_spi1.tx_buf));
    memset(g_bus_spi4.tx_buf, 0xFFU, sizeof(g_bus_spi4.tx_buf));
    memset(g_bus_spi5.tx_buf, 0xFFU, sizeof(g_bus_spi5.tx_buf));

    g_bus_spi1.tx_buf[0] = (uint8_t)(ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT);
    g_bus_spi4.tx_buf[0] = (uint8_t)(ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT);
    g_bus_spi5.tx_buf[0] = (uint8_t)(ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT);

    /* ── Обнуление RX-буферов ── */
    /* Для fault-датчиков данные в этих буферах останутся нулями */
    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));

    /* ── Сброс управляющих переменных ── */
    g_fifo_batch_ready  = 0U;
    g_buses_done_cnt    = 0U;
    g_sensor_fault_mask = 0U;

    g_bus_spi1.current_sensor_idx = 0U;
    g_bus_spi4.current_sensor_idx = 0U;
    g_bus_spi5.current_sensor_idx = 0U;

    g_bus_spi1.transfer_complete  = 0U;
    g_bus_spi4.transfer_complete  = 0U;
    g_bus_spi5.transfer_complete  = 0U;
}

/* ================================================================
 * ICM_WriteReg — блокирующая запись регистра USER BANK 0.
 *
 * Правильный порядок для STM32H7 SPI (TSIZE mode):
 *   1. CS LOW
 *   2. SetTransferSize(2)
 *   3. Enable + StartMasterTransfer
 *   4. Передать addr_byte (без READ_BIT)
 *   5. Передать data_byte
 *   6. Ждать EOT
 *   7. CS HIGH
 *   8. Disable
 *
 * Используется ТОЛЬКО в ICM_InitAllSensors() и ICM_WriteIReg().
 * В DMA-цикле не вызывать!
 * ================================================================ */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t val)
{
    SPI_TypeDef *spi = sensor->spi;

    /* 1. CS LOW — активировать датчик ПЕРЕД включением SPI */
    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* 2. Задать количество байт транзакции */
    LL_SPI_SetTransferSize(spi, 2U);

    /* 3. Включить SPI и запустить мастер-передачу */
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    /* 4. Байт адреса регистра (бит7=0 → запись) */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, (uint8_t)(reg & 0x7FU));

    /* 5. Байт данных */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, val);

    /* 6. Ждать конца транзакции */
    SPI_WaitEOT(spi);

    /* 7. CS HIGH — деактивировать датчик */
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* 8. Выключить SPI */
    LL_SPI_Disable(spi);
}

/* ================================================================
 * ICM_ReadReg — блокирующее чтение регистра USER BANK 0.
 *
 * Правильный порядок для STM32H7 SPI (TSIZE mode):
 *   1. CS LOW
 *   2. SetTransferSize(2)
 *   3. Enable + StartMasterTransfer
 *   4. Передать addr_byte | READ_BIT
 *   5. Передать dummy 0xFF
 *   6. Ждать EOT
 *   7. Прочитать RX FIFO (2 байта: dummy + data)
 *   8. CS HIGH
 *   9. Disable
 *
 * Используется ТОЛЬКО в ICM_InitAllSensors() и ICM_ReadIReg().
 * В DMA-цикле не вызывать!
 * ================================================================ */
uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg)
{
    uint8_t     result = 0U;
    SPI_TypeDef *spi   = sensor->spi;

    /* 1. CS LOW */
    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* 2. Задать количество байт */
    LL_SPI_SetTransferSize(spi, 2U);

    /* 3. Включить SPI */
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    /* 4. Передать адрес с битом READ */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, (uint8_t)(reg | ICM45686_SPI_READ_BIT));

    /* 5. Dummy-байт для генерации тактов при приёме данных */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, 0xFFU);

    /* 6. Ждать конца транзакции (все байты RX приняты) */
    SPI_WaitEOT(spi);

    /* 7. Читать RX FIFO: первый байт — dummy (ответ на адресный байт),
     *    второй — реальные данные регистра */
    (void)LL_SPI_ReceiveData8(spi); /* dummy */
    result = LL_SPI_ReceiveData8(spi);

    /* 8. CS HIGH */
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* 9. Выключить SPI */
    LL_SPI_Disable(spi);

    return result;
}

/* ================================================================
 * ICM_WriteIReg — блокирующая запись внутреннего IREG-регистра.
 *
 * Используется для настройки CLKIN через IPREG_TOP1:
 *   ICM_WriteIReg(sensor, 0xA4, 0x30, 0x06)  → IOC_PAD_SCENARIO_OVRD: CLKIN enable
 *   ICM_WriteIReg(sensor, 0xA4, 0x58, 0x42)  → SMC_CONTROL_0: RTC_MODE + TMST_EN
 *
 * Трёхшаговая процедура косвенного доступа:
 *   1. Записать старший байт IREG-адреса в IREG_ADDR_15_8
 *   2. Записать младший байт IREG-адреса в IREG_ADDR_7_0
 *   3. Записать данные в IREG_DATA
 *   4. Пауза >= 10 мкс (ждём пока аппаратура завершит транзакцию к IREG)
 *
 * ВЫЗЫВАТЬ ДО PWR_MGMT0, иначе CLKIN игнорируется!
 * ================================================================ */
void ICM_WriteIReg(ICM_Sensor_t *sensor, uint8_t addr_h, uint8_t addr_l, uint8_t val)
{
    /* Шаг 1: старший байт 16-битного IREG-адреса */
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);

    /* Шаг 2: младший байт 16-битного IREG-адреса */
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0, addr_l);

    /* Шаг 3: данные — аппаратура выполняет запись к IREG автоматически */
    ICM_WriteReg(sensor, ICM45686_REG_IREG_DATA, val);

    /* Шаг 4: обязательная пауза >= 10 мкс (1 мс с запасом) */
    Delay_ms(ICM45686_IREG_DELAY_MS);
}

/* ================================================================
 * ICM_ReadIReg — блокирующее чтение внутреннего IREG-регистра.
 *
 * Используется для верификации значений IREG после записи.
 * Аналогично ICM_WriteIReg, но читает IREG_DATA.
 * ================================================================ */
uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor, uint8_t addr_h, uint8_t addr_l)
{
    /* Шаг 1-2: задать IREG-адрес */
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);

    /* Шаг 3: инициировать чтение */
    Delay_ms(ICM45686_IREG_DELAY_MS);

    /* Шаг 4: прочитать результат */
    return ICM_ReadReg(sensor, ICM45686_REG_IREG_DATA);
}

/* ================================================================
 * ICM_InitAllSensors — инициализация всех 18 датчиков.
 *
 * Для каждого датчика выполняется:
 *   1.  Программный сброс (DEVICE_CONFIG[0]=1) + задержка
 *   2.  Проверка WHO_AM_I → при несовпадении: fault=1, continue
 *   3.  Выбор USER BANK 0
 *   4.  IREG: IOC_PAD_SCENARIO_OVRD = 0x06  (активация CLKIN)
 *   5.  IREG: SMC_CONTROL_0 = 0x42          (RTC_MODE + TMST_EN)
 *   6.  ACCEL_CONFIG0: FS + ODR из icm45686_config.h
 *   7.  GYRO_CONFIG0:  FS + ODR из icm45686_config.h
 *   8.  FIFO_CONFIG0:  STREAM, gyro+accel+temp+timestamp
 *   9.  FIFO watermark = ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES
 *   10. PWR_MGMT0: gyro LN + accel LN
 *   11. Задержка 200 мс (прогрев гироскопа до стабильных данных)
 *
 * Неисправные датчики НЕ останавливают процедуру.
 * Возвращает маску: бит N = 1 если датчик N неисправен.
 * ================================================================ */
uint32_t ICM_InitAllSensors(void)
{
    ICM_Bus_t   *buses[3] = { &g_bus_spi1, &g_bus_spi4, &g_bus_spi5 };
    uint8_t      b, s;
    uint8_t      whoami;
    uint8_t      ireg_check;
    ICM_Sensor_t *sensor;

    g_sensor_fault_mask = 0U;

    for (b = 0U; b < 3U; b++)
    {
        for (s = 0U; s < ICM_SENSORS_PER_BUS; s++)
        {
            sensor        = &buses[b]->sensors[s];
            sensor->fault = 0U;

            /* ── Шаг 1: программный сброс ── */
            ICM_WriteReg(sensor, ICM45686_REG_DEVICE_CONFIG, 0x01U);
            Delay_ms(ICM45686_RESET_DELAY_MS);

            /* ── Шаг 2: проверка WHO_AM_I ── */
            whoami = ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I);
            if (whoami != ICM45686_WHO_AM_I_VALUE)
            {
                /* Датчик не отвечает — помечаем неисправным, продолжаем */
                sensor->fault        = 1U;
                g_sensor_fault_mask |= (1UL << sensor->sensor_id);
                continue;
            }

            /* ── Шаг 3: выбор USER BANK 0 ── */
            ICM_WriteReg(sensor, ICM45686_REG_BANK_SEL, 0x00U);

            /* ── Шаг 4: активация CLKIN через IREG ──
             * IOC_PAD_SCENARIO_OVRD @ 0xA430:
             *   бит[2]=1 (CLKIN_EN_OVRD_VAL) + бит[1]=1 (CLKIN_EN_OVRD)
             *   = пин INT2 перенастраивается как вход CLKIN */
            ICM_WriteIReg(sensor,
                          ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_H,
                          ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L,
                          ICM45686_CLKIN_ENABLE_VAL);

            /* Верификация: убедиться что CLKIN включился */
            ireg_check = ICM_ReadIReg(sensor,
                                      ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_H,
                                      ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L);
            if (ireg_check != ICM45686_CLKIN_ENABLE_VAL)
            {
                /* IREG-запись не прошла — датчик считается неисправным */
                sensor->fault        = 1U;
                g_sensor_fault_mask |= (1UL << sensor->sensor_id);
                continue;
            }

            /* ── Шаг 5: включение RTC_MODE + TMST_EN через IREG ──
             * SMC_CONTROL_0 @ 0xA458:
             *   бит[6]=1 (RTC_MODE): ODR тактируется от внешнего CLKIN
             *   бит[1]=1 (TMST_EN):  включить timestamp в FIFO-пакет */
            ICM_WriteIReg(sensor,
                          ICM45686_IREG_SMC_CONTROL_0_H,
                          ICM45686_IREG_SMC_CONTROL_0_L,
                          ICM45686_RTC_MODE_TMST_ENABLE_VAL);

            /* ── Шаг 6: конфигурация акселерометра ──
             * Биты [7:5] = FS, биты [3:0] = ODR
             * Значения задаются в icm45686_config.h */
            ICM_WriteReg(sensor, ICM45686_REG_ACCEL_CONFIG0,
                         (uint8_t)(ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE));

            /* ── Шаг 7: конфигурация гироскопа ──
             * Биты [7:4] = FS, биты [3:0] = ODR */
            ICM_WriteReg(sensor, ICM45686_REG_GYRO_CONFIG0,
                         (uint8_t)(ICM_GYRO_FS_VALUE | ICM_GYRO_ODR_VALUE));

            /* ── Шаг 8: настройка FIFO ──
             * STREAM-режим: новые данные вытесняют старые при переполнении.
             * Включаем: гироскоп + акселерометр + температура + timestamp */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG0,
                         (uint8_t)(ICM45686_FIFO_MODE_STREAM |
                                   ICM45686_FIFO_SEL_GYRO    |
                                   ICM45686_FIFO_SEL_ACCEL   |
                                   ICM45686_FIFO_SEL_TEMP    |
                                   ICM45686_FIFO_SEL_TMST));

            /* ── Шаг 9: порог FIFO (watermark) ──
             * При достижении порога генерируется флаг INT_STATUS3.FIFO_THS.
             * Порог = количество пакетов × размер одного пакета.
             * Двухбайтовая запись: [0] = LSB, [1] = MSB[3:0] */
            {
                uint16_t wm = (uint16_t)(ICM_FIFO_POLL_PACKETS *
                                         ICM_FIFO_PACKET_BYTES);
                ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_0,
                             (uint8_t)(wm & 0x00FFU));
                ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_1,
                             (uint8_t)((wm >> 8U) & 0x000FU));
            }

            /* ── Шаг 10: включение питания ──
             * Гироскоп: Low-Noise mode (высшая точность)
             * Акселерометр: Low-Noise mode */
            ICM_WriteReg(sensor, ICM45686_REG_PWR_MGMT0,
                         (uint8_t)(ICM45686_PWR_GYRO_MODE_LN |
                                   ICM45686_PWR_ACCEL_MODE_LN));

            /* ── Шаг 11: прогрев гироскопа ──
             * Минимум 200 мс до стабилизации выходных данных */
            Delay_ms(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}

/* ================================================================
 * BusBurstRead_Start — запуск DMA-чтения FIFO для одного датчика.
 *
 * Правильная последовательность для STM32H7 SPI+DMA:
 *   1. Остановить оба DMA-стрима (на случай повторного запуска)
 *   2. Сбросить флаги TC/TE
 *   3. Настроить адреса памяти/периферии и длину для RX и TX
 *   4. Разрешить TC-прерывание только для RX-стрима
 *   5. CS LOW
 *   6. SetTransferSize → Enable DMA-запросы → Enable SPI → Start
 *   7. Запустить RX-стрим, затем TX-стрим
 *
 * ВЫЗЫВАТЬ только для исправных датчиков (fault == 0)!
 * ================================================================ */
static void BusBurstRead_Start(ICM_Bus_t *bus, uint8_t sensor_idx)
{
    ICM_Sensor_t *sensor  = &bus->sensors[sensor_idx];
    DMA_TypeDef  *dma     = bus->dma;
    uint32_t      rx_st   = bus->dma_stream_rx;
    uint32_t      tx_st   = bus->dma_stream_tx;
    uint8_t       bus_idx = Bus_GetIdx(bus);

    /* ── 1. Остановить стримы ── */
    LL_DMA_DisableStream(dma, rx_st);
    LL_DMA_DisableStream(dma, tx_st);
    /* Ждать физического останова (FIFO может ещё работать) */
    while (LL_DMA_IsEnabledStream(dma, rx_st) != 0U) {}
    while (LL_DMA_IsEnabledStream(dma, tx_st) != 0U) {}

    /* ── 2. Сброс флагов TC, TE, HT, FE ──
     * Необходимо сбрасывать ВСЕ флаги, иначе ISR не будет вызван */
    if (dma == DMA1)
    {
        /* RX Stream2 */
        LL_DMA_ClearFlag_TC2(DMA1);
        LL_DMA_ClearFlag_TE2(DMA1);
        LL_DMA_ClearFlag_HT2(DMA1);
        /* TX Stream3 */
        LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1);
        LL_DMA_ClearFlag_HT3(DMA1);
    }
    else /* DMA2 — используется SPI4 и SPI5 */
    {
        if (rx_st == LL_DMA_STREAM_0)       /* SPI4 RX */
        {
            LL_DMA_ClearFlag_TC0(DMA2);
            LL_DMA_ClearFlag_TE0(DMA2);
            LL_DMA_ClearFlag_HT0(DMA2);
            LL_DMA_ClearFlag_TC1(DMA2);     /* SPI4 TX */
            LL_DMA_ClearFlag_TE1(DMA2);
            LL_DMA_ClearFlag_HT1(DMA2);
        }
        else                                 /* SPI5 RX = Stream2 */
        {
            LL_DMA_ClearFlag_TC2(DMA2);
            LL_DMA_ClearFlag_TE2(DMA2);
            LL_DMA_ClearFlag_HT2(DMA2);
            LL_DMA_ClearFlag_TC3(DMA2);     /* SPI5 TX */
            LL_DMA_ClearFlag_TE3(DMA2);
            LL_DMA_ClearFlag_HT3(DMA2);
        }
    }

    /* ── 3. Настройка адресов и длины ── */
    /* RX: данные из SPI DR → g_fifo_data[bus][sensor] */
    LL_DMA_SetMemoryAddress(dma, rx_st,
                            (uint32_t)g_fifo_data[bus_idx][sensor_idx]);
    LL_DMA_SetPeriphAddress(dma, rx_st,
                            LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetDataLength(dma, rx_st, ICM_FIFO_DMA_BUF_SIZE);

    /* TX: tx_buf (команда + dummy) → SPI DR */
    LL_DMA_SetMemoryAddress(dma, tx_st, (uint32_t)bus->tx_buf);
    LL_DMA_SetPeriphAddress(dma, tx_st,
                            LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetDataLength(dma, tx_st, ICM_FIFO_DMA_BUF_SIZE);

    /* ── 4. Разрешить TC-прерывание только для RX ──
     * TX не нужно: завершение фиксируется по RX TC */
    LL_DMA_EnableIT_TC(dma, rx_st);

    /* ── 5. CS LOW — активировать датчик ── */
    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* ── 6. Настройка SPI для DMA ── */
    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);

    /* ── 7. Запуск: сначала RX-стрим, потом TX-стрим, затем SPI ──
     * Порядок важен: RX должен быть готов принимать до начала TX */
    LL_DMA_EnableStream(dma, rx_st);
    LL_DMA_EnableStream(dma, tx_st);

    SPI_EnableDMA(bus->spi); /* DMA-запросы + Enable + Start */
}

/* ================================================================
 * BusFinalize — завершение шины, запуск следующей в каскаде.
 *
 * Вызывается из BusBurstRead_Next() когда на текущей шине
 * больше нет исправных датчиков для опроса.
 *
 * Каскад: SPI1 → SPI4 → SPI5 → g_fifo_batch_ready = 1
 * ================================================================ */
static void BusFinalize(ICM_Bus_t *bus)
{
    uint8_t first;
    uint8_t cnt;

    /* Отметить шину как завершённую */
    bus->transfer_complete  = 1U;
    bus->current_sensor_idx = 0U;

    /* Атомарное чтение-модификация не требуется: вызывается только из ISR */
    cnt = ++g_buses_done_cnt;

    if (bus == &g_bus_spi1)
    {
        /* SPI1 завершена → запуск SPI4 */
        first = Bus_FindNextOK(&g_bus_spi4, 0U);
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
        /* SPI4 завершена → запуск SPI5 */
        first = Bus_FindNextOK(&g_bus_spi5, 0U);
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
    else /* bus == &g_bus_spi5 */
    {
        /* SPI5 завершена — проверяем что все три шины готовы */
        if (cnt >= 3U)
        {
            g_buses_done_cnt   = 0U;
            /* Сигнал для main-loop: пачка данных готова к обработке */
            g_fifo_batch_ready = 1U;
        }
    }
}

/* ================================================================
 * BusBurstRead_Next — обработка TC ISR: деактивация датчика,
 * переход к следующему или завершение шины.
 *
 * Вызывается из ICM_DMA_RxComplete_SPI1/4/5().
 * ================================================================ */
static void BusBurstRead_Next(ICM_Bus_t *bus, uint8_t prev_idx)
{
    uint8_t next;

    /* ── Завершить транзакцию текущего датчика ── */

    /* Отключить DMA-запросы и SPI (без busy-wait — EOT уже наступил при TC) */
    SPI_DisableDMA(bus->spi);

    /* CS HIGH — деактивировать завершённый датчик */
    LL_GPIO_SetOutputPin(bus->sensors[prev_idx].cs_port,
                         bus->sensors[prev_idx].cs_pin);

    /* ── Найти следующий исправный датчик ── */
    next = Bus_FindNextOK(bus, (uint8_t)(prev_idx + 1U));

    if (next < ICM_SENSORS_PER_BUS)
    {
        /* Есть ещё исправные датчики на этой шине — продолжаем каскад */
        bus->current_sensor_idx = next;
        BusBurstRead_Start(bus, next);
    }
    else
    {
        /* Все датчики шины опрошены (или пропущены) — финализация */
        BusFinalize(bus);
    }
}

/* ================================================================
 * ICM_StartBurstRead — единая точка входа из TIM6 UPDATE ISR.
 *
 * Запускает новый цикл DMA-опроса всех трёх шин.
 * Если предыдущий цикл не был обработан main-loop (g_fifo_batch_ready=1),
 * функция пропускает запуск — защита от перетекания буферов.
 * ================================================================ */
void ICM_StartBurstRead(void)
{
    uint8_t first;

    /* Предохранитель: пропустить тик если предыдущая пачка не обработана */
    if (g_fifo_batch_ready != 0U)
    {
        return;
    }

    /* Сброс флагов завершения шин перед новым циклом */
    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;
    g_buses_done_cnt             = 0U;

    /* Запустить первый исправный датчик SPI1 */
    first = Bus_FindNextOK(&g_bus_spi1, 0U);
    if (first < ICM_SENSORS_PER_BUS)
    {
        g_bus_spi1.current_sensor_idx = first;
        BusBurstRead_Start(&g_bus_spi1, first);
    }
    else
    {
        /* Все датчики SPI1 неисправны — запустить каскад с SPI4 */
        BusFinalize(&g_bus_spi1);
    }
}

/* Псевдоним для обратной совместимости с предыдущими версиями main.c */
void ICM_StartBurstRead_SPI1(void)
{
    ICM_StartBurstRead();
}

/* ================================================================
 * ISR-обработчики DMA RX Transfer Complete.
 * Вызываются из stm32h7xx_it.c в соответствующих DMA ISR:
 *
 *   DMA1_Stream2_IRQHandler → ICM_DMA_RxComplete_SPI1
 *   DMA2_Stream0_IRQHandler → ICM_DMA_RxComplete_SPI4
 *   DMA2_Stream2_IRQHandler → ICM_DMA_RxComplete_SPI5
 *
 * Перед вызовом в ISR необходимо сбросить флаг TC:
 *   LL_DMA_ClearFlag_TC2(DMA1);  // для SPI1
 *   LL_DMA_ClearFlag_TC0(DMA2);  // для SPI4
 *   LL_DMA_ClearFlag_TC2(DMA2);  // для SPI5
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
