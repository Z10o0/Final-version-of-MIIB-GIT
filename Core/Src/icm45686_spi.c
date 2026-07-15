/**
 * @file    icm45686_spi.c
 * @brief   Реализация DMA-транспорта для 18 датчиков ICM-45686 на STM32H723ZGT6.
 *
 * ──────────────────────────────────────────────────────────────
 *  АРХИТЕКТУРА КАСКАДНОГО DMA-ОПРОСА (3 шины SPI)
 * ──────────────────────────────────────────────────────────────
 *
 *  TIM6 UPDATE IRQ (каждые ~3.125 мс при ODR=3200 Гц, 10 пакетов)
 *     │
 *     ▼
 *  ICM_StartBurstRead()              ← единая точка входа
 *     │ запускает первый ИСПРАВНЫЙ датчик SPI1 (CS1..CS6)
 *     ▼
 *  DMA1_Stream2 TC ISR → ICM_DMA_RxComplete_SPI1()
 *     │ по очереди все 6 датчиков SPI1
 *     │ после последнего → запуск первого исправного SPI5
 *     ▼
 *  DMA2_Stream2 TC ISR → ICM_DMA_RxComplete_SPI5()
 *     │ по очереди все 6 датчиков SPI5 (CS7..CS12)
 *     │ после последнего → запуск первого исправного SPI4
 *     ▼
 *  DMA2_Stream0 TC ISR → ICM_DMA_RxComplete_SPI4()
 *     │ по очереди все 6 датчиков SPI4 (CS13..CS18)
 *     │ после последнего → g_fifo_batch_ready = 1
 *     ▼
 *  main-loop: g_fifo_batch_ready → ICM_ParseAllFIFO() → UART_SendBatch()
 *
 * ──────────────────────────────────────────────────────────────
 *  АКТУАЛЬНАЯ РАСПИНОВКА CS (версия 15.07.2026):
 *
 *  SPI1 (SCLK=PA5, MISO=PA6, MOSI=PA7) — DMA1 Stream2(RX)/Stream3(TX):
 *    sensor_id  0 → CS1  = PB12 = CS36_Pin / CS36_GPIO_Port
 *    sensor_id  1 → CS2  = PB13 = CS35_Pin / CS35_GPIO_Port
 *    sensor_id  2 → CS3  = PE8  = CS33_Pin / CS33_GPIO_Port
 *    sensor_id  3 → CS4  = PE9  = CS34_Pin / CS34_GPIO_Port
 *    sensor_id  4 → CS5  = PF13 = CS31_Pin / CS31_GPIO_Port
 *    sensor_id  5 → CS6  = PF14 = CS32_Pin / CS32_GPIO_Port
 *
 *  SPI5 (SCLK=PF7, MISO=PF8, MOSI=PF9) — DMA2 Stream2(RX)/Stream3(TX):
 *    sensor_id  6 → CS7  = PE14 = CS29_Pin / CS29_GPIO_Port
 *    sensor_id  7 → CS8  = PE15 = CS30_Pin / CS30_GPIO_Port
 *    sensor_id  8 → CS9  = PE7  = CS27_Pin / CS27_GPIO_Port
 *    sensor_id  9 → CS10 = PG1  = CS28_Pin / CS28_GPIO_Port
 *    sensor_id 10 → CS11 = PB0  = CS25_Pin / CS25_GPIO_Port
 *    sensor_id 11 → CS12 = PB1  = CS26_Pin / CS26_GPIO_Port
 *
 *  SPI4 (SCLK=PE2, MISO=PE5, MOSI=PE6) — DMA2 Stream0(RX)/Stream1(TX):
 *    sensor_id 12 → CS13 = PE10 = CS23_Pin / CS23_GPIO_Port
 *    sensor_id 13 → CS14 = PE11 = CS24_Pin / CS24_GPIO_Port
 *    sensor_id 14 → CS15 = PF15 = CS22_Pin / CS22_GPIO_Port
 *    sensor_id 15 → CS16 = PG0  = CS21_Pin / CS21_GPIO_Port
 *    sensor_id 16 → CS17 = PC4  = CS19_Pin / CS19_GPIO_Port
 *    sensor_id 17 → CS18 = PC5  = CS20_Pin / CS20_GPIO_Port
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
 * Индекс шины: SPI1=0, SPI5=1, SPI4=2

 * ================================================================ */
/* DMA-буферы — ТОЛЬКО в AXI SRAM (0x24000000), не в DTCM! */
/* DMA RX-буфер: ОБЯЗАТЕЛЬНО в AXI SRAM D1 (0x24000000).
 * aligned(32) — кратно cache-line для корректного SCB_CleanDCache. */
uint8_t g_fifo_data[3][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D1_noinit")))
    __attribute__((aligned(32)));

/* ================================================================
 * Флаг готовности пачки данных.
 * Устанавливается в ISR ICM_DMA_RxComplete_SPI4(), когда все
 * три шины завершили DMA-опрос за текущий тик TIM6.
 * Очищается в main-loop перед ICM_ParseAllFIFO().
 * volatile — доступ из ISR и main-loop.
 * ================================================================ */
volatile uint8_t g_fifo_batch_ready = 0U;

/* ================================================================
 * Маска неисправных датчиков.
 * Бит N = 1 если датчик N не ответил на WHO_AM_I или IREG-верификацию.
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
 * Порядок каскада DMA: SPI1 → SPI5 → SPI4
 * Индексы в g_fifo_data:  0      1      2
 *
 * ВАЖНО: макросы CSxx_Pin / CSxx_GPIO_Port генерируются CubeMX
 * в main.h. Имена макросов соответствуют меткам пинов в .ioc файле.
 * Не путать номер метки (CS19..CS36) с логическим номером датчика (CS1..CS18)!
 *
 * Соответствие: метка CubeMX → физический пин → логический CS
 *   CS36_Pin = PB12 → CS1   (sensor_id 0,  SPI1)
 *   CS35_Pin = PB13 → CS2   (sensor_id 1,  SPI1)
 *   CS33_Pin = PE8  → CS3   (sensor_id 2,  SPI1)
 *   CS34_Pin = PE9  → CS4   (sensor_id 3,  SPI1)
 *   CS31_Pin = PF13 → CS5   (sensor_id 4,  SPI1)
 *   CS32_Pin = PF14 → CS6   (sensor_id 5,  SPI1)
 *   CS29_Pin = PE14 → CS7   (sensor_id 6,  SPI5)
 *   CS30_Pin = PE15 → CS8   (sensor_id 7,  SPI5)
 *   CS27_Pin = PE7  → CS9   (sensor_id 8,  SPI5)
 *   CS28_Pin = PG1  → CS10  (sensor_id 9,  SPI5)
 *   CS25_Pin = PB0  → CS11  (sensor_id 10, SPI5)
 *   CS26_Pin = PB1  → CS12  (sensor_id 11, SPI5)
 *   CS23_Pin = PE10 → CS13  (sensor_id 12, SPI4)
 *   CS24_Pin = PE11 → CS14  (sensor_id 13, SPI4)
 *   CS22_Pin = PF15 → CS15  (sensor_id 14, SPI4)
 *   CS21_Pin = PG0  → CS16  (sensor_id 15, SPI4)
 *   CS19_Pin = PC4  → CS17  (sensor_id 16, SPI4)
 *   CS20_Pin = PC5  → CS18  (sensor_id 17, SPI4)
 * ================================================================ */

/* ── SPI1 (SCLK=PA5, MISO=PA6, MOSI=PA7) ── */
/* DMA1: RX=Stream2, TX=Stream3              */
/* __attribute__((section(".RAM_D1_noinit"))): структура лежит в AXI SRAM D1
 * (0x24000000), доступной DMA1/DMA2. Без этого — попадает в DTCM (0x20000000),
 * к которому DMA не имеет доступа, и tx_buf внутри структуры не передаётся. */
ICM_Bus_t g_bus_spi1
    __attribute__((section(".RAM_D1_noinit")))
    __attribute__((aligned(4)))
= {
    .spi           = SPI1,
    .dma           = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .sensors = {
        { SPI1, CS36_GPIO_Port, CS36_Pin,  0U, 0U }, /* PB12 — CS1  */
        { SPI1, CS35_GPIO_Port, CS35_Pin,  1U, 0U }, /* PB13 — CS2  */
        { SPI1, CS33_GPIO_Port, CS33_Pin,  2U, 0U }, /* PE8  — CS3  */
        { SPI1, CS34_GPIO_Port, CS34_Pin,  3U, 0U }, /* PE9  — CS4  */
        { SPI1, CS31_GPIO_Port, CS31_Pin,  4U, 0U }, /* PF13 — CS5  */
        { SPI1, CS32_GPIO_Port, CS32_Pin,  5U, 0U }, /* PF14 — CS6  */
    }
};

/* ── SPI5 (SCLK=PF7, MISO=PF8, MOSI=PF9) ── */
/* DMA2: RX=Stream2, TX=Stream3               */
ICM_Bus_t g_bus_spi5
    __attribute__((section(".RAM_D1_noinit")))
    __attribute__((aligned(4)))
= {
    .spi           = SPI5,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .sensors = {
        { SPI5, CS29_GPIO_Port, CS29_Pin,  6U, 0U }, /* PE14 — CS7  */
        { SPI5, CS30_GPIO_Port, CS30_Pin,  7U, 0U }, /* PE15 — CS8  */
        { SPI5, CS27_GPIO_Port, CS27_Pin,  8U, 0U }, /* PE7  — CS9  */
        { SPI5, CS28_GPIO_Port, CS28_Pin,  9U, 0U }, /* PG1  — CS10 */
        { SPI5, CS25_GPIO_Port, CS25_Pin, 10U, 0U }, /* PB0  — CS11 */
        { SPI5, CS26_GPIO_Port, CS26_Pin, 11U, 0U }, /* PB1  — CS12 */
    }
};

/* ── SPI4 (SCLK=PE2, MISO=PE5, MOSI=PE6) ── */
/* DMA2: RX=Stream0, TX=Stream1               */
ICM_Bus_t g_bus_spi4
    __attribute__((section(".RAM_D1_noinit")))
    __attribute__((aligned(4)))
= {
    .spi           = SPI4,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,
    .sensors = {
        { SPI4, CS23_GPIO_Port, CS23_Pin, 12U, 0U }, /* PE10 — CS13 */
        { SPI4, CS24_GPIO_Port, CS24_Pin, 13U, 0U }, /* PE11 — CS14 */
        { SPI4, CS22_GPIO_Port, CS22_Pin, 14U, 0U }, /* PF15 — CS15 */
        { SPI4, CS21_GPIO_Port, CS21_Pin, 15U, 0U }, /* PG0  — CS16 */
        { SPI4, CS19_GPIO_Port, CS19_Pin, 16U, 0U }, /* PC4  — CS17 */
        { SPI4, CS20_GPIO_Port, CS20_Pin, 17U, 0U }, /* PC5  — CS18 */
    }
};

/* ================================================================
 * Bus_GetIdx — индекс шины в массиве g_fifo_data.
 * SPI1=0, SPI5=1, SPI4=2.
 * Порядок соответствует каскаду: SPI1→SPI5→SPI4.
 * ================================================================ */
static uint8_t Bus_GetIdx(ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1) { return 0U; }
    if (bus == &g_bus_spi5) { return 1U; }
    return 2U; /* SPI4 */
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
 * SPI_WaitEOT — ожидание флага End Of Transfer (блокирующий режим).
 *
 * EOT устанавливается когда все байты TX отправлены И все RX получены.
 * Используется только в ICM_WriteReg / ICM_ReadReg.
 * В DMA-пути НЕ используется (там завершение фиксируется по TC ISR).
 * ================================================================ */
static inline void SPI_WaitEOT(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

/* ================================================================
 * SPI_EnableDMA — включение SPI в DMA-режиме.
 * Размер транзакции SetTransferSize() должен быть задан ДО вызова.
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
 * ВАЖНО: вызывать только из ISR-контекста, где DMA TC уже произошёл.
 * К моменту TC флаг EOT на SPI уже установлен — busy-wait не нужен.
 * Безопасно вызывать из ISR без задержек.
 * ================================================================ */
static inline void SPI_DisableDMA(SPI_TypeDef *spi)
{
    LL_SPI_DisableDMAReq_RX(spi);
    LL_SPI_DisableDMAReq_TX(spi);
    /* Очистить флаги завершения SPI-транзакции если установлены */
    if (LL_SPI_IsActiveFlag_EOT(spi))
    {
        LL_SPI_ClearFlag_EOT(spi);
        LL_SPI_ClearFlag_TXTF(spi);
    }
    LL_SPI_Disable(spi);
}

/* ================================================================
 * Delay_ms — программная задержка на основе убывающего счётчика SysTick.
 *
 * Используется только в ICM_InitAllSensors() — не в DMA-цикле.
 * SysTick->LOAD = SystemCoreClock/1000 - 1 при стандартной настройке CubeMX.
 * Обрабатывает переполнение VAL (wrap-around при каждом тике).
 * Не зависит от HAL uwTick и не требует разрешённых прерываний.
 * ================================================================ */
static void Delay_ms(uint32_t ms)
{
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
            elapsed += (prev_val - curr_val);               /* Обычный убывающий счёт */
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
        LL_GPIO_SetOutputPin(g_bus_spi5.sensors[s].cs_port,
                             g_bus_spi5.sensors[s].cs_pin);
        LL_GPIO_SetOutputPin(g_bus_spi4.sensors[s].cs_port,
                             g_bus_spi4.sensors[s].cs_pin);

        /* Сброс fault-флагов — все датчики считаются исправными до проверки */
        g_bus_spi1.sensors[s].fault = 0U;
        g_bus_spi5.sensors[s].fault = 0U;
        g_bus_spi4.sensors[s].fault = 0U;
    }

    /* ── Подготовить TX-буферы ── */
    /* Байт [0]: команда SPI burst-read FIFO: addr | READ_BIT */
    /* Байты [1..N]: 0xFF — dummy для генерации SCK при чтении RX */
    memset(g_bus_spi1.tx_buf, 0xFFU, sizeof(g_bus_spi1.tx_buf));
    memset(g_bus_spi5.tx_buf, 0xFFU, sizeof(g_bus_spi5.tx_buf));
    memset(g_bus_spi4.tx_buf, 0xFFU, sizeof(g_bus_spi4.tx_buf));

    g_bus_spi1.tx_buf[0] = (uint8_t)(ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT);
    g_bus_spi5.tx_buf[0] = (uint8_t)(ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT);
    g_bus_spi4.tx_buf[0] = (uint8_t)(ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT);

    /* ── Обнуление RX-буферов ── */
    /* Для fault-датчиков данные в этих буферах останутся нулями */
    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));

    /* ── Сброс управляющих переменных ── */
    g_fifo_batch_ready  = 0U;
    g_buses_done_cnt    = 0U;
    g_sensor_fault_mask = 0U;

    g_bus_spi1.current_sensor_idx = 0U;
    g_bus_spi5.current_sensor_idx = 0U;
    g_bus_spi4.current_sensor_idx = 0U;

    g_bus_spi1.transfer_complete  = 0U;
    g_bus_spi5.transfer_complete  = 0U;
    g_bus_spi4.transfer_complete  = 0U;
}

/* ================================================================
 * ICM_WriteReg — блокирующая запись регистра USER BANK 0.
 *
 * Правильный порядок для STM32H7 SPI (TSIZE mode):
 *   1. CS LOW
 *   2. SetTransferSize(2)
 *   3. Enable + StartMasterTransfer
 *   4. Передать addr_byte (бит7=0 → запись)
 *   5. Передать data_byte
 *   6. Ждать EOT
 *   7. ОБЯЗАТЕЛЬНО слить RX FIFO (2 dummy-байта)
 *   8. CS HIGH
 *   9. Disable
 *
 * Почему нужен шаг 7:
 *   SPI STM32H7 в Full-Duplex всегда принимает байт одновременно
 *   с передачей. При записи регистра принимаются 2 мусорных байта
 *   от датчика (MISO в это время тянется датчиком к питанию/земле).
 *   Если НЕ читать RX FIFO — через несколько вызовов WriteReg
 *   FIFO переполнится (глубина = 8 байт у H7), флаг OVR выставится,
 *   и следующий ICM_ReadReg вернёт мусор вместо реальных данных.
 *   Это именно то, что вызывало неправильный WHO_AM_I!
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

    /* 6. Ждать конца транзакции (все 2 байта переданы И приняты) */
    SPI_WaitEOT(spi);

    /* 7. СЛИТЬ RX FIFO — обязательный шаг для Full-Duplex STM32H7 SPI!
     *
     * За время передачи 2 байт SPI принял 2 мусорных байта с MISO.
     * Они сидят в RX FIFO и ДОЛЖНЫ быть прочитаны — иначе накапливаются
     * от вызова к вызову и через ~4 транзакции дают флаг OVR (overrun),
     * после чего ICM_ReadReg читает из FIFO старые мусорные данные.
     *
     * Ждём RXP (RX FIFO Not Empty) перед каждым чтением — на STM32H7
     * байт может появиться в FIFO чуть позже флага EOT (pipeline). */
    while ((LL_SPI_IsActiveFlag_RXP(spi)   == 0U) &&
           (LL_SPI_IsActiveFlag_RXWNE(spi) == 0U)) {}
    (void)LL_SPI_ReceiveData8(spi); /* dummy: ответ на адресный байт */

    while ((LL_SPI_IsActiveFlag_RXP(spi)   == 0U) &&
           (LL_SPI_IsActiveFlag_RXWNE(spi) == 0U)) {}
    (void)LL_SPI_ReceiveData8(spi); /* dummy: ответ на байт данных  */

    /* 8. CS HIGH — деактивировать датчик */
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* 9. Выключить SPI */
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
 *   7. Ждать RXP/RXWNE → читать 1-й байт (dummy)
 *   8. Ждать RXP/RXWNE → читать 2-й байт (реальные данные)
 *   9. CS HIGH
 *  10. Disable
 *
 * Почему нужно ждать RXP после EOT:
 *   На STM32H7 SPI-контроллер ставит флаг EOT когда последний бит
 *   ПЕРЕДАН, но последний принятый байт может ещё не попасть в
 *   RX FIFO из-за внутреннего pipeline. Чтение без ожидания RXP
 *   вернёт значение из предыдущей транзакции (старый мусор из FIFO)
 *   или 0x00 если FIFO пуст — именно это и давало неверный WHO_AM_I!
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

    /* 4. Передать адрес с битом READ (бит7=1) */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, (uint8_t)(reg | ICM45686_SPI_READ_BIT));

    /* 5. Dummy-байт: генерирует тактовые импульсы для приёма данных от датчика */
    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, 0xFFU);

    /* 6. Ждать конца транзакции (все байты TX отправлены, RX приняты) */
    SPI_WaitEOT(spi);

    /* 7. Ждать появления 1-го байта в RX FIFO и читать dummy
     *
     * RXP = "RX FIFO Not Empty" (есть хотя бы 1 байт для текущего DataWidth)
     * RXWNE = "RX Word Not Empty" (есть хотя бы 32-бит данных, независимо от DataWidth)
     * Проверяем оба флага — на 8-битном режиме STM32H7 иногда поднимает RXWNE
     * раньше чем RXP из-за особенностей внутреннего FIFO-пакетирования. */
    while ((LL_SPI_IsActiveFlag_RXP(spi)   == 0U) &&
           (LL_SPI_IsActiveFlag_RXWNE(spi) == 0U)) {}
    (void)LL_SPI_ReceiveData8(spi); /* dummy: ответ на адресный байт (MISO не несёт данных) */

    /* 8. Ждать появления 2-го байта и читать реальные данные регистра */
    while ((LL_SPI_IsActiveFlag_RXP(spi)   == 0U) &&
           (LL_SPI_IsActiveFlag_RXWNE(spi) == 0U)) {}
    result = LL_SPI_ReceiveData8(spi); /* реальное значение регистра */

    /* 9. CS HIGH */
    LL_GPIO_SetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* 10. Выключить SPI */
    LL_SPI_Disable(spi);

    return result;
}

/* ================================================================
 * ICM_WriteIReg — блокирующая запись внутреннего IREG-регистра.
 *
 * Используется для настройки CLKIN через IPREG_TOP1:
 *   ICM_WriteIReg(sensor, 0xA4, 0x30, 0x06) → IOC_PAD_SCENARIO_OVRD
 *   ICM_WriteIReg(sensor, 0xA4, 0x58, 0x42) → SMC_CONTROL_0
 *
 * Трёхшаговая процедура косвенного доступа:
 *   1. Записать старший байт IREG-адреса в IREG_ADDR_15_8
 *   2. Записать младший байт IREG-адреса в IREG_ADDR_7_0
 *   3. Записать данные в IREG_DATA
 *   4. Пауза >= 10 мкс (аппаратура завершает транзакцию к IREG)
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

    /* Шаг 3: пауза перед чтением (аппаратура завершает адресацию) */
    Delay_ms(ICM45686_IREG_DELAY_MS);

    /* Шаг 4: прочитать результат из IREG_DATA */
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
 *   5.  Верификация IREG-записи → при ошибке: fault=1, continue
 *   6.  IREG: SMC_CONTROL_0 = 0x42          (RTC_MODE + TMST_EN)
 *   7.  ACCEL_CONFIG0: FS + ODR из icm45686_config.h
 *   8.  GYRO_CONFIG0:  FS + ODR из icm45686_config.h
 *   9.  FIFO_CONFIG0:  STREAM, gyro+accel+temp+timestamp
 *   10. FIFO watermark = ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES
 *   11. PWR_MGMT0: gyro LN + accel LN
 *   12. Задержка 200 мс (прогрев гироскопа до стабильных данных)
 *
 * Порядок обхода шин совпадает с каскадом DMA: SPI1 → SPI5 → SPI4.
 * Неисправные датчики НЕ останавливают процедуру.
 * Возвращает маску: бит N = 1 если датчик N неисправен.
 * ================================================================ */
uint32_t ICM_InitAllSensors(void)
{
    /* Порядок обхода совпадает с порядком каскада DMA */
    ICM_Bus_t   *buses[3] = { &g_bus_spi1, &g_bus_spi5, &g_bus_spi4 };
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
                /* Датчик не отвечает или неисправен — пропускаем */
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

            /* ── Шаг 5: верификация CLKIN ── */
            ireg_check = ICM_ReadIReg(sensor,
                                      ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_H,
                                      ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L);
            if (ireg_check != ICM45686_CLKIN_ENABLE_VAL)
            {
                /* IREG-запись не прошла — датчик неисправен */
                sensor->fault        = 1U;
                g_sensor_fault_mask |= (1UL << sensor->sensor_id);
                continue;
            }

            /* ── Шаг 6: включение RTC_MODE + TMST_EN через IREG ──
             * SMC_CONTROL_0 @ 0xA458:
             *   бит[6]=1 (RTC_MODE): ODR тактируется от внешнего CLKIN
             *   бит[1]=1 (TMST_EN):  включить timestamp в FIFO-пакет */
            ICM_WriteIReg(sensor,
                          ICM45686_IREG_SMC_CONTROL_0_H,
                          ICM45686_IREG_SMC_CONTROL_0_L,
                          ICM45686_RTC_MODE_TMST_ENABLE_VAL);

            /* ── Шаг 7: конфигурация акселерометра ──
             * Биты [7:5] = FS, биты [3:0] = ODR
             * Значения задаются в icm45686_config.h */
            ICM_WriteReg(sensor, ICM45686_REG_ACCEL_CONFIG0,
                         (uint8_t)(ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE));

            /* ── Шаг 8: конфигурация гироскопа ──
             * Биты [7:4] = FS, биты [3:0] = ODR */
            ICM_WriteReg(sensor, ICM45686_REG_GYRO_CONFIG0,
                         (uint8_t)(ICM_GYRO_FS_VALUE | ICM_GYRO_ODR_VALUE));

            /* ── Шаг 9: настройка FIFO ──
             * STREAM-режим: новые данные вытесняют старые при переполнении.
             * Включаем: гироскоп + акселерометр + температура + timestamp */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG0,
                         (uint8_t)(ICM45686_FIFO_MODE_STREAM |
                                   ICM45686_FIFO_SEL_GYRO    |
                                   ICM45686_FIFO_SEL_ACCEL   |
                                   ICM45686_FIFO_SEL_TEMP    |
                                   ICM45686_FIFO_SEL_TMST));

            /* ── Шаг 10: порог FIFO (watermark) ──
             * Порог = количество пакетов × размер одного пакета в байтах.
             * Двухбайтовая запись: [0] = LSB, [1] = MSB[3:0] */
            {
                uint16_t wm = (uint16_t)(ICM_FIFO_POLL_PACKETS *
                                         ICM_FIFO_PACKET_BYTES);
                ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_0,
                             (uint8_t)(wm & 0x00FFU));
                ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_1,
                             (uint8_t)((wm >> 8U) & 0x000FU));
            }

            /* ── Шаг 11: включение питания ──
             * Гироскоп: Low-Noise mode (высшая точность)
             * Акселерометр: Low-Noise mode */
            ICM_WriteReg(sensor, ICM45686_REG_PWR_MGMT0,
                         (uint8_t)(ICM45686_PWR_GYRO_MODE_LN |
                                   ICM45686_PWR_ACCEL_MODE_LN));

            /* ── Шаг 12: прогрев гироскопа ──
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
 *   2. Дождаться физического останова стримов
 *   3. Сбросить флаги TC/TE/HT для обоих стримов
 *   4. Настроить адреса памяти/периферии и длину для RX и TX
 *   5. Разрешить TC-прерывание только для RX-стрима
 *   6. CS LOW
 *   7. SetTransferSize SPI
 *   8. Запустить RX-стрим, затем TX-стрим, затем SPI
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

    /* ── 1-2. Остановить стримы и дождаться физического останова ── */
    LL_DMA_DisableStream(dma, rx_st);
    LL_DMA_DisableStream(dma, tx_st);
    while (LL_DMA_IsEnabledStream(dma, rx_st) != 0U) {}
    while (LL_DMA_IsEnabledStream(dma, tx_st) != 0U) {}

    /* ── 3. Сброс флагов TC, TE, HT ──
     * Необходимо сбрасывать ВСЕ флаги перед повторным запуском,
     * иначе ISR сработает немедленно по старому флагу. */
    if (dma == DMA1)
    {
        /* SPI1: RX=Stream2, TX=Stream3 */
        LL_DMA_ClearFlag_TC2(DMA1);
        LL_DMA_ClearFlag_TE2(DMA1);
        LL_DMA_ClearFlag_HT2(DMA1);
        LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1);
        LL_DMA_ClearFlag_HT3(DMA1);
    }
    else /* DMA2 — используется SPI5 и SPI4 */
    {
        if (rx_st == LL_DMA_STREAM_2)   /* SPI5: RX=Stream2, TX=Stream3 */
        {
            LL_DMA_ClearFlag_TC2(DMA2);
            LL_DMA_ClearFlag_TE2(DMA2);
            LL_DMA_ClearFlag_HT2(DMA2);
            LL_DMA_ClearFlag_TC3(DMA2);
            LL_DMA_ClearFlag_TE3(DMA2);
            LL_DMA_ClearFlag_HT3(DMA2);
        }
        else                            /* SPI4: RX=Stream0, TX=Stream1 */
        {
            LL_DMA_ClearFlag_TC0(DMA2);
            LL_DMA_ClearFlag_TE0(DMA2);
            LL_DMA_ClearFlag_HT0(DMA2);
            LL_DMA_ClearFlag_TC1(DMA2);
            LL_DMA_ClearFlag_TE1(DMA2);
            LL_DMA_ClearFlag_HT1(DMA2);
        }
    }

    /* ── 4. Настройка адресов и длины ── */
    /* RX: данные из SPI DR → g_fifo_data[bus_idx][sensor_idx] */
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

    /* ── 5. Разрешить TC-прерывание только для RX ──
     * TX прерывание не нужно: завершение фиксируется по RX TC */
    LL_DMA_EnableIT_TC(dma, rx_st);

    /* ── 6. CS LOW — активировать датчик ── */
    LL_GPIO_ResetOutputPin(sensor->cs_port, sensor->cs_pin);

    /* ── 7. Задать размер SPI-транзакции ── */
    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);

    /* ── 8. Запуск: RX-стрим → TX-стрим → SPI ──
     * Порядок критичен: RX должен быть готов принимать до первого SCK */
    LL_DMA_EnableStream(dma, rx_st);
    LL_DMA_EnableStream(dma, tx_st);
    SPI_EnableDMA(bus->spi);
}

/* ================================================================
 * BusFinalize — завершение шины и запуск следующей в каскаде.
 *
 * Вызывается из BusBurstRead_Next() когда на текущей шине
 * больше нет исправных датчиков для опроса.
 *
 * Каскад: SPI1 → SPI5 → SPI4 → g_fifo_batch_ready = 1
 * ================================================================ */
static void BusFinalize(ICM_Bus_t *bus)
{
    uint8_t first;
    uint8_t cnt;

    /* Отметить шину как завершённую */
    bus->transfer_complete  = 1U;
    bus->current_sensor_idx = 0U;

    /* Инкремент счётчика завершённых шин (только из ISR — атомарность не нужна) */
    cnt = ++g_buses_done_cnt;

    if (bus == &g_bus_spi1)
    {
        /* SPI1 завершена → запуск SPI5 */
        first = Bus_FindNextOK(&g_bus_spi5, 0U);
        if (first < ICM_SENSORS_PER_BUS)
        {
            g_bus_spi5.current_sensor_idx = first;
            BusBurstRead_Start(&g_bus_spi5, first);
        }
        else
        {
            /* Все датчики SPI5 неисправны — пропускаем шину */
            BusFinalize(&g_bus_spi5);
        }
    }
    else if (bus == &g_bus_spi5)
    {
        /* SPI5 завершена → запуск SPI4 */
        first = Bus_FindNextOK(&g_bus_spi4, 0U);
        if (first < ICM_SENSORS_PER_BUS)
        {
            g_bus_spi4.current_sensor_idx = first;
            BusBurstRead_Start(&g_bus_spi4, first);
        }
        else
        {
            /* Все датчики SPI4 неисправны — пропускаем шину */
            BusFinalize(&g_bus_spi4);
        }
    }
    else /* bus == &g_bus_spi4 — последняя в каскаде */
    {
        /* SPI4 завершена — все три шины опрошены */
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
 * Вызывается из ICM_DMA_RxComplete_SPI1/5/4().
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

    /* ── Найти следующий исправный датчик на этой же шине ── */
    next = Bus_FindNextOK(bus, (uint8_t)(prev_idx + 1U));

    if (next < ICM_SENSORS_PER_BUS)
    {
        /* Есть ещё исправные датчики — продолжаем каскад на той же шине */
        bus->current_sensor_idx = next;
        BusBurstRead_Start(bus, next);
    }
    else
    {
        /* Все датчики шины опрошены (или пропущены) — финализация шины */
        BusFinalize(bus);
    }
}

/* ================================================================
 * ICM_StartBurstRead — единая точка входа из TIM6 UPDATE ISR.
 *
 * Запускает новый цикл DMA-опроса (каскад SPI1 → SPI5 → SPI4).
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
    g_bus_spi5.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;
    g_buses_done_cnt             = 0U;

    /* Запустить первый исправный датчик SPI1 (начало каскада) */
    first = Bus_FindNextOK(&g_bus_spi1, 0U);
    if (first < ICM_SENSORS_PER_BUS)
    {
        g_bus_spi1.current_sensor_idx = first;
        BusBurstRead_Start(&g_bus_spi1, first);
    }
    else
    {
        /* Все датчики SPI1 неисправны — запустить каскад с SPI5 */
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
 *
 * Порядок вызовов соответствует каскаду: SPI1 → SPI5 → SPI4.
 *
 * Вызываются из stm32h7xx_it.c:
 *   DMA1_Stream2_IRQHandler → ICM_DMA_RxComplete_SPI1()
 *   DMA2_Stream2_IRQHandler → ICM_DMA_RxComplete_SPI5()
 *   DMA2_Stream0_IRQHandler → ICM_DMA_RxComplete_SPI4()
 *
 * Шаблон stm32h7xx_it.c:
 *
 *   void DMA1_Stream2_IRQHandler(void) {
 *       if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
 *           LL_DMA_ClearFlag_TC2(DMA1);
 *           ICM_DMA_RxComplete_SPI1();
 *       }
 *   }
 *   void DMA2_Stream2_IRQHandler(void) {
 *       if (LL_DMA_IsActiveFlag_TC2(DMA2)) {
 *           LL_DMA_ClearFlag_TC2(DMA2);
 *           ICM_DMA_RxComplete_SPI5();
 *       }
 *   }
 *   void DMA2_Stream0_IRQHandler(void) {
 *       if (LL_DMA_IsActiveFlag_TC0(DMA2)) {
 *           LL_DMA_ClearFlag_TC0(DMA2);
 *           ICM_DMA_RxComplete_SPI4();
 *       }
 *   }
 * ================================================================ */

void ICM_DMA_RxComplete_SPI1(void)
{
    BusBurstRead_Next(&g_bus_spi1, g_bus_spi1.current_sensor_idx);
}

void ICM_DMA_RxComplete_SPI5(void)
{
    BusBurstRead_Next(&g_bus_spi5, g_bus_spi5.current_sensor_idx);
}

void ICM_DMA_RxComplete_SPI4(void)
{
    BusBurstRead_Next(&g_bus_spi4, g_bus_spi4.current_sensor_idx);
}
