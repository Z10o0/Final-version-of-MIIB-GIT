/**
 * @file    icm45686_spi.h
 * @brief   Низкоуровневый SPI-транспорт для ICM-45686 на STM32H7.
 *          Все операции выполняются через LL-драйвер и DMA.
 *          HAL не используется.
 *
 *          Архитектура DMA-опроса:
 *          ┌─────────────────────────────────────────────────────┐
 *          │  TIM6 (прерывание каждые ~3.125 мс при 3200 Гц)     │
 *          │  → запускает ICM_StartBurstRead()                   │
 *          │  → DMA1 RX TC ISR → ICM_DMA_RxComplete_SPI1()       │
 *          │     (6 датчиков SPI1 по очереди, fault пропускаются) │
 *          │  → DMA2 RX TC ISR → ICM_DMA_RxComplete_SPI4()       │
 *          │     (6 датчиков SPI4 по очереди, fault пропускаются) │
 *          │  → DMA2 RX TC ISR → ICM_DMA_RxComplete_SPI5()       │
 *          │     (6 датчиков SPI5) → g_fifo_batch_ready           │
 *          └─────────────────────────────────────────────────────┘
 *
 *          Неисправные датчики (fault=1):
 *          - Пропускаются в DMA-каскаде (CS не активируется)
 *          - В UART-пакете их поля заполняются нулями (count=0)
 *          - Маска ошибок: g_sensor_fault_mask (бит N = датчик N неисправен)
 */

#ifndef ICM45686_SPI_H
#define ICM45686_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "main.h"
#include "icm45686_config.h"

/* ================================================================
 * Дескриптор одного датчика ICM-45686
 * ================================================================ */
typedef struct {
    SPI_TypeDef  *spi;           /* Указатель на периферию SPI */
    GPIO_TypeDef *cs_port;       /* GPIO-порт CS */
    uint32_t      cs_pin;        /* Пин CS (LL_GPIO_PIN_x) */
    uint8_t       sensor_id;     /* Глобальный номер датчика 0..17 */
    uint8_t       fault;         /* 0 = исправен, 1 = неисправен (нет ответа на WHO_AM_I) */
} ICM_Sensor_t;

/* ================================================================
 * Дескриптор шины SPI (6 датчиков на одном SPI)
 * ================================================================ */
typedef struct {
    SPI_TypeDef  *spi;                          /* Периферия SPI */
    DMA_TypeDef  *dma;                          /* DMA контроллер (DMA1 или DMA2) */
    uint32_t      dma_stream_rx;                /* Стрим RX (LL_DMA_STREAM_x) */
    uint32_t      dma_stream_tx;                /* Стрим TX (LL_DMA_STREAM_x) */
    ICM_Sensor_t  sensors[ICM_SENSORS_PER_BUS]; /* Датчики на этой шине */
    uint8_t       current_sensor_idx;           /* Текущий датчик в процессе опроса */

    /* TX-буфер: команда чтения FIFO + dummy-байты */
    uint8_t  tx_buf[ICM_FIFO_DMA_BUF_SIZE] __attribute__((aligned(4)));

    volatile uint8_t  transfer_complete;        /* Флаг завершения последнего датчика шины */
} ICM_Bus_t;

/* ================================================================
 * Глобальные дескрипторы трёх шин
 * ================================================================ */
extern ICM_Bus_t g_bus_spi1;  /* SPI1: датчики  0.. 5 */
extern ICM_Bus_t g_bus_spi4;  /* SPI4: датчики  6..11 */
extern ICM_Bus_t g_bus_spi5;  /* SPI5: датчики 12..17 */

/* ================================================================
 * Флаг готовности новой пачки данных (устанавливается в ISR)
 * ================================================================ */
extern volatile uint8_t g_fifo_batch_ready;  /* =1 когда все 3 шины завершили цикл */

/* ================================================================
 * Маска неисправных датчиков (бит N = датчик N неисправен).
 * Устанавливается один раз в ICM_InitAllSensors().
 * Используется в ICM_ParseAllFIFO() для записи нулей.
 * uint32_t достаточно для 18 датчиков (биты 0..17).
 * ================================================================ */
extern volatile uint32_t g_sensor_fault_mask;

/* ================================================================
 * Сырые данные FIFO: [шина 0=SPI1 / 1=SPI4 / 2=SPI5][датчик 0..5][байт]
 * ================================================================ */
extern uint8_t g_fifo_data[3][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE];

/* ================================================================
 * Публичные функции
 * ================================================================ */

/**
 * @brief  Инициализация структур шин и CS-пинов (CS -> HIGH).
 *         Сбрасывает все fault-флаги. Вызывать до ICM_InitAllSensors().
 */
void ICM_BusesInit(void);

/**
 * @brief  Инициализация всех 18 датчиков ICM-45686.
 *         При ошибке WHO_AM_I датчик помечается fault=1 и инициализация
 *         продолжается на следующем датчике. Система не останавливается.
 * @retval Маска неисправных датчиков (uint32_t, бит N = датчик N сбойный).
 *         0 = все датчики исправны.
 */
uint32_t ICM_InitAllSensors(void);

/**
 * @brief  Запуск DMA-чтения FIFO (единая точка входа из TIM6 ISR).
 *         Запускает SPI1, далее каскад продолжается через ISR.
 *         Неисправные датчики пропускаются автоматически.
 */
void ICM_StartBurstRead(void);

/**
 * @brief  Псевдоним ICM_StartBurstRead() для обратной совместимости.
 */
void ICM_StartBurstRead_SPI1(void);

/**
 * @brief  Вызывается из DMA1_Stream2 ISR (SPI1 RX TC).
 */
void ICM_DMA_RxComplete_SPI1(void);

/**
 * @brief  Вызывается из DMA2_Stream0 ISR (SPI4 RX TC).
 */
void ICM_DMA_RxComplete_SPI4(void);

/**
 * @brief  Вызывается из DMA2_Stream2 ISR (SPI5 RX TC).
 */
void ICM_DMA_RxComplete_SPI5(void);

/**
 * @brief  Запись одного регистра датчика (блокирующая, только для init).
 */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t val);

/**
 * @brief  Чтение одного регистра датчика (блокирующая, только для init).
 */
uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_SPI_H */
