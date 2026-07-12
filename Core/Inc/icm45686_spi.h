/**
 * @file    icm45686_spi.h
 * @brief   Низкоуровневый SPI-транспорт для ICM-45686 на STM32H7.
 *          Все операции выполняются через LL-драйвер и DMA.
 *          HAL не используется.
 *
 *          Архитектура DMA-опроса:
 *          ┌─────────────────────────────────────────────────────┐
 *          │  TIM6 (прерывание каждые ~3.125 мс при 3200 Гц)     │
 *          │  → запускает ICM_StartBurstRead_SPI1()              │
 *          │  → DMA1 RX TC ISR → запускает SPI4                  │
 *          │  → DMA2 RX TC ISR (SPI4) → запускает SPI5           │
 *          │  → DMA2 RX TC ISR (SPI5) → вызывает обработчик данных│
 *          └─────────────────────────────────────────────────────┘
 *
 *          На каждой шине 6 датчиков опрашиваются ПОСЛЕДОВАТЕЛЬНО
 *          (один за другим с переключением CS), так как SPI-шина одна.
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

    /* RX-буфер не используется напрямую — данные идут в g_fifo_data */
    volatile uint8_t  transfer_complete;        /* Флаг завершения последнего датчика шины */
} ICM_Bus_t;

/* ================================================================
 * Глобальные дескрипторы трёх шин
 * ================================================================ */
extern ICM_Bus_t g_bus_spi1;  /* SPI1: датчики 13..18 (индексы 0..5)  */
extern ICM_Bus_t g_bus_spi4;  /* SPI4: датчики  1.. 6 (индексы 0..5)  */
extern ICM_Bus_t g_bus_spi5;  /* SPI5: датчики  7..12 (индексы 0..5)  */

/* ================================================================
 * Флаг готовности новой пачки данных (устанавливается в ISR)
 * ================================================================ */
extern volatile uint8_t g_fifo_batch_ready;  /* =1 когда все 3 шины завершили цикл */

/* ================================================================
 * Сырые данные FIFO: [шина][датчик на шине][байт]
 * bus_idx: 0=SPI1, 1=SPI4, 2=SPI5
 * ================================================================ */
extern uint8_t g_fifo_data[3][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE];

/* ================================================================
 * Публичные функции
 * ================================================================ */

/**
 * @brief  Инициализация структур шин и CS-пинов (CS -> HIGH).
 *         Вызывать один раз до ICM_InitAllSensors().
 */
void ICM_BusesInit(void);

/**
 * @brief  Инициализация всех 18 датчиков ICM-45686.
 *         Выполняет: сброс, WHO_AM_I, конфигурацию PWR/ODR/FS/FIFO.
 * @retval 0 — успех, ненулевое — номер датчика, на котором сбой
 */
uint8_t ICM_InitAllSensors(void);

/**
 * @brief  Запуск DMA-чтения FIFO на шине SPI1 (первый шаг каскада).
 *         Вызывать из TIM6 ISR каждые 3.125 мс.
 */
void ICM_StartBurstRead_SPI1(void);

/**
 * @brief  Вызывается из DMA-ISR SPI1 RX TC — запускает следующий датчик
 *         SPI1 или переходит к SPI4.
 */
void ICM_DMA_RxComplete_SPI1(void);

/**
 * @brief  Вызывается из DMA-ISR SPI4 RX TC.
 */
void ICM_DMA_RxComplete_SPI4(void);

/**
 * @brief  Вызывается из DMA-ISR SPI5 RX TC.
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
