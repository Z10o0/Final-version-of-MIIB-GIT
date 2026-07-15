/**
 * @file    icm45686_spi.h
 * @brief   Низкоуровневый SPI-транспорт для 18 датчиков ICM-45686 на STM32H723ZGT6.
 *          Все операции выполняются через LL-драйвер и DMA. HAL не используется.
 *
 * ──────────────────────────────────────────────────────────────
 *  АРХИТЕКТУРА DMA-ОПРОСА (18 датчиков, 3 шины SPI)
 * ──────────────────────────────────────────────────────────────
 *
 *  TIM6 UPDATE IRQ (каждые ~3.125 мс при ODR=3200 Гц, ~1.5625 мс при 6400 Гц)
 *     │
 *     ▼
 *  ICM_StartBurstRead()              ← единая точка входа из TIM6 ISR
 *     │ запускает первый исправный датчик SPI1 (датчики CS13..CS18)
 *     ▼
 *  DMA1_Stream2 TC ISR → ICM_DMA_RxComplete_SPI1()
 *     │ по очереди все 6 датчиков SPI1 → запуск первого исправного SPI4
 *     ▼
 *  DMA2_Stream0 TC ISR → ICM_DMA_RxComplete_SPI4()
 *     │ по очереди все 6 датчиков SPI4 (CS1..CS6) → запуск SPI5
 *     ▼
 *  DMA2_Stream2 TC ISR → ICM_DMA_RxComplete_SPI5()
 *     │ по очереди все 6 датчиков SPI5 (CS7..CS12) → g_fifo_batch_ready = 1
 *     ▼
 *  main-loop: проверка g_fifo_batch_ready → ICM_ParseAllFIFO() → UART_SendBatch()
 *
 * ──────────────────────────────────────────────────────────────
 *  РАСПРЕДЕЛЕНИЕ ДАТЧИКОВ ПО ШИНАМ:
 *    SPI1 (DMA1):  CS13=PF13, CS14=PF14, CS15=PE8,  CS16=PE9,
 *                  CS17=PB13, CS18=PB12  (sensor_id 0..5)
 *    SPI4 (DMA2):  CS1=PC4,   CS2=PC5,   CS3=PG0,   CS4=PF15,
 *                  CS5=PE10,  CS6=PE11   (sensor_id 6..11)
 *    SPI5 (DMA2):  CS7=PB0,   CS8=PB1,   CS9=PE7,   CS10=PG1,
 *                  CS11=PE14, CS12=PE15  (sensor_id 12..17)
 *
 * ──────────────────────────────────────────────────────────────
 *  ПОВЕДЕНИЕ НЕИСПРАВНЫХ ДАТЧИКОВ (fault = 1):
 *    - Полностью пропускаются в DMA-каскаде (CS не трогается)
 *    - Буфер g_fifo_data для неисправного датчика сохраняет нули (memset при старте)
 *    - ICM_ParseAllFIFO() записывает нули в g_sensor_batches для fault-датчика
 *    - В UART-пакете поля fault-датчиков заполняются нулями
 *    - Бит N в g_sensor_fault_mask = 1 если датчик N неисправен
 *
 * ──────────────────────────────────────────────────────────────
 *  ОГРАНИЧЕНИЯ ПАМЯТИ:
 *    Буферы g_fifo_data размещены в SRAM AXI (D2) — доступно для DMA1 и DMA2.
 *    DMA1/DMA2 НЕ имеют доступа к ITCM/DTCM (0x00000000 и 0x20000000)!
 *    Буферы tx_buf внутри ICM_Bus_t также должны быть в AXI SRAM.
 *    Выравнивание по 4 байта (атрибут aligned(4)) обязательно.
 *
 * ──────────────────────────────────────────────────────────────
 *  ИНИЦИАЛИЗАЦИЯ С CLKIN:
 *    Внешнее тактирование MEMS-датчиков от единого кварцевого генератора
 *    (32.768 кГц) обеспечивает идентичный опорный период ODR
 *    для всех 18 датчиков — обязательное условие для
 *    coherent beamforming и межсенсорной синхронизации.
 *
 *    Порядок активации CLKIN в ICM_InitAllSensors() для каждого датчика:
 *      1. Программный сброс + задержка
 *      2. Проверка WHO_AM_I
 *      3. Выбор банка 0
 *      4. ICM_WriteIReg(IOC_PAD_SCENARIO_OVRD, 0x06)  ← CLKIN enable
 *      5. ICM_WriteIReg(SMC_CONTROL_0, 0x42)          ← RTC_MODE + TMST_EN
 *      6. ACCEL_CONFIG0 / GYRO_CONFIG0 / FIFO_CONFIG0 / watermark
 *      7. PWR_MGMT0 (gyro LN + accel LN)
 *      8. Задержка 200 мс
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
#include "icm45686_regs.h"    /* добавлен: нужен для ICM45686_IREG_* констант */

/* ================================================================
 * Дескриптор одного датчика ICM-45686
 *
 * Хранит всю периферийную информацию, необходимую для
 * адресации CS-пина и SPI-шины для данного датчика.
 * ================================================================ */
typedef struct
{
    SPI_TypeDef  *spi;       /**< Указатель на периферию SPI (SPI1, SPI4 или SPI5) */
    GPIO_TypeDef *cs_port;   /**< GPIO-порт пина CS датчика */
    uint32_t      cs_pin;    /**< Номер пина CS в формате LL_GPIO_PIN_x */
    uint8_t       sensor_id; /**< Глобальный номер датчика 0..17
                               *   0..5  = SPI1 (CS13..CS18)
                               *   6..11 = SPI4 (CS1..CS6)
                               *  12..17 = SPI5 (CS7..CS12) */
    uint8_t       fault;     /**< Статус датчика:
                               *  0 = датчик исправен (прошёл WHO_AM_I)
                               *  1 = датчик неисправен (запись 0 в UART-пакет) */
} ICM_Sensor_t;

/* ================================================================
 * Дескриптор шины SPI (6 датчиков на одном SPI)
 *
 * Содержит всю информацию для управления DMA-каскадом
 * для одной SPI-шины.
 * ================================================================ */
typedef struct
{
    SPI_TypeDef  *spi;            /**< Периферия SPI (SPI1, SPI4 или SPI5) */
    DMA_TypeDef  *dma;            /**< DMA-контроллер:
                                   *   SPI1 → DMA1, SPI4/SPI5 → DMA2 */
    uint32_t      dma_stream_rx;  /**< Номер RX-стрима в формате LL_DMA_STREAM_x:
                                   *   SPI1 → DMA1 Stream2
                                   *   SPI4 → DMA2 Stream0
                                   *   SPI5 → DMA2 Stream2 */
    uint32_t      dma_stream_tx;  /**< Номер TX-стрима в формате LL_DMA_STREAM_x:
                                   *   SPI1 → DMA1 Stream3
                                   *   SPI4 → DMA2 Stream1
                                   *   SPI5 → DMA2 Stream3 */
    ICM_Sensor_t  sensors[ICM_SENSORS_PER_BUS]; /**< Массив датчиков данной шины */
    uint8_t       current_sensor_idx; /**< Индекс датчика, опрашиваемого в данный момент.
                                       *   Устанавливается в BusBurstRead_Start и BusBurstRead_Next */

    /**
     * TX-буфер: первый байт = команда чтения FIFO (0x80 | 0x14),
     * остальные = 0xFF dummy-байты для генерации тактовых импульсов.
     * Выравнивание по 4 байта обязательно для правильной работы DMA.
     * Буфер должен находиться в SRAM AXI (D2), а НЕ в DTCM/ITCM.
     */
    uint8_t  tx_buf[ICM_FIFO_DMA_BUF_SIZE] __attribute__((aligned(4)));

    volatile uint8_t  transfer_complete; /**< Флаг завершения опроса всех датчиков шины.
                                          *   0 = данные ещё читаются
                                          *   1 = все датчики шины опрошены (или пропущены) */
} ICM_Bus_t;

/* ================================================================
 * Глобальные дескрипторы трёх шин
 * Определены в icm45686_spi.c, используются в main.c и stm32h7xx_it.c.
 * ================================================================ */
extern ICM_Bus_t g_bus_spi1;  /**< SPI1: датчики CS13..CS18 (sensor_id  0..5)  */
extern ICM_Bus_t g_bus_spi4;  /**< SPI4: датчики CS1..CS6   (sensor_id  6..11) */
extern ICM_Bus_t g_bus_spi5;  /**< SPI5: датчики CS7..CS12  (sensor_id 12..17) */

/* ================================================================
 * Глобальные флаги и буферы (определены в icm45686_spi.c)
 * ================================================================ */

/**
 * g_fifo_batch_ready — флаг готовности пачки данных.
 *
 * Устанавливается в ISR ICM_DMA_RxComplete_SPI5() после того,
 * как все три шины завершили DMA-опрос.
 * Очищается в main-loop перед ICM_ParseAllFIFO().
 * volatile т.к. разделяется между ISR и main-loop.
 */
extern volatile uint8_t g_fifo_batch_ready;

/**
 * g_sensor_fault_mask — маска неисправных датчиков.
 *
 * Бит N = 1 если датчик N не ответил на WHO_AM_I при инициализации.
 * Устанавливается один раз в ICM_InitAllSensors(), затем не изменяется.
 * uint32_t достаточно для 18 датчиков (биты 0..17).
 * 0x00000000 = все датчики исправны.
 */
extern volatile uint32_t g_sensor_fault_mask;

/**
 * g_fifo_data — сырые RX-буферы FIFO.
 *
 * Индексация: [bus_idx][sensor_on_bus][byte]
 *   bus_idx 0 = SPI1 (датчики CS13..CS18)
 *   bus_idx 1 = SPI4 (датчики CS1..CS6)
 *   bus_idx 2 = SPI5 (датчики CS7..CS12)
 *
 * Размещать в SRAM AXI (D2) — единственная память,
 * доступная одновременно DMA1 и DMA2.
 */
extern uint8_t g_fifo_data[3][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE];

/* ================================================================
 * ПУБЛИЧНЫЕ ФУНКЦИИ
 * ================================================================ */

/**
 * @brief  Инициализация структур дескрипторов шин и CS-пинов.
 *
 *         Взводит все CS-пины в HIGH (неактивное состояние).
 *         Сбрасывает fault-флаги всех датчиков.
 *         Заполняет TX-буферы: байт[0] = команда FIFO burst read,
 *         байты[1..N] = 0xFF dummy.
 *         Обнуляет g_fifo_data.
 *
 * @note   Вызывать ДО ICM_InitAllSensors().
 */
void ICM_BusesInit(void);

/**
 * @brief  Инициализация всех 18 датчиков ICM-45686.
 *
 *         Последовательность для каждого датчика:
 *           1. Программный сброс + задержка
 *           2. Проверка WHO_AM_I (0xE9)
 *           3. Выбор банка 0
 *           4. Активация CLKIN: ICM_WriteIReg(IOC_PAD_SCENARIO_OVRD, 0x06)
 *           5. Активация RTC_MODE: ICM_WriteIReg(SMC_CONTROL_0, 0x42)
 *           6. ACCEL_CONFIG0, GYRO_CONFIG0, FIFO_CONFIG0, watermark
 *           7. PWR_MGMT0 (gyro LN + accel LN)
 *           8. Задержка 200 мс
 *
 *         При ошибке WHO_AM_I датчик помечается fault=1,
 *         инициализация продолжается на следующем.
 *
 * @retval Маска неисправных датчиков (uint32_t, бит N = датчик N сбойный).
 *         0x00000000 = все датчики исправны.
 */
uint32_t ICM_InitAllSensors(void);

/**
 * @brief  Запуск DMA-чтения FIFO для всех трёх шин.
 *
 *         Единая точка входа для одного цикла опроса.
 *         Вызывается исключительно из TIM6 UPDATE ISR.
 *         Запускает DMA для SPI1, далее каскад продолжается через ISR.
 *         Неисправные датчики пропускаются автоматически.
 *
 * @note   Если g_fifo_batch_ready != 0 (предыдущий цикл не обработан),
 *         функция возвращается без запуска (защита от перетекания).
 */
void ICM_StartBurstRead(void);

/**
 * @brief  Псевдоним ICM_StartBurstRead() для обратной совместимости.
 *         В новом коде использовать ICM_StartBurstRead().
 */
void ICM_StartBurstRead_SPI1(void);

/**
 * @brief  Обработчик завершения DMA-передачи SPI1.
 *
 *         Вызывается из DMA1_Stream2_IRQHandler() в stm32h7xx_it.c.
 *         Переходит к следующему датчику SPI1 или запускает SPI4.
 *
 * @note   Работает в контексте ISR — не вызывать из main-уровня.
 */
void ICM_DMA_RxComplete_SPI1(void);

/**
 * @brief  Обработчик завершения DMA-передачи SPI4.
 *
 *         Вызывается из DMA2_Stream0_IRQHandler() в stm32h7xx_it.c.
 *         Переходит к следующему датчику SPI4 или запускает SPI5.
 *
 * @note   Работает в контексте ISR — не вызывать из main-уровня.
 */
void ICM_DMA_RxComplete_SPI4(void);

/**
 * @brief  Обработчик завершения DMA-передачи SPI5.
 *
 *         Вызывается из DMA2_Stream2_IRQHandler() в stm32h7xx_it.c.
 *         После последнего датчика SPI5 устанавливает g_fifo_batch_ready = 1.
 *
 * @note   Работает в контексте ISR — не вызывать из main-уровня.
 */
void ICM_DMA_RxComplete_SPI5(void);

/**
 * @brief  Блокирующая запись одного регистра датчика (USER BANK 0).
 *
 *         Используется только в ICM_InitAllSensors() и ICM_WriteIReg().
 *         В реальном времени (DMA-цикл) вызывать нельзя.
 *
 * @param  sensor  Указатель на дескриптор датчика
 * @param  reg     Адрес регистра USER BANK 0 (0x00..0x7F)
 * @param  val     Значение для записи
 */
void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t val);

/**
 * @brief  Блокирующее чтение одного регистра датчика (USER BANK 0).
 *
 *         Используется только в ICM_InitAllSensors().
 *         В реальном времени (DMA-цикл) вызывать нельзя.
 *
 * @param  sensor  Указатель на дескриптор датчика
 * @param  reg     Адрес регистра USER BANK 0 (0x00..0x7F)
 * @retval Прочитанное значение регистра
 */
uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg);

/**
 * @brief  Блокирующая запись внутреннего регистра IREG (IPREG_TOP1/SYS1/SYS2).
 *
 *         Используется для настройки CLKIN (IOC_PAD_SCENARIO_OVRD)
 *         и RTC_MODE (SMC_CONTROL_0) через косвенный доступ к IPREG_TOP1.
 *
 *         Внутренняя процедура:
 *           1. ICM_WriteReg(IREG_ADDR_15_8, addr_h)
 *           2. ICM_WriteReg(IREG_ADDR_7_0,  addr_l)
 *           3. ICM_WriteReg(IREG_DATA,       val)
 *           4. Delay_ms(ICM45686_IREG_DELAY_MS)  (пауза >= 10 мкс)
 *
 * @param  sensor  Указатель на дескриптор датчика
 * @param  addr_h  Старший байт 16-битного IREG-адреса (0xA4 для IPREG_TOP1)
 * @param  addr_l  Младший байт 16-битного IREG-адреса (0x30, 0x58 ...)
 * @param  val     Значение для записи
 *
 * @note   Использовать только в ICM_InitAllSensors() (не в DMA-цикле).
 * @note   До вызова убедиться, что SPI находится в банке 0 (BANK_SEL = 0x00).
 */
void ICM_WriteIReg(ICM_Sensor_t *sensor, uint8_t addr_h, uint8_t addr_l, uint8_t val);

/**
 * @brief  Блокирующее чтение внутреннего регистра IREG (IPREG_TOP1/SYS1/SYS2).
 *
 *         Аналогично ICM_WriteIReg(), но читает данные из IREG_DATA.
 *         Полезен для верификации значений IOC_PAD_SCENARIO_OVRD
 *         и SMC_CONTROL_0 после записи.
 *
 * @param  sensor  Указатель на дескриптор датчика
 * @param  addr_h  Старший байт 16-битного IREG-адреса
 * @param  addr_l  Младший байт 16-битного IREG-адреса
 * @retval Прочитанное значение IREG-регистра
 *
 * @note   Использовать только в ICM_InitAllSensors() (не в DMA-цикле).
 */
uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor, uint8_t addr_h, uint8_t addr_l);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_SPI_H */
