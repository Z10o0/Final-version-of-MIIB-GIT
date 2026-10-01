/* =============================================================================
 * icm45686_spi.h
 *
 * Структуры, прототипы и топология SPI/DMA для массива ICM-45686.
 *
 * Регистровые адреса и битовые маски определены только в
 * icm45686_regs.h.
 *
 * Частоты, размер FIFO-батча, размер DMA-транзакции и топология
 * определены только в icm45686_config.h.
 * =============================================================================
 */

#ifndef ICM45686_SPI_H
#define ICM45686_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"
#include "icm45686_regs.h"
#include "icm45686_config.h"

/* ============================================================================
 * Состояния FSM одной SPI-шины
 * ========================================================================== */
typedef enum
{
    BUS_IDLE         = 0U,
    BUS_START_SENSOR = 1U,
    BUS_DMA_ACTIVE   = 2U,
    BUS_WAIT_EOT     = 3U,
    BUS_NEXT_SENSOR  = 4U,
    BUS_COMPLETE     = 5U,
    BUS_ERROR        = 6U,
    BUS_RECOVERY     = 7U
} icm_bus_state_t;

/* ============================================================================
 * Предварительно рассчитанный DMA-дескриптор
 *
 * Дескрипторы заполняются при ICM_BusesInit(). В hot path адреса и длина
 * переносятся непосредственно в DMA/BDMA без повторных вычислений.
 * ========================================================================== */
typedef struct
{
    uint32_t rx_mem_addr;
    uint32_t tx_mem_addr;
    uint16_t length;
} icm_dma_desc_t;

/* ============================================================================
 * Event bitmap: handoff между ISR и main loop
 * ========================================================================== */
#define ICM_EVT_BATCH_READY   (1UL << 0)
#define ICM_EVT_BUS0_FAULT    (1UL << 1)
#define ICM_EVT_BUS1_FAULT    (1UL << 2)
#define ICM_EVT_BUS2_FAULT    (1UL << 3)
#define ICM_EVT_DMA_TIMEOUT   (1UL << 4)
#define ICM_EVT_FRAME_SKIP    (1UL << 5)
#define ICM_EVT_BUS3_FAULT    (1UL << 6)
#define ICM_EVT_BUS4_FAULT    (1UL << 7)
#define ICM_EVT_BUS5_FAULT    (1UL << 8)

/* ============================================================================
 * Профилирование acquisition
 * ========================================================================== */
typedef struct
{
    uint32_t acq_start_cyc;
    uint32_t acq_end_cyc;
    uint32_t acq_lat_last_us;
    uint32_t acq_lat_max_us;
    uint32_t batch_count;
    uint32_t tim6_total;
    uint32_t frame_skip_count;
    uint32_t dma_timeout_count;
} icm_profile_t;

/* ============================================================================
 * Описание одного датчика
 * ========================================================================== */
typedef struct
{
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint32_t      cs_pin;

    uint8_t  sensor_id;       /* Глобальный ID 0...35. */
    uint8_t  fault;           /* 1: датчик временно изолирован. */
    uint8_t  fault_count;     /* Число последовательных ошибок. */
    uint16_t reint_countdown; /* Watchdog ticks до reintegration. */
} ICM_Sensor_t;

/* ============================================================================
 * Описание одной SPI-шины
 * ========================================================================== */
typedef struct
{
    SPI_TypeDef  *spi;

    /*
     * Для SPI1...SPI5 используется DMA.
     * Для SPI6 dma == NULL, используется BDMA.
     */
    DMA_TypeDef  *dma;
    BDMA_TypeDef *bdma;

    uint8_t       is_bdma;

    uint32_t dma_stream_rx;
    uint32_t dma_stream_tx;

    uint8_t *tx_buf;

    ICM_Sensor_t sensors[ICM_SENSORS_PER_BUS];

    volatile uint8_t current_sensor_idx;
    volatile uint8_t transfer_complete;
    volatile uint8_t eot_handled;

    icm_dma_desc_t dma_desc[ICM_SENSORS_PER_BUS];

    volatile icm_bus_state_t state;
    volatile uint32_t dma_start_cyc;
    volatile uint32_t timeout_count;
    volatile uint32_t dma_error_count;
} ICM_Bus_t;

/* ============================================================================
 * Глобальные объекты шин
 *
 * Порядок оставлен совместимым с существующим рабочим проектом:
 * нижняя плата — SPI1, SPI5, SPI4;
 * верхняя плата — SPI2, SPI3, SPI6.
 * ========================================================================== */
extern ICM_Bus_t g_bus_spi1;
extern ICM_Bus_t g_bus_spi5;
extern ICM_Bus_t g_bus_spi4;

extern ICM_Bus_t g_bus_spi2;
extern ICM_Bus_t g_bus_spi3;
extern ICM_Bus_t g_bus_spi6;

/* ============================================================================
 * FIFO RX-буферы
 *
 * SPI1...SPI5:
 *   .RAM_D2
 *
 * SPI6:
 *   .RAM_D3 / SRAM4, доступная BDMA
 * ========================================================================== */
extern uint8_t g_fifo_data
    [ICM_SPI_BUS_COUNT - 1U]
    [ICM_SENSORS_PER_BUS]
    [ICM_FIFO_DMA_BUF_SIZE];

extern uint8_t g_fifo_data_spi6
    [ICM_SENSORS_PER_BUS]
    [ICM_FIFO_DMA_BUF_SIZE];

/* ============================================================================
 * Состояние acquisition
 * ========================================================================== */
extern volatile uint8_t g_fifo_batch_ready;
extern volatile uint8_t g_dma_cycle_active;

/*
 * Sticky diagnostic flag.
 *
 * Устанавливается при:
 *   - пропуске TIM6-батча;
 *   - DMA timeout;
 *   - recovery любой SPI-шины.
 *
 * Флаг не запускает register/FIFO I/O из ISR. Полная безопасная
 * ресинхронизация должна выполняться отдельной процедурой вне ISR.
 */
extern volatile uint8_t g_fifo_resync_required;

/*
 * Для 36 датчиков требуется 64-битная маска.
 *
 * При установке и очистке отдельных битов обязательно использовать
 * 1ULL << sensor_id, а не 1UL << sensor_id.
 */
extern volatile uint64_t g_sensor_fault_mask;

extern volatile uint32_t g_dma_error_mask;
extern volatile uint32_t g_tim6_skip_count;
extern volatile uint32_t g_clk_ok_mask;
extern volatile uint32_t g_clk_fail_mask;

extern volatile uint32_t g_icm_events;
extern icm_profile_t      g_icm_profile;

/* ============================================================================
 * Инициализация
 * ========================================================================== */
void ICM_BusesInit(void);
void ICM_DWT_Init(void);

uint64_t ICM_InitAllSensors(void);

/* ============================================================================
 * Register access
 * ========================================================================== */
void ICM_WriteReg(ICM_Sensor_t *sensor,
                  uint8_t reg,
                  uint8_t value);

uint8_t ICM_ReadReg(ICM_Sensor_t *sensor,
                    uint8_t reg);

void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t addr_h,
                   uint8_t addr_l,
                   uint8_t value);

uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t addr_h,
                     uint8_t addr_l);

/* ============================================================================
 * Acquisition
 * ========================================================================== */

/*
 * Запускает параллельное обслуживание шести SPI-шин.
 *
 * На каждой шине шесть датчиков обслуживаются последовательно.
 * SPI1...SPI5 используют DMA, SPI6 использует BDMA.
 */
void ICM_StartBurstRead(void);

/*
 * Сохранено для совместимости с существующей архитектурой проекта.
 */
void ICM_StartBurstRead_SPI1(void);

/* ============================================================================
 * Watchdog и события
 * ========================================================================== */

/*
 * Вызывать из TIM7 IRQ с частотой 1 кГц.
 *
 * Приоритет TIM7 должен быть ниже приоритетов:
 *   - SPI RX DMA;
 *   - SPI EOT;
 *   - TIM6 acquisition trigger.
 */
void ICM_WatchdogTick(void);

/*
 * Атомарно получает накопленный event bitmap и очищает его.
 */
uint32_t ICM_ConsumeEvents(void);

/* ============================================================================
 * DMA RX completion wrappers
 * ========================================================================== */
void ICM_DMA_RxComplete_SPI1(void);
void ICM_DMA_RxComplete_SPI5(void);
void ICM_DMA_RxComplete_SPI4(void);

void ICM_DMA_RxComplete_SPI2(void);
void ICM_DMA_RxComplete_SPI3(void);
void ICM_DMA_RxComplete_SPI6(void);

/* ============================================================================
 * DMA error wrappers
 * ========================================================================== */
void ICM_DMA_Error_SPI1(void);
void ICM_DMA_Error_SPI5(void);
void ICM_DMA_Error_SPI4(void);

void ICM_DMA_Error_SPI2(void);
void ICM_DMA_Error_SPI3(void);
void ICM_DMA_Error_SPI6(void);

/* ============================================================================
 * SPI EOT wrappers
 * ========================================================================== */
void ICM_SPI_Eot_SPI1(void);
void ICM_SPI_Eot_SPI5(void);
void ICM_SPI_Eot_SPI4(void);

void ICM_SPI_Eot_SPI2(void);
void ICM_SPI_Eot_SPI3(void);
void ICM_SPI_Eot_SPI6(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_SPI_H */
