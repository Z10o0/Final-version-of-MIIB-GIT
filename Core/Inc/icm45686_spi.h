/*
 * Структуры, прототипы и топология шин ICM-45686.
 *
 * Register map: icm45686_regs.h.
 * Acquisition/output configuration: icm45686_config.h.
 *
 * Шесть параллельных шин, шесть датчиков последовательно на каждой.
 * SPI6 использует BDMA и отдельные D3/SRAM4-буферы.
 */

#ifndef ICM45686_SPI_H
#define ICM45686_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#include "icm45686_regs.h"
#include "icm45686_config.h"

/*
 * ICM_SPI_BUS_COUNT, ICM_SENSORS_PER_BUS,
 * ICM_TOTAL_SENSORS и FIFO-геометрия определены в config.h.
 * Всего 36 датчиков.
 */

typedef enum
{
    BUS_IDLE = 0U,
    BUS_START_SENSOR = 1U,
    BUS_DMA_ACTIVE = 2U,
    BUS_WAIT_EOT = 3U,
    BUS_NEXT_SENSOR = 4U,
    BUS_COMPLETE = 5U,
    BUS_ERROR = 6U,
    BUS_RECOVERY = 7U
} icm_bus_state_t;

typedef struct
{
    uint32_t rx_mem_addr;
    uint32_t tx_mem_addr;
    uint16_t length;
} icm_dma_desc_t;

/* Event bitmap: ISR -> main loop. */
#define ICM_EVT_BATCH_READY   (1UL << 0)

#define ICM_EVT_BUS0_FAULT    (1UL << 1)
#define ICM_EVT_BUS1_FAULT    (1UL << 2)
#define ICM_EVT_BUS2_FAULT    (1UL << 3)
#define ICM_EVT_BUS3_FAULT    (1UL << 6)
#define ICM_EVT_BUS4_FAULT    (1UL << 7)
#define ICM_EVT_BUS5_FAULT    (1UL << 8)

#define ICM_EVT_DMA_TIMEOUT   (1UL << 4)
#define ICM_EVT_FRAME_SKIP    (1UL << 5)

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

typedef struct
{
    SPI_TypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint32_t cs_pin;

    uint8_t sensor_id; /* Глобальный ID 0..35. */
    uint8_t fault;

    uint8_t fault_count;
    uint16_t reint_countdown;
} ICM_Sensor_t;

typedef struct
{
    SPI_TypeDef *spi;

    DMA_TypeDef *dma;   /* NULL для SPI6/BDMA. */
    BDMA_TypeDef *bdma; /* Используется только для SPI6. */

    uint8_t is_bdma;

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

/* Bus descriptors: порядок соответствует global sensor IDs. */
extern ICM_Bus_t g_bus_spi1;
extern ICM_Bus_t g_bus_spi5;
extern ICM_Bus_t g_bus_spi4;

extern ICM_Bus_t g_bus_spi2;
extern ICM_Bus_t g_bus_spi3;
extern ICM_Bus_t g_bus_spi6;

/* SPI1...SPI5 DMA buffers: .RAM_D2. */
extern uint8_t g_fifo_data
    [ICM_SPI_BUS_COUNT]
    [ICM_SENSORS_PER_BUS]
    [ICM_FIFO_DMA_BUF_SIZE];

/* SPI6 BDMA buffers: .RAM_D3 / SRAM4. */
extern uint8_t g_fifo_data_spi6
    [ICM_SENSORS_PER_BUS]
    [ICM_FIFO_DMA_BUF_SIZE];

extern volatile uint8_t g_fifo_batch_ready;
extern volatile uint8_t g_dma_cycle_active;

/*
 * Sticky diagnostic flag.
 *
 * Устанавливается при frame skip, DMA timeout и recovery любой шины.
 * Не запускает блокирующий FIFO/register I/O из ISR.
 * Безопасная полная FIFO-ресинхронизация относится к отдельному патчу.
 */
extern volatile uint8_t g_fifo_resync_required;

/* 36-bit sensor fault mask: обязательно uint64_t и 1ULL << sensor_id. */
extern volatile uint64_t g_sensor_fault_mask;

extern volatile uint32_t g_dma_error_mask;
extern volatile uint32_t g_tim6_skip_count;

extern volatile uint32_t g_clk_ok_mask;
extern volatile uint32_t g_clk_fail_mask;

extern volatile uint32_t g_icm_events;
extern icm_profile_t g_icm_profile;

/* Initialization. */
void ICM_BusesInit(void);
void ICM_DWT_Init(void);

uint64_t ICM_InitAllSensors(void);

/*
 * TIM7 watchdog: 1 кГц.
 * IRQ priority ниже SPI/DMA/TIM6.
 */
void ICM_WatchdogTick(void);

uint32_t ICM_ConsumeEvents(void);

/* Register access API сохранён. */
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

/* Acquisition. */
void ICM_StartBurstRead(void);
void ICM_StartBurstRead_SPI1(void);

/* DMA RX complete: нижняя плата. */
void ICM_DMA_RxComplete_SPI1(void);
void ICM_DMA_RxComplete_SPI5(void);
void ICM_DMA_RxComplete_SPI4(void);

/* DMA error: нижняя плата. */
void ICM_DMA_Error_SPI1(void);
void ICM_DMA_Error_SPI5(void);
void ICM_DMA_Error_SPI4(void);

/* DMA RX complete: верхняя плата. */
void ICM_DMA_RxComplete_SPI2(void);
void ICM_DMA_RxComplete_SPI3(void);
void ICM_DMA_RxComplete_SPI6(void);

/* DMA/BDMA error: верхняя плата. */
void ICM_DMA_Error_SPI2(void);
void ICM_DMA_Error_SPI3(void);
void ICM_DMA_Error_SPI6(void);

/* SPI EOT: нижняя плата. */
void ICM_SPI_Eot_SPI1(void);
void ICM_SPI_Eot_SPI5(void);
void ICM_SPI_Eot_SPI4(void);

/* SPI EOT: верхняя плата. */
void ICM_SPI_Eot_SPI2(void);
void ICM_SPI_Eot_SPI3(void);
void ICM_SPI_Eot_SPI6(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_SPI_H */
