/* =============================================================================
 * icm45686_spi.h
 *
 * Структуры, прототипы и топология шин для ICM-45686 (SPI + DMA).
 *
 * ВНИМАНИЕ: все адреса регистров и битовые маски — ТОЛЬКО в icm45686_regs.h.
 *           Этот файл НЕ содержит регистровых макросов.
 * =============================================================================
 */

#ifndef ICM45686_SPI_H
#define ICM45686_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "icm45686_regs.h"   /* единственный источник всех макросов */
#include "icm45686_config.h" /* ODR / FS / FIFO-параметры           */

/* ===========================================================================
 *  Топология системы
 * ========================================================================== */
/* ICM_SPI_BUS_COUNT и ICM_SENSORS_PER_BUS определены в icm45686_config.h */
#define ICM_TOTAL_SENSORS    (ICM_SPI_BUS_COUNT * ICM_SENSORS_PER_BUS)  /* 18 */

/* ===========================================================================
 *  FIFO DMA буфер
 *  Размер считается из icm45686_config.h: ICM_FIFO_DMA_BUF_SIZE уже определён.
 * ========================================================================== */
/* ICM_FIFO_DMA_BUF_SIZE     — из icm45686_config.h */
/* ICM_FIFO_WATERMARK_BYTES  — из icm45686_config.h */

/* ===========================================================================
 *  Структуры
 * ========================================================================== */

/* [NEW] Явная FSM шины — заменяет неявные комбинации
 * current_sensor_idx/transfer_complete/eot_handled. */
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

/* [NEW] Precomputed DMA descriptor — заполняется один раз в ICM_BusesInit(),
 * хот-пас (ICM_StartBusRead) только копирует значения в регистры DMA. */
typedef struct
{
    uint32_t rx_mem_addr;
    uint32_t tx_mem_addr;
    uint16_t length;
} icm_dma_desc_t;

/* [NEW] Event bitmap — атомарный lock-free handoff между ISR и main loop. */
#define ICM_EVT_BATCH_READY   (1UL << 0)
#define ICM_EVT_BUS0_FAULT    (1UL << 1)
#define ICM_EVT_BUS1_FAULT    (1UL << 2)
#define ICM_EVT_BUS2_FAULT    (1UL << 3)
#define ICM_EVT_BUS3_FAULT    (1UL << 6)
#define ICM_EVT_BUS4_FAULT    (1UL << 7)
#define ICM_EVT_BUS5_FAULT    (1UL << 8)
#define ICM_EVT_DMA_TIMEOUT   (1UL << 4)
#define ICM_EVT_FRAME_SKIP    (1UL << 5)

/* [NEW] DWT-профилирование latency/загрузки для диагностики и отчётности. */
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
    SPI_TypeDef   *spi;
    GPIO_TypeDef  *cs_port;
    uint32_t       cs_pin;
    uint8_t        sensor_id;        /* Глобальный ID 0..17 */
    uint8_t        fault;            /* 1 = датчик неисправен */
    uint8_t        fault_count;      /* [NEW] подряд идущие ошибки */
    uint16_t       reint_countdown;  /* [NEW] тиков watchdog до повторной попытки reintegration */
} ICM_Sensor_t;

typedef struct
{
    SPI_TypeDef   *spi;
    DMA_TypeDef   *dma;         /* NULL для BDMA-шин */
    BDMA_TypeDef  *bdma;        /* [NEW] используется только для SPI6 */
    uint8_t        is_bdma;     /* [NEW] 1 = шина работает через BDMA */
    uint32_t       dma_stream_rx;
    uint32_t       dma_stream_tx;
    uint8_t       *tx_buf;

    ICM_Sensor_t   sensors[ICM_SENSORS_PER_BUS];

    volatile uint8_t  current_sensor_idx;
    volatile uint8_t  transfer_complete;
    volatile uint8_t  eot_handled;

    icm_dma_desc_t    dma_desc[ICM_SENSORS_PER_BUS];
    volatile icm_bus_state_t state;
    volatile uint32_t dma_start_cyc;
    volatile uint32_t timeout_count;
    volatile uint32_t dma_error_count;
} ICM_Bus_t;

/* ===========================================================================
 *  Глобальные переменные (extern)
 * ========================================================================== */

extern ICM_Bus_t g_bus_spi1;
extern ICM_Bus_t g_bus_spi5;
extern ICM_Bus_t g_bus_spi4;

extern ICM_Bus_t g_bus_spi2;
extern ICM_Bus_t g_bus_spi3;
extern ICM_Bus_t g_bus_spi6;

extern uint8_t          g_fifo_data[ICM_SPI_BUS_COUNT][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE];
extern uint8_t 			g_fifo_data_spi6[ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE];
extern volatile uint8_t  g_fifo_batch_ready;
extern volatile uint8_t  g_dma_cycle_active;
extern volatile uint32_t g_sensor_fault_mask;
extern volatile uint32_t g_dma_error_mask;
extern volatile uint32_t g_tim6_skip_count;
extern volatile uint32_t g_clk_ok_mask;
extern volatile uint32_t g_clk_fail_mask;

extern volatile uint32_t g_icm_events;   /* [NEW] атомарный event bitmap */
extern icm_profile_t     g_icm_profile;  /* [NEW] DWT-профилирование */

/* ===========================================================================
 *  Публичные функции
 * ========================================================================== */

void     ICM_BusesInit       (void);
void     ICM_DWT_Init        (void);   /* [NEW] */
uint32_t ICM_InitAllSensors  (void);
void     ICM_WatchdogTick    (void);   /* [NEW] вызывать из TIM7 IRQ на частоте >100 Гц */
uint32_t ICM_ConsumeEvents   (void);   /* [NEW] атомарно забирает и очищает event bitmap */

void     ICM_WriteReg        (ICM_Sensor_t *sensor, uint8_t reg, uint8_t value);
uint8_t  ICM_ReadReg         (ICM_Sensor_t *sensor, uint8_t reg);
void     ICM_WriteIReg       (ICM_Sensor_t *sensor, uint8_t addr_h, uint8_t addr_l, uint8_t value);
uint8_t  ICM_ReadIReg        (ICM_Sensor_t *sensor, uint8_t addr_h, uint8_t addr_l);

void     ICM_StartBurstRead  (void);
void     ICM_StartBurstRead_SPI1(void);

/* DMA ISR обёртки */
void ICM_DMA_RxComplete_SPI1(void);
void ICM_DMA_RxComplete_SPI5(void);
void ICM_DMA_RxComplete_SPI4(void);
void ICM_DMA_Error_SPI1(void);
void ICM_DMA_Error_SPI5(void);
void ICM_DMA_Error_SPI4(void);

/* DMA ISR обёртки — верхняя плата */
void ICM_DMA_RxComplete_SPI2(void);
void ICM_DMA_RxComplete_SPI3(void);
void ICM_DMA_RxComplete_SPI6(void);
void ICM_DMA_Error_SPI2(void);
void ICM_DMA_Error_SPI3(void);
void ICM_DMA_Error_SPI6(void);

/* SPI EOT ISR обёртки */
void ICM_SPI_Eot_SPI1(void);
void ICM_SPI_Eot_SPI5(void);
void ICM_SPI_Eot_SPI4(void);

/* SPI EOT ISR обёртки — верхняя плата */
void ICM_SPI_Eot_SPI2(void);
void ICM_SPI_Eot_SPI3(void);
void ICM_SPI_Eot_SPI6(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_SPI_H */
