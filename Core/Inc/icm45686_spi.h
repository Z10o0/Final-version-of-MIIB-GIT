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

typedef struct
{
    SPI_TypeDef   *spi;
    GPIO_TypeDef  *cs_port;
    uint32_t       cs_pin;
    uint8_t        sensor_id;   /* Глобальный ID 0..17 */
    uint8_t        fault;       /* 1 = датчик неисправен */
} ICM_Sensor_t;

typedef struct
{
    SPI_TypeDef   *spi;
    DMA_TypeDef   *dma;
    uint32_t       dma_stream_rx;
    uint32_t       dma_stream_tx;
    uint8_t       *tx_buf;

    ICM_Sensor_t   sensors[ICM_SENSORS_PER_BUS];

    volatile uint8_t  current_sensor_idx;
    volatile uint8_t  transfer_complete;
    volatile uint8_t  eot_handled;
} ICM_Bus_t;

/* ===========================================================================
 *  Глобальные переменные (extern)
 * ========================================================================== */

extern ICM_Bus_t g_bus_spi1;
extern ICM_Bus_t g_bus_spi5;
extern ICM_Bus_t g_bus_spi4;

extern uint8_t          g_fifo_data[ICM_SPI_BUS_COUNT][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE];
extern volatile uint8_t  g_fifo_batch_ready;
extern volatile uint8_t  g_dma_cycle_active;
extern volatile uint32_t g_sensor_fault_mask;
extern volatile uint32_t g_dma_error_mask;
extern volatile uint32_t g_tim6_skip_count;
extern volatile uint32_t g_clk_ok_mask;
extern volatile uint32_t g_clk_fail_mask;

/* ===========================================================================
 *  Публичные функции
 * ========================================================================== */

void     ICM_BusesInit       (void);
uint32_t ICM_InitAllSensors  (void);

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

/* SPI EOT ISR обёртки */
void ICM_SPI_Eot_SPI1(void);
void ICM_SPI_Eot_SPI5(void);
void ICM_SPI_Eot_SPI4(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_SPI_H */
