#ifndef ICM45686_SPI_H
#define ICM45686_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "icm45686_config.h"

typedef struct
{
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint32_t      cs_pin;
    uint8_t       sensor_id;
    uint8_t       fault;
} ICM_Sensor_t;

typedef struct
{
    SPI_TypeDef *spi;
    DMA_TypeDef *dma;
    uint32_t dma_stream_rx;
    uint32_t dma_stream_tx;
    ICM_Sensor_t sensors[ICM_SENSORS_PER_BUS];
    volatile uint8_t current_sensor_idx;
    volatile uint8_t transfer_complete;
    volatile uint8_t eot_handled;   /* защита от двойного входа в ICM_OnSpiEot */
    uint8_t *tx_buf;   /* указатель на AXI-буфер, не вложенный массив */
} ICM_Bus_t;

/*
 * Порядок global sensor_id:
 * SPI1: 0..5;
 * SPI5: 6..11;
 * SPI4: 12..17.
 */
extern ICM_Bus_t g_bus_spi1;
extern ICM_Bus_t g_bus_spi5;
extern ICM_Bus_t g_bus_spi4;

/*
 * Индекс первого измерения:
 * [0] — SPI1;
 * [1] — SPI5;
 * [2] — SPI4.
 *
 * g_fifo_data[x][y][0] — мусор, полученный во время FIFO address byte.
 */
extern uint8_t g_fifo_data[ICM_SPI_BUS_COUNT]
                          [ICM_SENSORS_PER_BUS]
                          [ICM_FIFO_DMA_BUF_SIZE];

extern volatile uint8_t g_fifo_batch_ready;
extern volatile uint8_t g_dma_cycle_active;
extern volatile uint32_t g_sensor_fault_mask;
extern volatile uint32_t g_dma_error_mask;
/* Диагностика захвата внешнего тактирования:
 * g_clk_ok_mask   — бит N = датчик N успешно захватил CLKIN (PLL_RDY)
 * g_clk_fail_mask — бит N = датчик N НЕ захватил CLKIN за таймаут 10 мс  */
extern volatile uint32_t g_clk_ok_mask;
extern volatile uint32_t g_clk_fail_mask;

void ICM_BusesInit(void);
uint32_t ICM_InitAllSensors(void);
void ICM_StartBurstRead(void);
void ICM_StartBurstRead_SPI1(void);

void ICM_DMA_RxComplete_SPI1(void);
void ICM_DMA_RxComplete_SPI5(void);
void ICM_DMA_RxComplete_SPI4(void);
void ICM_DMA_Error_SPI1(void);
void ICM_DMA_Error_SPI5(void);
void ICM_DMA_Error_SPI4(void);
void ICM_SPI_Eot_SPI1(void);
void ICM_SPI_Eot_SPI5(void);
void ICM_SPI_Eot_SPI4(void);

extern volatile uint8_t  g_fifo_batch_ready;
extern volatile uint8_t  g_dma_cycle_active;
extern volatile uint32_t g_sensor_fault_mask;
extern volatile uint32_t g_dma_error_mask;
extern uint8_t g_fifo_data[ICM_SPI_BUS_COUNT][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE];

void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t value);
uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg);

void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t addr_h,
                   uint8_t addr_l,
                   uint8_t value);

uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t addr_h,
                     uint8_t addr_l);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_SPI_H */
