/* =============================================================================
 * icm45686_spi.c — ПАРАЛЛЕЛЬНАЯ версия, все 3 шины SPI работают одновременно
 *
 *  1.  WHO_AM_I
 *  2.  Soft Reset (REG_MISC2 bit1)
 *  3.  Ждём INT1_STATUS0 bit7 (RESET_DONE) — надёжнее чем WHO_AM_I
 *  4.  0x30[bit1]=1, [bit0]=0  — AUX1 off
 *  5.  0x31[bit2]=1, [1:0]=10b — INT2 → CLKIN
 *  6.  IREG 0xA268[bit2]=0     — I3C STC mode off
 *  7.  IREG 0xA57B[1:0]=0b10   — accel_src_ctrl = FIR+interp
 *  8.  IREG 0xA4A6[6:5]=0b10   — gyro_src_ctrl  = FIR+interp
 *  9а. IREG 0xA258[bit4|bit0]  — ACCEL_LP_CLK_SEL + tmst_en (оба при CLKIN!)
 *  9б. 0x26[bit6|bit5]=1       — rtc_align + rtc_mode
 * 10.  ACCEL_CONFIG0 + GYRO_CONFIG0
 * 11.  PWR_MGMT0: Gyro LN + Accel LN  ← ДО FIFO
 * 12.  FIFO: flush → config0 → watermark → config4(TMST) → config3 → IF_EN
 *      [FIX] Порядок изменён: FLUSH первый, FIFO_CONFIG4 ДО IF_EN
 * 13.  Startup delay 200 мс
 * =============================================================================
 */

#include "icm45686_spi.h"
#include <string.h>

#define ICM45686_INT1_STATUS0_RESET_DONE    (1U << 7)
#define ICM45686_ACCEL_LP_CLK_SEL           (1U << 4)

static void    ICM_DelayUs            (uint32_t us);
static void    ICM_DelayMs            (uint32_t ms);
static void    ICM_CS_Low             (const ICM_Sensor_t *s);
static void    ICM_CS_High            (const ICM_Sensor_t *s);
static void    ICM_SPI_EnsureDisabled (SPI_TypeDef *spi);
static void    ICM_SPI_WaitEOT        (SPI_TypeDef *spi);
static void    ICM_SPI_DrainRx        (SPI_TypeDef *spi, uint32_t n);
static uint8_t ICM_BusIndex           (const ICM_Bus_t *bus);
static uint8_t ICM_FindNextHealthy    (const ICM_Bus_t *bus, uint8_t from);
static void    ICM_ClearDmaFlags      (const ICM_Bus_t *bus);
static void    ICM_WaitIRegReady      (void);
static void    ICM_StartBusRead       (ICM_Bus_t *bus, uint8_t idx);
static void    ICM_NextSensor         (ICM_Bus_t *bus);
static void    ICM_OnSpiEot           (ICM_Bus_t *bus);
static void    ICM_FinishBus          (ICM_Bus_t *bus);
static void    ICM_TryCompleteBatch   (void);
static void    ICM_MarkFault          (ICM_Sensor_t *s);

uint8_t g_fifo_data[ICM_SPI_BUS_COUNT][ICM_SENSORS_PER_BUS][ICM_FIFO_DMA_BUF_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static uint8_t g_tx_spi1[ICM_FIFO_DMA_BUF_SIZE] __attribute__((section(".RAM_D2"), aligned(32)));
static uint8_t g_tx_spi5[ICM_FIFO_DMA_BUF_SIZE] __attribute__((section(".RAM_D2"), aligned(32)));
static uint8_t g_tx_spi4[ICM_FIFO_DMA_BUF_SIZE] __attribute__((section(".RAM_D2"), aligned(32)));

volatile uint8_t  g_fifo_batch_ready  = 0U;
volatile uint8_t  g_dma_cycle_active  = 0U;
volatile uint32_t g_sensor_fault_mask = 0U;
volatile uint32_t g_dma_error_mask    = 0U;
volatile uint32_t g_tim6_skip_count   = 0U;
volatile uint32_t g_clk_ok_mask       = 0U;
volatile uint32_t g_clk_fail_mask     = 0U;

ICM_Bus_t g_bus_spi1 =
{
    .spi           = SPI1,
    .dma           = DMA1,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .tx_buf        = g_tx_spi1,
    .eot_handled   = 0U,
    .sensors =
    {
        { SPI1, GPIOB, LL_GPIO_PIN_12, 0U,  0U },
        { SPI1, GPIOB, LL_GPIO_PIN_13, 1U,  0U },
        { SPI1, GPIOE, LL_GPIO_PIN_8,  2U,  0U },
        { SPI1, GPIOE, LL_GPIO_PIN_9,  3U,  0U },
        { SPI1, GPIOF, LL_GPIO_PIN_13, 4U,  0U },
        { SPI1, GPIOF, LL_GPIO_PIN_14, 5U,  0U }
    }
};

ICM_Bus_t g_bus_spi5 =
{
    .spi           = SPI5,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_2,
    .dma_stream_tx = LL_DMA_STREAM_3,
    .tx_buf        = g_tx_spi5,
    .eot_handled   = 0U,
    .sensors =
    {
        { SPI5, GPIOE, LL_GPIO_PIN_14, 6U,  0U },
        { SPI5, GPIOE, LL_GPIO_PIN_15, 7U,  0U },
        { SPI5, GPIOE, LL_GPIO_PIN_7,  8U,  0U },
        { SPI5, GPIOG, LL_GPIO_PIN_1,  9U,  0U },
        { SPI5, GPIOB, LL_GPIO_PIN_0,  10U, 0U },
        { SPI5, GPIOB, LL_GPIO_PIN_1,  11U, 0U }
    }
};

ICM_Bus_t g_bus_spi4 =
{
    .spi           = SPI4,
    .dma           = DMA2,
    .dma_stream_rx = LL_DMA_STREAM_0,
    .dma_stream_tx = LL_DMA_STREAM_1,
    .tx_buf        = g_tx_spi4,
    .eot_handled   = 0U,
    .sensors =
    {
        { SPI4, GPIOE, LL_GPIO_PIN_10, 12U, 0U },
        { SPI4, GPIOE, LL_GPIO_PIN_11, 13U, 0U },
        { SPI4, GPIOF, LL_GPIO_PIN_15, 14U, 0U },
        { SPI4, GPIOG, LL_GPIO_PIN_0,  15U, 0U },
        { SPI4, GPIOC, LL_GPIO_PIN_4,  16U, 0U },
        { SPI4, GPIOC, LL_GPIO_PIN_5,  17U, 0U }
    }
};

void ICM_BusesInit(void)
{
    uint8_t i;

    g_bus_spi1.tx_buf = g_tx_spi1;
    g_bus_spi5.tx_buf = g_tx_spi5;
    g_bus_spi4.tx_buf = g_tx_spi4;

    memset(g_fifo_data, 0x00U, sizeof(g_fifo_data));
    memset(g_tx_spi1,   0xFFU, sizeof(g_tx_spi1));
    memset(g_tx_spi5,   0xFFU, sizeof(g_tx_spi5));
    memset(g_tx_spi4,   0xFFU, sizeof(g_tx_spi4));

    g_tx_spi1[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi5[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;
    g_tx_spi4[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    for (i = 0U; i < ICM_SENSORS_PER_BUS; i++)
    {
        ICM_CS_High(&g_bus_spi1.sensors[i]);
        ICM_CS_High(&g_bus_spi5.sensors[i]);
        ICM_CS_High(&g_bus_spi4.sensors[i]);
        g_bus_spi1.sensors[i].fault = 0U;
        g_bus_spi5.sensors[i].fault = 0U;
        g_bus_spi4.sensors[i].fault = 0U;
    }

    g_bus_spi1.current_sensor_idx = 0U;
    g_bus_spi5.current_sensor_idx = 0U;
    g_bus_spi4.current_sensor_idx = 0U;
    g_bus_spi1.transfer_complete  = 0U;
    g_bus_spi5.transfer_complete  = 0U;
    g_bus_spi4.transfer_complete  = 0U;
    g_bus_spi1.eot_handled        = 0U;
    g_bus_spi5.eot_handled        = 0U;
    g_bus_spi4.eot_handled        = 0U;

    g_fifo_batch_ready  = 0U;
    g_dma_cycle_active  = 0U;
    g_sensor_fault_mask = 0U;
    g_dma_error_mask    = 0U;
    g_tim6_skip_count   = 0U;
    g_clk_ok_mask       = 0U;
    g_clk_fail_mask     = 0U;
}

void ICM_WriteReg(ICM_Sensor_t *sensor, uint8_t reg, uint8_t value)
{
    SPI_TypeDef *spi = sensor->spi;

    ICM_SPI_EnsureDisabled(spi);
    LL_SPI_SetTransferSize(spi, 2U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, reg & 0x7FU);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, value);

    ICM_SPI_WaitEOT(spi);
    ICM_SPI_DrainRx(spi, 2U);

    ICM_CS_High(sensor);
    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}
}

uint8_t ICM_ReadReg(ICM_Sensor_t *sensor, uint8_t reg)
{
    SPI_TypeDef *spi = sensor->spi;
    uint8_t      dummy;
    uint8_t      result;

    ICM_SPI_EnsureDisabled(spi);
    LL_SPI_SetTransferSize(spi, 2U);
    LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_ClearFlag_EOT(spi);

    ICM_CS_Low(sensor);
    LL_SPI_Enable(spi);
    LL_SPI_StartMasterTransfer(spi);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, (reg & 0x7FU) | ICM45686_SPI_READ_BIT);

    while (LL_SPI_IsActiveFlag_TXP(spi) == 0U) {}
    LL_SPI_TransmitData8(spi, 0xFFU);

    ICM_SPI_WaitEOT(spi);

    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
    dummy = LL_SPI_ReceiveData8(spi);
    (void)dummy;

    while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
    result = LL_SPI_ReceiveData8(spi);

    ICM_CS_High(sensor);
    LL_SPI_Disable(spi);
    while (LL_SPI_IsEnabled(spi) != 0U) {}

    return result;
}

static void ICM_WaitIRegReady(void)
{
    ICM_DelayUs(10U);
}

void ICM_WriteIReg(ICM_Sensor_t *sensor,
                   uint8_t       addr_h,
                   uint8_t       addr_l,
                   uint8_t       value)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);
    ICM_WaitIRegReady();
    ICM_WriteReg(sensor, ICM45686_REG_IREG_DATA, value);
    ICM_WaitIRegReady();
}

uint8_t ICM_ReadIReg(ICM_Sensor_t *sensor,
                     uint8_t       addr_h,
                     uint8_t       addr_l)
{
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_15_8, addr_h);
    ICM_WriteReg(sensor, ICM45686_REG_IREG_ADDR_7_0,  addr_l);
    ICM_WaitIRegReady();
    return ICM_ReadReg(sensor, ICM45686_REG_IREG_DATA);
}

uint32_t ICM_InitAllSensors(void)
{
    ICM_Bus_t * const buses[ICM_SPI_BUS_COUNT] = { &g_bus_spi1, &g_bus_spi5, &g_bus_spi4 };
    uint8_t  bus_idx, sensor_idx, reg_val;
    uint32_t timeout;

    g_sensor_fault_mask = 0U;
    g_clk_ok_mask       = 0U;
    g_clk_fail_mask     = 0U;

    ICM_DelayMs(3U);

    for (bus_idx = 0U; bus_idx < ICM_SPI_BUS_COUNT; bus_idx++)
    {
        for (sensor_idx = 0U; sensor_idx < ICM_SENSORS_PER_BUS; sensor_idx++)
        {
            ICM_Sensor_t *sensor = &buses[bus_idx]->sensors[sensor_idx];

            /* ШАГ 1: WHO_AM_I до сброса */
            if (ICM_ReadReg(sensor, ICM45686_REG_WHO_AM_I) != ICM45686_WHO_AM_I_VALUE) {
                ICM_MarkFault(sensor); continue;
            }

            /* ШАГ 2: Soft Reset */
            ICM_WriteReg(sensor, ICM45686_REG_REG_MISC2, ICM45686_MISC2_SOFT_RST);

            /* ШАГ 3: Ждём RESET_DONE */
            timeout = 1000U;
            do {
                ICM_DelayUs(10U);
                reg_val = ICM_ReadReg(sensor, ICM45686_REG_INT1_STATUS0);
                timeout--;
            } while (((reg_val & ICM45686_INT1_STATUS0_RESET_DONE) == 0U) && (timeout != 0U));

            if (timeout == 0U) { ICM_MarkFault(sensor); continue; }


            /* === ОБЯЗАТЕЛЬНО: включаем Big Endian для FIFO и регистров ===
             * SREGCTRL.SREGDATAENDIANSEL = 1 (Big Endian)
             * IREG IPREGTOP1 base = 0xA200, reg addr = 0x67 → 0xA267
             * Даташит DS-000577, раздел 20.50 SREGCTRL
             */
            ICM_WriteIReg(sensor, 0xA2U, 0x67U, 0x02U);  /* bit1 = SREGDATAENDIANSEL */
            ICM_DelayUs(10U);


            /* ШАГ 4 & 5: AUX1 off и INT2 -> CLKIN */
            reg_val = ICM_ReadReg(sensor, ICM45686_REG_IOC_PAD_AUX_OVRD);
            reg_val |= ICM45686_AUX1_ENABLE_OVRD; reg_val &= ~ICM45686_AUX1_ENABLE_OVRD_VAL;
            ICM_WriteReg(sensor, ICM45686_REG_IOC_PAD_AUX_OVRD, reg_val);

            reg_val = ICM_ReadReg(sensor, ICM45686_REG_IOC_PAD_SCENARIO_OVRD);
            reg_val |= ICM45686_INT2_CFG_OVRD_EN; reg_val = (reg_val & ~0x03U) | ICM45686_INT2_CFG_CLKIN_VAL;
            ICM_WriteReg(sensor, ICM45686_REG_IOC_PAD_SCENARIO_OVRD, reg_val);

            /* ШАГ 6-8: I3C STC off и FIR фильтры (Оставляем как было) */
            reg_val = ICM_ReadIReg(sensor, ICM45686_IREG_I3C_STC_CFG_H, ICM45686_IREG_I3C_STC_CFG_L);
            reg_val &= ~ICM45686_I3C_STC_MODE_BIT;
            ICM_WriteIReg(sensor, ICM45686_IREG_I3C_STC_CFG_H, ICM45686_IREG_I3C_STC_CFG_L, reg_val);

            reg_val = ICM_ReadIReg(sensor, ICM45686_IREG_ACCEL_SRC_CTRL_H, ICM45686_IREG_ACCEL_SRC_CTRL_L);
            reg_val = (reg_val & ~0x03U) | ICM45686_ACCEL_SRC_FIR_INTERP;
            ICM_WriteIReg(sensor, ICM45686_IREG_ACCEL_SRC_CTRL_H, ICM45686_IREG_ACCEL_SRC_CTRL_L, reg_val);

            reg_val = ICM_ReadIReg(sensor, ICM45686_IREG_GYRO_SRC_CTRL_H, ICM45686_IREG_GYRO_SRC_CTRL_L);
            reg_val = (reg_val & ~ICM45686_GYRO_SRC_CTRL_MASK) | (ICM45686_GYRO_SRC_FIR_INTERP << ICM45686_GYRO_SRC_CTRL_SHIFT);
            ICM_WriteIReg(sensor, ICM45686_IREG_GYRO_SRC_CTRL_H, ICM45686_IREG_GYRO_SRC_CTRL_L, reg_val);

            /* ШАГ 9: включаем timestamp-счётчик
             * tmst_en (bit0) = главный тик счётчика.
             * ACCEL_LP_CLK_SEL (bit4) — ТОЛЬКО для LP-режима, в LN НЕ ставим!
             */
            reg_val = ICM_ReadIReg(sensor, ICM45686_IREG_SMC_CONTROL_0_H,
                                           ICM45686_IREG_SMC_CONTROL_0_L);
            reg_val |= ICM45686_TMST_EN;              /* бит 0 — запускает счётчик */
            reg_val &= ~ICM45686_ACCEL_LP_CLK_SEL;   /* бит 4 — убираем, мы в LN! */
            ICM_WriteIReg(sensor, ICM45686_IREG_SMC_CONTROL_0_H,
                                  ICM45686_IREG_SMC_CONTROL_0_L, reg_val);
            ICM_DelayUs(500U);

            /* FIFO_CONFIG4: разрешаем класть timestamp в пакет */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG4, ICM45686_FIFO_TMST_FSYNC_EN);

            /* ШАГ 10: ODR + FSR */
            ICM_WriteReg(sensor, ICM45686_REG_ACCEL_CONFIG0, ICM_ACCEL_FS_VALUE | ICM_ACCEL_ODR_VALUE);
            ICM_WriteReg(sensor, ICM45686_REG_GYRO_CONFIG0, ICM_GYRO_FS_VALUE | ICM_GYRO_ODR_VALUE);

            /* ШАГ 11: Питание ДО FIFO! */
            ICM_WriteReg(sensor, ICM45686_REG_PWR_MGMT0, ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN);
            ICM_DelayMs(5U);

            /* ==============================================================
             * ШАГ 12: FIFO CONFIGURATION (20-BYTE HIRES MODE)
             * ============================================================== */
            /* 1. Останавливаем интерфейс и очищаем FIFO */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG3, 0x00);
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG2, ICM45686_FIFO_FLUSH);
            ICM_DelayUs(100U);

            /* 2. Stream mode + Max Depth */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG0, ICM45686_FIFO_MODE_STREAM | ICM45686_FIFO_DEPTH_MAX);

            /* 3. Watermark */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_0,
                (uint8_t)(ICM_FIFO_WATERMARK_BYTES & 0x00FFU));
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG1_1,
                (uint8_t)((ICM_FIFO_WATERMARK_BYTES >> 8U) & 0x00FFU));

            /* 4. Включаем Timestamp */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG4, ICM45686_FIFO_TMST_FSYNC_EN);

            /* 5. Включаем Accel, Gyro И HI-RES (!), но БЕЗ IF_EN */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN | ICM45686_FIFO_HIRES_EN);

            /* 6. Включаем IF_EN */
            ICM_WriteReg(sensor, ICM45686_REG_FIFO_CONFIG3,
                         ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN | ICM45686_FIFO_HIRES_EN | ICM45686_FIFO_IF_EN);

            /* ШАГ 13: Startup delay */
            ICM_DelayMs(ICM45686_STARTUP_DELAY_MS);
        }
    }

    return g_sensor_fault_mask;
}

void ICM_StartBurstRead(void)
{
    uint8_t first1, first5, first4;

    if ((g_fifo_batch_ready != 0U) || (g_dma_cycle_active != 0U))
    {
        g_tim6_skip_count++;
        return;
    }

    g_dma_cycle_active = 1U;
    g_fifo_batch_ready = 0U;

    g_bus_spi1.transfer_complete = 0U;
    g_bus_spi5.transfer_complete = 0U;
    g_bus_spi4.transfer_complete = 0U;
    g_bus_spi1.eot_handled       = 0U;
    g_bus_spi5.eot_handled       = 0U;
    g_bus_spi4.eot_handled       = 0U;

    first1 = ICM_FindNextHealthy(&g_bus_spi1, 0U);
    first5 = ICM_FindNextHealthy(&g_bus_spi5, 0U);
    first4 = ICM_FindNextHealthy(&g_bus_spi4, 0U);

    if (first1 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi1, first1); }
    else                               { ICM_FinishBus(&g_bus_spi1); }

    if (first5 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi5, first5); }
    else                               { ICM_FinishBus(&g_bus_spi5); }

    if (first4 < ICM_SENSORS_PER_BUS) { ICM_StartBusRead(&g_bus_spi4, first4); }
    else                               { ICM_FinishBus(&g_bus_spi4); }
}

void ICM_StartBurstRead_SPI1(void) { ICM_StartBurstRead(); }

void ICM_DMA_RxComplete_SPI1(void) { ICM_NextSensor(&g_bus_spi1); }
void ICM_DMA_RxComplete_SPI5(void) { ICM_NextSensor(&g_bus_spi5); }
void ICM_DMA_RxComplete_SPI4(void) { ICM_NextSensor(&g_bus_spi4); }
void ICM_DMA_Error_SPI1(void) { g_dma_error_mask |= (1UL << 0U); ICM_NextSensor(&g_bus_spi1); }
void ICM_DMA_Error_SPI5(void) { g_dma_error_mask |= (1UL << 1U); ICM_NextSensor(&g_bus_spi5); }
void ICM_DMA_Error_SPI4(void) { g_dma_error_mask |= (1UL << 2U); ICM_NextSensor(&g_bus_spi4); }

void ICM_SPI_Eot_SPI1(void) { ICM_OnSpiEot(&g_bus_spi1); }
void ICM_SPI_Eot_SPI5(void) { ICM_OnSpiEot(&g_bus_spi5); }
void ICM_SPI_Eot_SPI4(void) { ICM_OnSpiEot(&g_bus_spi4); }

static void ICM_StartBusRead(ICM_Bus_t *bus, uint8_t idx)
{
    ICM_Sensor_t *sensor;
    uint8_t       bus_idx;
    uint8_t      *rx_buf;

    bus_idx = ICM_BusIndex(bus);
    if (bus_idx >= ICM_SPI_BUS_COUNT)   { return; }
    if (idx     >= ICM_SENSORS_PER_BUS) { return; }

    sensor = &bus->sensors[idx];
    rx_buf = g_fifo_data[bus_idx][idx];

    bus->eot_handled = 0U;

    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_rx) != 0U) {}
    while (LL_DMA_IsEnabledStream(bus->dma, bus->dma_stream_tx) != 0U) {}

    ICM_ClearDmaFlags(bus);

    memset(rx_buf,      0x00U, ICM_FIFO_DMA_BUF_SIZE);
    memset(bus->tx_buf, 0x00U, ICM_FIFO_DMA_BUF_SIZE);
    bus->tx_buf[0] = ICM45686_REG_FIFO_DATA | ICM45686_SPI_READ_BIT;

    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_rx,
                            LL_SPI_DMA_GetRxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_rx, (uint32_t)rx_buf);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_rx, ICM_FIFO_DMA_BUF_SIZE);

    LL_DMA_SetPeriphAddress(bus->dma, bus->dma_stream_tx,
                            LL_SPI_DMA_GetTxRegAddr(bus->spi));
    LL_DMA_SetMemoryAddress(bus->dma, bus->dma_stream_tx, (uint32_t)bus->tx_buf);
    LL_DMA_SetDataLength   (bus->dma, bus->dma_stream_tx, ICM_FIFO_DMA_BUF_SIZE);

    LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream_rx);
    LL_SPI_DisableIT_EOT(bus->spi);

    bus->current_sensor_idx = idx;

    ICM_CS_Low(sensor);
    ICM_DelayUs(2U);

    ICM_SPI_EnsureDisabled(bus->spi);

    while (LL_SPI_IsActiveFlag_RXP(bus->spi) != 0U)
        (void)LL_SPI_ReceiveData8(bus->spi);
    WRITE_REG(bus->spi->IFCR, 0x0FF8U);

    LL_SPI_SetTransferSize(bus->spi, ICM_FIFO_DMA_BUF_SIZE);
    LL_SPI_SetInternalSSLevel(bus->spi, LL_SPI_SS_LEVEL_HIGH);

    LL_DMA_EnableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_EnableStream(bus->dma, bus->dma_stream_tx);

    LL_SPI_EnableDMAReq_RX(bus->spi);
    LL_SPI_EnableDMAReq_TX(bus->spi);

    LL_SPI_Enable(bus->spi);
    __DSB();
    LL_SPI_StartMasterTransfer(bus->spi);
}

static void ICM_NextSensor(ICM_Bus_t *bus)
{
    LL_SPI_DisableDMAReq_RX(bus->spi);
    LL_SPI_DisableDMAReq_TX(bus->spi);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_rx);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream_tx);

    LL_SPI_EnableIT_EOT(bus->spi);

    if (LL_SPI_IsActiveFlag_EOT(bus->spi) != 0U)
        ICM_OnSpiEot(bus);
}

static void ICM_OnSpiEot(ICM_Bus_t *bus)
{
    uint8_t prev_idx;
    uint8_t next_idx;

    if (LL_SPI_IsActiveFlag_EOT(bus->spi) == 0U) { return; }

    if (bus->eot_handled != 0U)
    {
        LL_SPI_DisableIT_EOT(bus->spi);
        LL_SPI_ClearFlag_EOT(bus->spi);
        return;
    }
    bus->eot_handled = 1U;

    LL_SPI_DisableIT_EOT(bus->spi);
    LL_SPI_ClearFlag_EOT(bus->spi);
    LL_SPI_ClearFlag_TXTF(bus->spi);
    WRITE_REG(bus->spi->IFCR, 0x0FF8U);

    prev_idx = bus->current_sensor_idx;
    if (prev_idx >= ICM_SENSORS_PER_BUS) { ICM_FinishBus(bus); return; }

    ICM_CS_High(&bus->sensors[prev_idx]);
    LL_SPI_Disable(bus->spi);
    __DSB();

    next_idx = ICM_FindNextHealthy(bus, (uint8_t)(prev_idx + 1U));
    if (next_idx < ICM_SENSORS_PER_BUS)
        ICM_StartBusRead(bus, next_idx);
    else
        ICM_FinishBus(bus);
}

static void ICM_FinishBus(ICM_Bus_t *bus)
{
    bus->transfer_complete = 1U;
    ICM_TryCompleteBatch();
}

static void ICM_TryCompleteBatch(void)
{
    if ((g_bus_spi1.transfer_complete != 0U) &&
        (g_bus_spi5.transfer_complete != 0U) &&
        (g_bus_spi4.transfer_complete != 0U))
    {
        g_dma_cycle_active = 0U;
        g_fifo_batch_ready = 1U;
    }
}

static void ICM_MarkFault(ICM_Sensor_t *s)
{
    s->fault = 1U;
    g_sensor_fault_mask |= (1UL << s->sensor_id);
}

static void ICM_CS_Low(const ICM_Sensor_t *s)
{
    LL_GPIO_ResetOutputPin(s->cs_port, s->cs_pin);
}

static void ICM_CS_High(const ICM_Sensor_t *s)
{
    LL_GPIO_SetOutputPin(s->cs_port, s->cs_pin);
}

static uint8_t ICM_BusIndex(const ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1) { return 0U; }
    if (bus == &g_bus_spi5) { return 1U; }
    return 2U;
}

static uint8_t ICM_FindNextHealthy(const ICM_Bus_t *bus, uint8_t from)
{
    uint8_t i;
    for (i = from; i < ICM_SENSORS_PER_BUS; i++)
    {
        if (bus->sensors[i].fault == 0U) { return i; }
    }
    return ICM_SENSORS_PER_BUS;
}

static void ICM_ClearDmaFlags(const ICM_Bus_t *bus)
{
    if (bus == &g_bus_spi1)
    {
        LL_DMA_ClearFlag_TC2(DMA1); LL_DMA_ClearFlag_HT2(DMA1);
        LL_DMA_ClearFlag_TE2(DMA1); LL_DMA_ClearFlag_DME2(DMA1);
        LL_DMA_ClearFlag_FE2(DMA1);
        LL_DMA_ClearFlag_TC3(DMA1); LL_DMA_ClearFlag_HT3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1); LL_DMA_ClearFlag_DME3(DMA1);
        LL_DMA_ClearFlag_FE3(DMA1);
    }
    else if (bus == &g_bus_spi5)
    {
        LL_DMA_ClearFlag_TC2(DMA2); LL_DMA_ClearFlag_HT2(DMA2);
        LL_DMA_ClearFlag_TE2(DMA2); LL_DMA_ClearFlag_DME2(DMA2);
        LL_DMA_ClearFlag_FE2(DMA2);
        LL_DMA_ClearFlag_TC3(DMA2); LL_DMA_ClearFlag_HT3(DMA2);
        LL_DMA_ClearFlag_TE3(DMA2); LL_DMA_ClearFlag_DME3(DMA2);
        LL_DMA_ClearFlag_FE3(DMA2);
    }
    else
    {
        LL_DMA_ClearFlag_TC0(DMA2); LL_DMA_ClearFlag_HT0(DMA2);
        LL_DMA_ClearFlag_TE0(DMA2); LL_DMA_ClearFlag_DME0(DMA2);
        LL_DMA_ClearFlag_FE0(DMA2);
        LL_DMA_ClearFlag_TC1(DMA2); LL_DMA_ClearFlag_HT1(DMA2);
        LL_DMA_ClearFlag_TE1(DMA2); LL_DMA_ClearFlag_DME1(DMA2);
        LL_DMA_ClearFlag_FE1(DMA2);
    }
}

static void ICM_SPI_EnsureDisabled(SPI_TypeDef *spi)
{
    if (LL_SPI_IsEnabled(spi) != 0U)
    {
        LL_SPI_Disable(spi);
        while (LL_SPI_IsEnabled(spi) != 0U) {}
    }
}

static void ICM_SPI_WaitEOT(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_EOT(spi) == 0U) {}
    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);
}

static void ICM_SPI_DrainRx(SPI_TypeDef *spi, uint32_t n)
{
    while (n != 0U)
    {
        while (LL_SPI_IsActiveFlag_RXP(spi) == 0U) {}
        (void)LL_SPI_ReceiveData8(spi);
        n--;
    }
}

static void ICM_DelayUs(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while (cycles > 3U) { __NOP(); cycles -= 3U; }
}

static void ICM_DelayMs(uint32_t ms)
{
    while (ms != 0U) { ICM_DelayUs(1000U); ms--; }
}
