/* =============================================================================
 * icm45686_spi.h
 *
 * Определения, структуры и прототипы для драйвера ICM-45686 (SPI + DMA).
 * Адреса регистров и битовые маски — строго по даташиту DS-000577 Rev 1.0.
 *
 * USER BANK 0:
 *   Все прямые регистры доступны через ICM_WriteReg / ICM_ReadReg.
 *
 * IREG (Indirect Register):
 *   Доступ через IREG_ADDR_15_8 (0x7C), IREG_ADDR_7_0 (0x7D), IREG_DATA (0x7E).
 *   Банки и их базовые адреса (Section 14, DS-000577):
 *     IMEMSRAM  base = 0x0000
 *     IPREGBAR  base = 0xA000
 *     IPREGTOP1 base = 0xA200
 *     IPREGSYS1 base = 0xA400
 *     IPREGSYS2 base = 0xA500
 * =============================================================================
 */

#ifndef ICM45686_SPI_H
#define ICM45686_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===========================================================================
 *  Топология системы
 * ========================================================================== */
#define ICM_SPI_BUS_COUNT       3U
#define ICM_SENSORS_PER_BUS     6U
#define ICM_TOTAL_SENSORS       (ICM_SPI_BUS_COUNT * ICM_SENSORS_PER_BUS)  /* 18 */

/* ===========================================================================
 *  SPI протокол
 * ========================================================================== */
#define ICM45686_SPI_READ_BIT   0x80U

/* ===========================================================================
 *  USER BANK 0 — прямые регистры (Section 16.1, DS-000577)
 * ========================================================================== */
#define ICM45686_REG_PWR_MGMT0          0x10U   /* GYRO_MODE, ACCEL_MODE        */
#define ICM45686_REG_INT1_CONFIG0       0x16U   /* INT1 config 0                */
#define ICM45686_REG_INT1_CONFIG1       0x17U   /* INT1 config 1 (PLL_RDY en)   */
#define ICM45686_REG_INT1_CONFIG2       0x18U   /* INT1 config 2 (drive/mode)   */
#define ICM45686_REG_INT1_STATUS0       0x19U   /* INT1 status 0                */
#define ICM45686_REG_INT1_STATUS1       0x1AU   /* INT1 status 1 (PLL_RDY)      */
#define ICM45686_REG_ACCEL_CONFIG0      0x1BU   /* ACCEL_UI_FS_SEL, ACCEL_ODR   */
#define ICM45686_REG_GYRO_CONFIG0       0x1CU   /* GYRO_UI_FS_SEL, GYRO_ODR     */
#define ICM45686_REG_FIFO_CONFIG0       0x1DU   /* FIFO_MODE, FIFO_DEPTH        */
#define ICM45686_REG_FIFO_CONFIG10      0x1EU   /* FIFO_WM_TH[7:0]              */
#define ICM45686_REG_FIFO_CONFIG11      0x1FU   /* FIFO_WM_TH[15:8]             */
#define ICM45686_REG_FIFO_CONFIG2       0x20U   /* FIFO_FLUSH                   */
#define ICM45686_REG_FIFO_CONFIG3       0x21U   /* FIFO_ACCEL_EN, FIFO_GYRO_EN  */
#define ICM45686_REG_FIFO_CONFIG4       0x22U   /* FIFO_TMST_FSYNC_EN           */
#define ICM45686_REG_RTC_CONFIG         0x26U   /* RTC_MODE_EN, RTC_ALIGN       */
#define ICM45686_REG_FIFO_DATA          0x14U   /* FIFO burst read              */
#define ICM45686_REG_WHO_AM_I           0x72U   /* WHO_AM_I                     */
#define ICM45686_REG_REG_MISC1          0x39U   /* clk_src_sel (bit0)           */
#define ICM45686_REG_IREG_ADDR_15_8     0x7CU   /* IREG address [15:8]          */
#define ICM45686_REG_IREG_ADDR_7_0      0x7DU   /* IREG address [7:0]           */
#define ICM45686_REG_IREG_DATA          0x7EU   /* IREG data                    */
#define ICM45686_REG_REG_MISC2          0x7FU   /* SOFT_RST(bit1), IREG_DONE(bit0) */

/* ===========================================================================
 *  REG_MISC2 (0x7F) битовые маски  [Section 17, DS-000577]
 * ========================================================================== */
#define ICM45686_MISC2_IREG_DONE        0x01U   /* bit0: IREG операция завершена */
#define ICM45686_MISC2_SOFT_RST         0x02U   /* bit1: Software reset          */

/* ===========================================================================
 *  REG_MISC1 (0x39) — clk_src_sel  [Section 17.51, DS-000577]
 *
 *  clk_src_sel[0] = 0 → внутренний генератор (default)
 *  clk_src_sel[0] = 1 → внешний CLKIN (pin 9)
 * ========================================================================== */
#define ICM45686_CLK_SRC_INTERNAL       0x00U
#define ICM45686_CLK_SRC_EXTERNAL       0x01U   /* Выбор внешнего CLKIN          */

/* ===========================================================================
 *  INT1_CONFIG1 (0x17) битовые маски  [Section 17.22, DS-000577]
 * ========================================================================== */
#define ICM45686_INT1_PLL_RDY_EN        0x01U   /* bit0: разрешить прерывание PLL_RDY */

/* ===========================================================================
 *  INT1_STATUS1 (0x1A) битовые маски  [Section 17.25, DS-000577]
 * ========================================================================== */
#define ICM45686_INT1_STATUS_PLL_RDY    0x01U   /* bit0: PLL захватил внешний клок    */

/* ===========================================================================
 *  PWR_MGMT0 (0x10)  [Section 17.17, DS-000577]
 *
 *  GYRO_MODE  [3:2]: 00=off, 01=standby, 11=LN
 *  ACCEL_MODE [1:0]: 00=off, 01=ULP, 10=LP, 11=LN
 * ========================================================================== */
#define ICM45686_PWR_GYRO_MODE_OFF      0x00U
#define ICM45686_PWR_GYRO_MODE_STANDBY  0x04U
#define ICM45686_PWR_GYRO_MODE_LN       0x0CU
#define ICM45686_PWR_ACCEL_MODE_OFF     0x00U
#define ICM45686_PWR_ACCEL_MODE_LP      0x02U
#define ICM45686_PWR_ACCEL_MODE_LN      0x03U

/* ===========================================================================
 *  ACCEL_CONFIG0 (0x1B) / GYRO_CONFIG0 (0x1C)  [Section 17.26/17.27]
 *
 *  ACCEL_UI_FS_SEL [6:5]: 000=16g 001=8g 010=4g 011=2g 100=32g
 *  ACCEL_ODR       [3:0]: 0110=200Hz 0111=100Hz 1000=50Hz ...
 *  GYRO_UI_FS_SEL  [7:5]: 000=2000dps 001=1000dps 010=500dps ...
 *  GYRO_ODR        [3:0]: 0110=200Hz ...
 * ========================================================================== */
#define ICM_ACCEL_FS_16G                (0x00U << 5)
#define ICM_ACCEL_FS_8G                 (0x01U << 5)
#define ICM_ACCEL_ODR_200HZ             0x06U
#define ICM_GYRO_FS_2000DPS             (0x00U << 5)
#define ICM_GYRO_FS_1000DPS             (0x01U << 5)
#define ICM_GYRO_ODR_200HZ              0x06U

/* Значения по умолчанию для проекта */
#define ICM_ACCEL_FS_VALUE              ICM_ACCEL_FS_16G
#define ICM_ACCEL_ODR_VALUE             ICM_ACCEL_ODR_200HZ
#define ICM_GYRO_FS_VALUE               ICM_GYRO_FS_2000DPS
#define ICM_GYRO_ODR_VALUE              ICM_GYRO_ODR_200HZ

/* ===========================================================================
 *  FIFO_CONFIG0 (0x1D)  [Section 17.28, DS-000577]
 *
 *  FIFO_MODE  [7:6]: 00=bypass 01=stream 10=stop-on-full
 *  FIFO_DEPTH [4:2]: 000=0 001=256B 010=512B 011=1K 100=2K 101=4K 110=8K
 * ========================================================================== */
#define ICM45686_FIFO_MODE_STREAM       (0x01U << 6)
#define ICM45686_FIFO_DEPTH_2K          (0x04U << 2)
#define ICM45686_FIFO_DEPTH_8K          (0x06U << 2)

/* ===========================================================================
 *  FIFO_CONFIG2 (0x20)  [Section 17.31, DS-000577]
 * ========================================================================== */
#define ICM45686_FIFO_FLUSH             0x80U   /* bit7: flush FIFO              */

/* ===========================================================================
 *  FIFO_CONFIG3 (0x21)  [Section 17.32, DS-000577]
 * ========================================================================== */
#define ICM45686_FIFO_ACCEL_EN          0x01U   /* bit0                          */
#define ICM45686_FIFO_GYRO_EN           0x02U   /* bit1                          */
#define ICM45686_FIFO_HIRES_EN          0x04U   /* bit2                          */
#define ICM45686_FIFO_IF_EN             0x20U   /* bit5: FIFO interface enable   */

/* ===========================================================================
 *  FIFO_CONFIG4 (0x22)  [Section 17.33, DS-000577]
 * ========================================================================== */
#define ICM45686_FIFO_TMST_FSYNC_EN     0x08U   /* bit3: timestamp in FIFO       */

/* ===========================================================================
 *  RTC_CONFIG (0x26)  [Section 17.37, DS-000577]
 *
 *  RTC_MODE[1]: 0=disable 1=enable RTC mode
 *  RTC_ALIGN[2]: align RTC to ODR
 * ========================================================================== */
#define ICM45686_RTC_MODE_EN            0x02U   /* bit1                          */

/* ===========================================================================
 *  WHO_AM_I
 * ========================================================================== */
#define ICM45686_WHO_AM_I_VALUE         0xE9U

/* ===========================================================================
 *  IREG — банки и смещения  [Section 16, DS-000577]
 *
 *  Для записи адреса в IREG_ADDR_15_8 / IREG_ADDR_7_0:
 *    IREG_ADDR_15_8 = (base >> 8)   + (offset >> 8)  ← старший байт полного адреса
 *    IREG_ADDR_7_0  = (base & 0xFF) + (offset & 0xFF) ← младший байт полного адреса
 *
 *  IPREGBAR  base = 0xA000  → IREG_ADDR_15_8 = 0xA0
 *  IPREGTOP1 base = 0xA200  → IREG_ADDR_15_8 = 0xA2
 *
 *  IOC_PAD_SCENARIO_OVRD находится в IPREGBAR (Section 16.3):
 *    offset в банке IPREGBAR = 0x04 → полный адрес = 0xA004
 *    IREG_ADDR_15_8 = 0xA0, IREG_ADDR_7_0 = 0x04
 *
 *  IOC_PAD_SCENARIO_AUX_OVRD — IPREGBAR offset = 0x05 → 0xA005
 * ========================================================================== */

/* Старшие байты для IREG_ADDR_15_8 */
#define ICM45686_IREG_IPREGTOP1_ADDR_H      0xA2U   /* база IPREGTOP1 = 0xA200 */
#define ICM45686_IREG_IPREGSYS1_ADDR_H      0xA4U   /* база IPREGSYS1 = 0xA400 */
#define ICM45686_IREG_IPREGSYS2_ADDR_H      0xA5U   /* база IPREGSYS2 = 0xA500 */

/*
 * IOC_PAD_SCENARIO_OVRD (IPREGBAR, offset 0x04)  [Section 16.3, DS-000577]
 *
 *  Этот регистр управляет назначением пина 9 (INT2/FSYNC/CLKIN).
 *
 *  Bits [1:0] = pads_int2_cfg_ovrd_val:
 *    00 = INT2
 *    01 = FSYNC
 *    10 = CLKIN  ← нужно нам
 *  Bit [2]    = pads_int2_cfg_ovrd: 1 = override enable
 *
 *  Значение для CLKIN: bit2=1, bits[1:0]=10 → 0b110 = 0x06
 */
#define ICM45686_REG_IOCPAD_SCENARIO        0x2FU   /* R  — read only статус         */
#define ICM45686_REG_IOCPAD_SCENARIO_AUXOVRD 0x30U  /* RW — управление AUX1/CLKIN    */
#define ICM45686_AUXOVRD_AUX1_DISABLE       0x04U   /* AUX1ENABLEOVRD=1, VAL=0       */

/*
 * IOC_PAD_SCENARIO_AUX_OVRD (IPREGBAR, offset 0x05)  [Section 16.3, DS-000577]
 *  Используется для отключения AUX1 если мешает пину CLKIN.
 *  AUX1_ENABLE_OVRD=1, AUX1_ENABLE_OVRD_VAL=0 → AUX1 disabled
 *  Значение: 0x03 (bits[1:0] = ovrd_enable=1, val=1... )
 *  По умолчанию оставляем как есть (не трогаем), если AUX не используется.
 */
#define ICM45686_IREG_IOC_PAD_AUX_OVRD_L   0x05U

/* ===========================================================================
 *  Тайминги  [Section 14.3, DS-000577]
 *
 *  Minimum wait time-gap = время от CS HIGH после записи IREG_ADDR до
 *  следующей SPI транзакции (CS LOW).
 *  По даташиту: минимум 4 мкс. Используем 10 мкс с запасом.
 * ========================================================================== */
#define ICM45686_IREG_WAIT_US               10U
#define ICM45686_RESET_DELAY_US             2000U   /* 2 мс после soft reset    */
#define ICM45686_STARTUP_DELAY_MS           200U    /* 200 мс startup           */
#define ICM45686_PLL_TIMEOUT_MS             20U     /* 20 мс на захват PLL      */

/* ===========================================================================
 *  FIFO DMA буфер
 * ========================================================================== */

/*
 * Структура пакета FIFO ICM-45686 (Section 6, DS-000577):
 *   1 байт  — header
 *   6 байт  — accel XYZ (2 байта на ось, big-endian по умолчанию little-endian)
 *   6 байт  — gyro  XYZ
 *   2 байта — temperature
 *   2 байта — timestamp
 *   = 17 байт на пакет (без HIRES)
 *
 *  Водяной знак = 1 пакет × количество пакетов за период ODR.
 *  При 200 Гц и опросе с запасом берём 32 пакета.
 *  +1 байт на команду чтения (первый байт TX/RX — адрес регистра).
 */
#define ICM45686_FIFO_PACKET_SIZE           17U
#define ICM45686_FIFO_PACKETS_MAX           32U
#define ICM_FIFO_DMA_BUF_SIZE               (1U + ICM45686_FIFO_PACKET_SIZE * ICM45686_FIFO_PACKETS_MAX)
#define ICM_FIFO_WATERMARK_BYTES            (ICM45686_FIFO_PACKET_SIZE * 8U)  /* 8 пакетов */

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
void     ICM_StartBurstRead_SPI1(void);  /* Вызывается из TIM6 ISR */

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
