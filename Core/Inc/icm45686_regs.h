#ifndef ICM45686_REGS_H
#define ICM45686_REGS_H

/*
 * icm45686_regs.h
 *
 * Единственный источник истины для всех адресов регистров и битовых масок ICM-45686.
 * Сверено с официальным TDK-драйвером:
 *   tdk-invn-oss/motion.mcu.icm45686.driver
 *   inv_imu_driver_advanced.c  — inv_imu_adv_enable_clkin_rtc()
 *   inv_imu_regmap_le.h        — точные адреса IPREG
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 *  USER BANK 0 — адреса регистров
 * ========================================================================== */

#define ICM45686_REG_PWR_MGMT0          0x10U
#define ICM45686_REG_FIFO_COUNT_0       0x12U
#define ICM45686_REG_FIFO_COUNT_1       0x13U
#define ICM45686_REG_FIFO_DATA          0x14U

#define ICM45686_REG_INT1_CONFIG0       0x16U
#define ICM45686_REG_INT1_CONFIG1       0x17U
#define ICM45686_REG_INT1_CONFIG2       0x18U
#define ICM45686_REG_INT1_STATUS0       0x19U
#define ICM45686_REG_INT1_STATUS1       0x1AU

#define ICM45686_REG_ACCEL_CONFIG0      0x1BU
#define ICM45686_REG_GYRO_CONFIG0       0x1CU

#define ICM45686_REG_FIFO_CONFIG0       0x1DU
#define ICM45686_REG_FIFO_CONFIG1_0     0x1EU
#define ICM45686_REG_FIFO_CONFIG1_1     0x1FU
#define ICM45686_REG_FIFO_CONFIG2       0x20U
#define ICM45686_REG_FIFO_CONFIG3       0x21U
#define ICM45686_REG_FIFO_CONFIG4       0x22U

/* Алиасы под имена из icm45686_spi.c */
#define ICM45686_REG_FIFO_CONFIG10      ICM45686_REG_FIFO_CONFIG1_0
#define ICM45686_REG_FIFO_CONFIG11      ICM45686_REG_FIFO_CONFIG1_1

#define ICM45686_REG_TMST_WOM_CONFIG    0x23U

/* *** CLKIN — User Bank 0 прямые регистры *** */
#define ICM45686_REG_RTC_CONFIG         0x26U
/* ============================================================================
 *  IPREG_TOP1 (base 0xA200) — все через ICM_WriteIReg/ICM_ReadIReg
 * ========================================================================== */
#define ICM45686_IREG_TOP1_ADDR_H              0xA2U

/* IOC_PAD_SCENARIO_OVRD — offset 0x31 */
#define ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L  0x31U
#define ICM45686_PADS_INT2_CFG_OVRD_EN         (1U << 2)
#define ICM45686_PADS_INT2_CFG_CLKIN           0x02U

/* IOC_PAD_AUX_OVRD — offset 0x32 */
#define ICM45686_IREG_IOC_PAD_AUX_OVRD_L       0x32U
#define ICM45686_AUX2_ENABLE_OVRD              (1U << 2)
#define ICM45686_AUX2_ENABLE_VAL_BIT           (1U << 1)

/* SIFS_I3C_STC_CFG — offset 0x52 */
#define ICM45686_IREG_SIFS_I3C_STC_CFG_L       0x52U
#define ICM45686_I3C_STC_MODE_BIT              (1U << 0)

#define ICM45686_REG_REG_MISC1          0x39U
#define ICM45686_REG_REG_MISC2          0x7FU
#define ICM45686_REG_WHO_AM_I           0x72U

#define ICM45686_REG_IREG_ADDR_15_8     0x7CU
#define ICM45686_REG_IREG_ADDR_7_0      0x7DU
#define ICM45686_REG_IREG_DATA          0x7EU

/* ============================================================================
 *  Идентификация / SPI
 * ========================================================================== */

#define ICM45686_WHO_AM_I_VALUE         0xE9U
#define ICM45686_SPI_READ_BIT           0x80U

/* ============================================================================
 *  REG_MISC2 (0x7F)
 *  bit0: IREG_DONE   bit1: SOFT_RST
 * ========================================================================== */

#define ICM45686_MISC2_IREG_DONE        (1U << 0)
#define ICM45686_MISC2_SOFT_RST         (1U << 1)
#define ICM45686_SOFT_RESET             ICM45686_MISC2_SOFT_RST   /* алиас */

/* ============================================================================
 *  PWR_MGMT0 (0x10)
 * ========================================================================== */

#define ICM45686_PWR_GYRO_MODE_OFF      (0x00U << 2)
#define ICM45686_PWR_GYRO_MODE_STANDBY  (0x01U << 2)
#define ICM45686_PWR_GYRO_MODE_LP       (0x02U << 2)
#define ICM45686_PWR_GYRO_MODE_LN       (0x03U << 2)

#define ICM45686_PWR_ACCEL_MODE_OFF     0x00U
#define ICM45686_PWR_ACCEL_MODE_LP      0x02U
#define ICM45686_PWR_ACCEL_MODE_LN      0x03U

/* ============================================================================
 *  ACCEL_CONFIG0 (0x1B)
 * ========================================================================== */

#define ICM45686_ACCEL_FS_32G           (0x00U << 4)
#define ICM45686_ACCEL_FS_16G           (0x01U << 4)
#define ICM45686_ACCEL_FS_8G            (0x02U << 4)
#define ICM45686_ACCEL_FS_4G            (0x03U << 4)
#define ICM45686_ACCEL_FS_2G            (0x04U << 4)

#define ICM45686_ACCEL_ODR_6400HZ       0x03U
#define ICM45686_ACCEL_ODR_3200HZ       0x04U
#define ICM45686_ACCEL_ODR_1600HZ       0x05U
#define ICM45686_ACCEL_ODR_800HZ        0x06U
#define ICM45686_ACCEL_ODR_400HZ        0x07U
#define ICM45686_ACCEL_ODR_200HZ        0x08U
#define ICM45686_ACCEL_ODR_100HZ        0x09U
#define ICM45686_ACCEL_ODR_50HZ         0x0AU
#define ICM45686_ACCEL_ODR_25HZ         0x0BU
#define ICM45686_ACCEL_ODR_12_5HZ       0x0CU

/* ============================================================================
 *  GYRO_CONFIG0 (0x1C)
 * ========================================================================== */

#define ICM45686_GYRO_FS_4000DPS        (0x00U << 4)
#define ICM45686_GYRO_FS_2000DPS        (0x01U << 4)
#define ICM45686_GYRO_FS_1000DPS        (0x02U << 4)
#define ICM45686_GYRO_FS_500DPS         (0x03U << 4)
#define ICM45686_GYRO_FS_250DPS         (0x04U << 4)
#define ICM45686_GYRO_FS_125DPS         (0x05U << 4)
#define ICM45686_GYRO_FS_62_5DPS        (0x06U << 4)
#define ICM45686_GYRO_FS_31_25DPS       (0x07U << 4)

#define ICM45686_GYRO_ODR_6400HZ        0x03U
#define ICM45686_GYRO_ODR_3200HZ        0x04U
#define ICM45686_GYRO_ODR_1600HZ        0x05U
#define ICM45686_GYRO_ODR_800HZ         0x06U
#define ICM45686_GYRO_ODR_400HZ         0x07U
#define ICM45686_GYRO_ODR_200HZ         0x08U
#define ICM45686_GYRO_ODR_100HZ         0x09U
#define ICM45686_GYRO_ODR_50HZ          0x0AU
#define ICM45686_GYRO_ODR_25HZ          0x0BU
#define ICM45686_GYRO_ODR_12_5HZ        0x0CU

/* ============================================================================
 *  FIFO_CONFIG0 (0x1D)
 * ========================================================================== */

#define ICM45686_FIFO_MODE_BYPASS       (0x00U << 6)
#define ICM45686_FIFO_MODE_STREAM       (0x01U << 6)
#define ICM45686_FIFO_MODE_SNAPSHOT     (0x02U << 6)

#define ICM45686_FIFO_DEPTH_MAX         0x1EU
#define ICM45686_FIFO_DEPTH_APEX        0x07U
#define ICM45686_FIFO_DEPTH_GAF         0x04U
#define ICM45686_FIFO_DEPTH_2K          ICM45686_FIFO_DEPTH_MAX   /* алиас */

/* ============================================================================
 *  FIFO_CONFIG2 (0x20)
 * ========================================================================== */

#define ICM45686_FIFO_FLUSH             (1U << 7)
#define ICM45686_FIFO_WM_GT_TH          (1U << 3)

/* ============================================================================
 *  FIFO_CONFIG3 (0x21)
 * ========================================================================== */

#define ICM45686_FIFO_IF_EN             (1U << 5)
#define ICM45686_FIFO_ACCEL_EN          (1U << 0)
#define ICM45686_FIFO_GYRO_EN           (1U << 1)
#define ICM45686_FIFO_HIRES_EN          (1U << 2)

/* ============================================================================
 *  FIFO_CONFIG4 (0x22)
 * ========================================================================== */

#define ICM45686_FIFO_TMST_FSYNC_EN     (1U << 3)
#define ICM45686_FIFO_COMP_EN           (1U << 2)

/* ============================================================================
 *  FIFO header
 * ========================================================================== */

#define ICM45686_FIFO_HEADER_EXT        (1U << 7)
#define ICM45686_FIFO_HEADER_ACCEL_BIT  (1U << 6)
#define ICM45686_FIFO_HEADER_GYRO_BIT   (1U << 5)
#define ICM45686_FIFO_HEADER_HIRES_BIT  (1U << 4)
#define ICM45686_FIFO_HEADER_TMST_BIT   (1U << 3)
#define ICM45686_FIFO_HEADER_FSYNC_BIT  (1U << 2)

#define ICM45686_FIFO_PACKET_SIZE_16BIT 16U
#define ICM45686_FIFO_PACKET_SIZE_HIRES 20U
#define ICM45686_FIFO_SIZE_BYTES        8192U

/* ============================================================================
 *  INT1_CONFIG1 (0x17) / INT1_STATUS1 (0x1A)
 * ========================================================================== */

#define ICM45686_INT1_PLL_RDY_EN        (1U << 0)
#define ICM45686_INT1_STATUS_PLL_RDY    (1U << 0)

/* ============================================================================
 *  RTC_CONFIG (0x26)
 *  bit1: rtc_mode   bit6: rtc_align
 * ========================================================================== */

#define ICM45686_RTC_MODE_EN            (1U << 1)
#define ICM45686_RTC_ALIGN_EN           (1U << 6)

/* ============================================================================
 *  SIFS_I3C_STC_CFG (0x27)
 *  bit0: i3c_stc_mode — ДОЛЖЕН быть 0 перед включением CLKIN
 *  (TDK: "I3CSM STC has higher priority over CLKIN")
 * ========================================================================== */

#define ICM45686_I3C_STC_MODE_BIT       (1U << 0)

/* ============================================================================
 *  IOC_PAD_SCENARIO_OVRD (0x2F) — пин INT2 → CLKIN
 *  bit2:    pads_int2_cfg_ovrd     = 1 (включить override)
 *  bit[1:0]: pads_int2_cfg_ovrd_val
 *    0b00 = INT2  0b01 = FSYNC  0b10 = CLKIN
 *
 *  IOC_PAD_AUX_OVRD (0x30) — отключить AUX2 (освободить пин INT2)
 *  bit2: aux2_enable_ovrd = 1
 *  bit1: aux2_enable_ovrd_val = 0
 * ========================================================================== */

#define ICM45686_PADS_INT2_CFG_OVRD_EN  (1U << 2)
#define ICM45686_PADS_INT2_CFG_CLKIN    0x02U
#define ICM45686_AUX2_ENABLE_OVRD       (1U << 2)
#define ICM45686_AUX2_ENABLE_VAL_BIT    (1U << 1)

/* ============================================================================
 *  REG_MISC1 (0x39) — clk_src_sel bit0
 *  0 = internal RCOSC / PLL   1 = внешний CLKIN
 * ========================================================================== */

#define ICM45686_CLK_SRC_INTERNAL       0x00U
#define ICM45686_CLK_SRC_EXTERNAL       0x01U

/* Алиасы osc_id_ovrd */
#define ICM45686_OSC_ID_OVRD_OFF        0x00U
#define ICM45686_OSC_ID_OVRD_EDOSC      0x01U
#define ICM45686_OSC_ID_OVRD_RCOSC      0x02U
#define ICM45686_OSC_ID_OVRD_PLL        0x04U
#define ICM45686_OSC_ID_OVRD_EXT_CLK    0x08U

/* ============================================================================
 *  IPREG_SYS2_REG_123 — доступ через ICM_ReadIReg/ICM_WriteIReg
 *  addr_h = 0xA5,  addr_l = 0x7B
 *  bits[1:0] = accel_src_ctrl
 *    0b00 = off   0b10 = FIR + interpolator ON  ← нужно для CLKIN
 * ========================================================================== */

#define ICM45686_IPREG_SYS2_ADDR_H              0xA5U
#define ICM45686_IPREG_SYS2_REG_123_ADDR_L      0x7BU
#define ICM45686_ACCEL_SRC_CTRL_MASK            0x03U
#define ICM45686_ACCEL_SRC_CTRL_FIR_INTERP      0x02U

/* ============================================================================
 *  IPREG_SYS1_REG_166 — доступ через ICM_ReadIReg/ICM_WriteIReg
 *  addr_h = 0xA4,  addr_l = 0xA6
 *  bits[1:0] = gyro_src_ctrl
 *    0b00 = off   0b10 = FIR + interpolator ON  ← нужно для CLKIN
 * ========================================================================== */

#define ICM45686_IPREG_SYS1_ADDR_H              0xA4U
#define ICM45686_IPREG_SYS1_REG_166_ADDR_L      0xA6U
#define ICM45686_GYRO_SRC_CTRL_MASK             0x03U
#define ICM45686_GYRO_SRC_CTRL_FIR_INTERP       0x02U

/* ============================================================================
 *  Задержки
 * ========================================================================== */

#define ICM45686_RESET_DELAY_US         2000U
#define ICM45686_IREG_DELAY_US          10U
#define ICM45686_IREG_WAIT_US           ICM45686_IREG_DELAY_US
#define ICM45686_PLL_TIMEOUT_US         10000U
#define ICM45686_PLL_TIMEOUT_MS         20U
#define ICM45686_STARTUP_DELAY_MS       200U

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_REGS_H */
