#ifndef ICM45686_REGS_H
#define ICM45686_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 *  USER BANK 0 — прямые регистры
 * ========================================================================== */
#define ICM45686_REG_PWR_MGMT0              0x10U
#define ICM45686_REG_FIFO_COUNT_0           0x12U
#define ICM45686_REG_FIFO_COUNT_1           0x13U
#define ICM45686_REG_FIFO_DATA              0x14U
#define ICM45686_REG_INT1_CONFIG0           0x16U
#define ICM45686_REG_INT1_CONFIG1           0x17U
#define ICM45686_REG_INT1_CONFIG2           0x18U
#define ICM45686_REG_INT1_STATUS0           0x19U
#define ICM45686_REG_INT1_STATUS1           0x1AU
#define ICM45686_REG_ACCEL_CONFIG0          0x1BU
#define ICM45686_REG_GYRO_CONFIG0           0x1CU
#define ICM45686_REG_FIFO_CONFIG0           0x1DU
#define ICM45686_REG_FIFO_CONFIG1_0         0x1EU
#define ICM45686_REG_FIFO_CONFIG1_1         0x1FU
#define ICM45686_REG_FIFO_CONFIG2           0x20U
#define ICM45686_REG_FIFO_CONFIG3           0x21U
#define ICM45686_REG_FIFO_CONFIG4           0x22U
#define ICM45686_REG_RTC_CONFIG             0x26U

/*
 * IOC_PAD_SCENARIO_AUX_OVRD (0x30)
 *   bit1 = aux1_enable_ovrd, bit0 = aux1_enable_ovrd_val
 */
#define ICM45686_REG_IOC_PAD_AUX_OVRD      0x30U
#define ICM45686_AUX1_ENABLE_OVRD          (1U << 1)
#define ICM45686_AUX1_ENABLE_OVRD_VAL      (1U << 0)

/*
 * IOC_PAD_SCENARIO_OVRD (0x31)
 *   bits[1:0] = pads_int2_cfg_ovrd_val (0b10 = CLKIN), bit2 = pads_int2_cfg_ovrd
 */
#define ICM45686_REG_IOC_PAD_SCENARIO_OVRD 0x31U
#define ICM45686_INT2_CFG_OVRD_EN          (1U << 2)
#define ICM45686_INT2_CFG_CLKIN_VAL        0x02U

#define ICM45686_REG_WHO_AM_I               0x72U
#define ICM45686_REG_IREG_ADDR_15_8         0x7CU
#define ICM45686_REG_IREG_ADDR_7_0          0x7DU
#define ICM45686_REG_IREG_DATA              0x7EU
#define ICM45686_REG_REG_MISC2              0x7FU

/* ============================================================================
 *  IREG — Internal Registers
 * ========================================================================== */

/* SIFS_I3C_STC_CFG — IREG 0xA268, bit2 = i3c_stc_mode */
#define ICM45686_IREG_I3C_STC_CFG_H        0xA2U
#define ICM45686_IREG_I3C_STC_CFG_L        0x68U
#define ICM45686_I3C_STC_MODE_BIT          (1U << 2)

/* IPREG_SYS2_REG_123 — IREG 0xA57B, bits[1:0] = accel_src_ctrl */
#define ICM45686_IREG_ACCEL_SRC_CTRL_H     0xA5U
#define ICM45686_IREG_ACCEL_SRC_CTRL_L     0x7BU
#define ICM45686_ACCEL_SRC_FIR_INTERP      0x02U

/* IPREG_SYS1_REG_166 — IREG 0xA4A6, bits[6:5] = gyro_src_ctrl */
#define ICM45686_IREG_GYRO_SRC_CTRL_H      0xA4U
#define ICM45686_IREG_GYRO_SRC_CTRL_L      0xA6U
#define ICM45686_GYRO_SRC_FIR_INTERP       0x02U
#define ICM45686_GYRO_SRC_CTRL_MASK        0x60U
#define ICM45686_GYRO_SRC_CTRL_SHIFT       5U

/*
 * SMC_CONTROL_0 — IREG 0xA258
 *   bit0 = tmst_en       — ГЛАВНЫЙ включатель timestamp-счётчика.
 *                          Без него счётчик не тикает и timestamp
 *                          в каждом FIFO-пакете = 0x0000.
 *   bit1 = tmst_fsync_en — привязка к FSYNC (не используем)
 *
 *  FIFO_CONFIG4.fifo_tmst_fsync_en (0x22, bit1) только разрешает
 *  класть timestamp в FIFO-пакет, НЕ запускает сам счётчик!
 */
#define ICM45686_IREG_SMC_CONTROL_0_H      0xA2U
#define ICM45686_IREG_SMC_CONTROL_0_L      0x58U
#define ICM45686_TMST_EN                   (1U << 0)
#define ICM45686_TMST_FSYNC_EN_IREG        (1U << 1)

/* ============================================================================
 *  SPI / ID
 * ========================================================================== */
#define ICM45686_WHO_AM_I_VALUE             0xE9U
#define ICM45686_SPI_READ_BIT               0x80U

/* ============================================================================
 *  REG_MISC2 (0x7F)
 * ========================================================================== */
#define ICM45686_MISC2_IREG_DONE            (1U << 0)
#define ICM45686_MISC2_SOFT_RST             (1U << 1)

/* ============================================================================
 *  PWR_MGMT0 (0x10)
 * ========================================================================== */
#define ICM45686_PWR_GYRO_MODE_OFF          (0x00U << 2)
#define ICM45686_PWR_GYRO_MODE_STANDBY      (0x01U << 2)
#define ICM45686_PWR_GYRO_MODE_LP           (0x02U << 2)
#define ICM45686_PWR_GYRO_MODE_LN           (0x03U << 2)
#define ICM45686_PWR_ACCEL_MODE_OFF         0x00U
#define ICM45686_PWR_ACCEL_MODE_LP          0x02U
#define ICM45686_PWR_ACCEL_MODE_LN          0x03U

/* ============================================================================
 *  ACCEL_CONFIG0 (0x1B)
 *  bits[3:0] = accel_odr, bits[6:4] = accel_ui_fs_sel
 * ========================================================================== */
#define ICM45686_ACCEL_FS_32G               (0x00U << 4)
#define ICM45686_ACCEL_FS_16G               (0x01U << 4)
#define ICM45686_ACCEL_FS_8G                (0x02U << 4)
#define ICM45686_ACCEL_FS_4G                (0x03U << 4)
#define ICM45686_ACCEL_FS_2G                (0x04U << 4)
#define ICM45686_ACCEL_ODR_6400HZ           0x03U
#define ICM45686_ACCEL_ODR_3200HZ           0x04U
#define ICM45686_ACCEL_ODR_1600HZ           0x05U
#define ICM45686_ACCEL_ODR_800HZ            0x06U
#define ICM45686_ACCEL_ODR_400HZ            0x07U
#define ICM45686_ACCEL_ODR_200HZ            0x08U
#define ICM45686_ACCEL_ODR_100HZ            0x09U
#define ICM45686_ACCEL_ODR_50HZ             0x0AU

/* ============================================================================
 *  GYRO_CONFIG0 (0x1C)
 *  bits[3:0] = gyro_odr, bits[7:4] = gyro_ui_fs_sel
 * ========================================================================== */
#define ICM45686_GYRO_FS_4000DPS            (0x00U << 4)
#define ICM45686_GYRO_FS_2000DPS            (0x01U << 4)
#define ICM45686_GYRO_FS_1000DPS            (0x02U << 4)
#define ICM45686_GYRO_FS_500DPS             (0x03U << 4)
#define ICM45686_GYRO_FS_250DPS             (0x04U << 4)
#define ICM45686_GYRO_FS_125DPS             (0x05U << 4)
#define ICM45686_GYRO_FS_62_5DPS            (0x06U << 4)
#define ICM45686_GYRO_FS_31_25DPS           (0x07U << 4)
#define ICM45686_GYRO_ODR_6400HZ            0x03U
#define ICM45686_GYRO_ODR_3200HZ            0x04U
#define ICM45686_GYRO_ODR_1600HZ            0x05U
#define ICM45686_GYRO_ODR_800HZ             0x06U
#define ICM45686_GYRO_ODR_400HZ             0x07U
#define ICM45686_GYRO_ODR_200HZ             0x08U
#define ICM45686_GYRO_ODR_100HZ             0x09U
#define ICM45686_GYRO_ODR_50HZ              0x0AU

/* ============================================================================
 *  FIFO_CONFIG0 (0x1D)
 * ========================================================================== */
#define ICM45686_FIFO_MODE_BYPASS           (0x00U << 6)
#define ICM45686_FIFO_MODE_STREAM           (0x01U << 6)
#define ICM45686_FIFO_MODE_SNAPSHOT         (0x02U << 6)
#define ICM45686_FIFO_DEPTH_MAX             0x1EU

/* ============================================================================
 *  FIFO_CONFIG2 (0x20)
 * ========================================================================== */
#define ICM45686_FIFO_FLUSH                 (1U << 7)

/* ============================================================================
 *  FIFO_CONFIG3 (0x21)
 * ========================================================================== */
#define ICM45686_FIFO_IF_EN                 (1U << 0)
#define ICM45686_FIFO_ACCEL_EN              (1U << 1)
#define ICM45686_FIFO_GYRO_EN               (1U << 2)
#define ICM45686_FIFO_HIRES_EN              (1U << 3)

/* ============================================================================
 *  FIFO Header bits
 *   bit7 = MSG   — пустой пакет-заглушка
 *   bit6 = ACCEL — данные акселерометра валидны
 *   bit5 = GYRO  — данные гироскопа валидны
 * ========================================================================== */
#define ICM45686_FIFO_HEADER_MSG_BIT        (1U << 7)
#define ICM45686_FIFO_HEADER_ACCEL_BIT      (1U << 6)
#define ICM45686_FIFO_HEADER_GYRO_BIT       (1U << 5)

/* ============================================================================
 *  FIFO_CONFIG4 (0x22)
 *  bit1 = fifo_tmst_fsync_en — разрешает класть timestamp в FIFO-пакет.
 *         ВНИМАНИЕ: требует также tmst_en=1 в SMC_CONTROL_0 (IREG 0xA258)!
 * ========================================================================== */
#define ICM45686_FIFO_TMST_FSYNC_EN         (1U << 1)

/* ============================================================================
 *  FIFO пакет (16-bit режим, Little-Endian — LSB по меньшему адресу)
 *  byte  0     = Header
 *  bytes 1-2   = Accel X  [1]=LSB, [2]=MSB
 *  bytes 3-4   = Accel Y  [3]=LSB, [4]=MSB
 *  bytes 5-6   = Accel Z  [5]=LSB, [6]=MSB
 *  bytes 7-8   = Gyro  X  [7]=LSB, [8]=MSB
 *  bytes 9-10  = Gyro  Y  [9]=LSB, [10]=MSB
 *  bytes 11-12 = Gyro  Z  [11]=LSB,[12]=MSB
 *  byte  13    = Temp
 *  byte  14    = Timestamp LSB
 *  byte  15    = Timestamp MSB
 * ========================================================================== */
#define ICM45686_FIFO_PACKET_SIZE_16BIT     20U
#define ICM45686_FIFO_PACKET_SIZE_HIRES     20U

/* ============================================================================
 *  RTC_CONFIG (0x26)
 * ========================================================================== */
#define ICM45686_RTC_MODE_EN                (1U << 5)
#define ICM45686_RTC_ALIGN_EN               (1U << 6)

/* ============================================================================
 *  Задержки
 * ========================================================================== */
#define ICM45686_RESET_DELAY_US             2000U
#define ICM45686_IREG_WAIT_US               10U
#define ICM45686_STARTUP_DELAY_MS           200U

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_REGS_H */
