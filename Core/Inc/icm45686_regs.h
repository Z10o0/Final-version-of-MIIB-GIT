#ifndef ICM45686_REGS_H
#define ICM45686_REGS_H

/*
 * icm45686_regs.h
 *
 * Единственный источник истины для всех адресов регистров и битовых масок ICM-45686.
 * Все значения сверены с официальным драйвером TDK:
 *   tdk-invn-oss/motion.mcu.icm45686.driver (icm45686/imu/inv_imu_defs.h)
 *   и регистровой картой inv_imu_regmap_le.h.
 *
 * НЕ переопределяйте эти макросы в других хедерах — используйте только этот файл.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 *  USER BANK 0 — адреса регистров
 * ========================================================================== */

/* Питание и данные */
#define ICM45686_REG_PWR_MGMT0          0x10U
#define ICM45686_REG_FIFO_COUNT_0       0x12U   /* FIFO_COUNT[7:0]              */
#define ICM45686_REG_FIFO_COUNT_1       0x13U   /* FIFO_COUNT[15:8]             */
#define ICM45686_REG_FIFO_DATA          0x14U   /* чтение FIFO через SPI        */

/* INT1 */
#define ICM45686_REG_INT1_CONFIG0       0x16U
#define ICM45686_REG_INT1_CONFIG1       0x17U
#define ICM45686_REG_INT1_CONFIG2       0x18U
#define ICM45686_REG_INT1_STATUS0       0x19U
#define ICM45686_REG_INT1_STATUS1       0x1AU

/* Конфигурация датчиков */
#define ICM45686_REG_ACCEL_CONFIG0      0x1BU
#define ICM45686_REG_GYRO_CONFIG0       0x1CU

/* FIFO */
#define ICM45686_REG_FIFO_CONFIG0       0x1DU
#define ICM45686_REG_FIFO_CONFIG1_0     0x1EU   /* watermark low byte           */
#define ICM45686_REG_FIFO_CONFIG1_1     0x1FU   /* watermark high byte          */
#define ICM45686_REG_FIFO_CONFIG2       0x20U
#define ICM45686_REG_FIFO_CONFIG3       0x21U
#define ICM45686_REG_FIFO_CONFIG4       0x22U

/* Timestamp / WOM */
#define ICM45686_REG_TMST_WOM_CONFIG    0x23U

/* RTC / clock */
#define ICM45686_REG_RTC_CONFIG         0x26U

/* Misc — ВНИМАНИЕ: правильный адрес 0x39, а НЕ 0x35 */
#define ICM45686_REG_REG_MISC1          0x39U   /* osc_id_ovrd[3:0]             */
#define ICM45686_REG_REG_MISC2          0x7FU   /* soft reset                   */

/* Идентификация */
#define ICM45686_REG_WHO_AM_I           0x72U

/* Косвенный доступ IREG */
#define ICM45686_REG_IREG_ADDR_15_8     0x7CU
#define ICM45686_REG_IREG_ADDR_7_0      0x7DU
#define ICM45686_REG_IREG_DATA          0x7EU

/* ============================================================================
 *  Идентификация
 * ========================================================================== */

#define ICM45686_WHO_AM_I_VALUE         0xE9U
#define ICM45686_SPI_READ_BIT           0x80U   /* ORed с адресом при чтении    */

/* ============================================================================
 *  PWR_MGMT0 (0x10)
 *  gyro_mode[3:2]  accel_mode[1:0]
 * ========================================================================== */

/* gyro_mode — биты [3:2] */
#define ICM45686_PWR_GYRO_MODE_OFF      (0x00U << 2)   /* 0x00 */
#define ICM45686_PWR_GYRO_MODE_STANDBY  (0x01U << 2)   /* 0x04 */
#define ICM45686_PWR_GYRO_MODE_LP       (0x02U << 2)   /* 0x08 */
#define ICM45686_PWR_GYRO_MODE_LN       (0x03U << 2)   /* 0x0C */

/* accel_mode — биты [1:0] */
#define ICM45686_PWR_ACCEL_MODE_OFF     0x00U
#define ICM45686_PWR_ACCEL_MODE_LP      0x02U
#define ICM45686_PWR_ACCEL_MODE_LN      0x03U

/* ============================================================================
 *  ACCEL_CONFIG0 (0x1B)
 *  accel_ui_fs_sel[6:4]  accel_odr[3:0]
 * ========================================================================== */

/* accel_ui_fs_sel — биты [6:4] */
#define ICM45686_ACCEL_FS_32G           (0x00U << 4)   /* 0x00 */
#define ICM45686_ACCEL_FS_16G           (0x01U << 4)   /* 0x10 */
#define ICM45686_ACCEL_FS_8G            (0x02U << 4)   /* 0x20 */
#define ICM45686_ACCEL_FS_4G            (0x03U << 4)   /* 0x30 */
#define ICM45686_ACCEL_FS_2G            (0x04U << 4)   /* 0x40 */

/* accel_odr — биты [3:0] */
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
 *  gyro_ui_fs_sel[7:4]  gyro_odr[3:0]
 * ========================================================================== */

/* gyro_ui_fs_sel — биты [7:4] */
#define ICM45686_GYRO_FS_4000DPS        (0x00U << 4)   /* 0x00 */
#define ICM45686_GYRO_FS_2000DPS        (0x01U << 4)   /* 0x10 */
#define ICM45686_GYRO_FS_1000DPS        (0x02U << 4)   /* 0x20 */
#define ICM45686_GYRO_FS_500DPS         (0x03U << 4)   /* 0x30 */
#define ICM45686_GYRO_FS_250DPS         (0x04U << 4)   /* 0x40 */
#define ICM45686_GYRO_FS_125DPS         (0x05U << 4)   /* 0x50 */
#define ICM45686_GYRO_FS_62_5DPS        (0x06U << 4)   /* 0x60 */
#define ICM45686_GYRO_FS_31_25DPS       (0x07U << 4)   /* 0x70 */

/* gyro_odr — биты [3:0] */
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
 *  fifo_mode[7:6]  fifo_depth[5:0]
 *
 *  TDK errata AN-000364: для максимальной глубины использовать 0x1E, не 0x07.
 * ========================================================================== */

/* fifo_mode — биты [7:6] */
#define ICM45686_FIFO_MODE_BYPASS       (0x00U << 6)   /* 0x00 */
#define ICM45686_FIFO_MODE_STREAM       (0x01U << 6)   /* 0x40 */
#define ICM45686_FIFO_MODE_SNAPSHOT     (0x02U << 6)   /* 0x80 */

/* fifo_depth — биты [5:0]  (TDK errata AN-000364) */
#define ICM45686_FIFO_DEPTH_MAX         0x1EU          /* 8 Кбайт (errata)       */
#define ICM45686_FIFO_DEPTH_APEX        0x07U          /* для APEX               */
#define ICM45686_FIFO_DEPTH_GAF         0x04U          /* для GAF                */

/* ============================================================================
 *  FIFO_CONFIG2 (0x20)
 *  fifo_flush[7]  fifo_wr_wm_gt_th[3]
 * ========================================================================== */

#define ICM45686_FIFO_FLUSH             (1U << 7)      /* bit7: сброс FIFO        */
#define ICM45686_FIFO_WM_GT_TH          (1U << 3)      /* bit3: WM ≥ порог        */

/* ============================================================================
 *  FIFO_CONFIG3 (0x21)
 *
 *  bit5: fifo_if_en
 *  bit0: fifo_accel_en
 *  bit1: fifo_gyro_en
 *  bit2: fifo_hires_en
 *
 *  ВНИМАНИЕ: в старом icm45686_regs.h биты были сдвинуты — ИСПРАВЛЕНО.
 * ========================================================================== */

#define ICM45686_FIFO_IF_EN             (1U << 5)      /* 0x20 bit5               */
#define ICM45686_FIFO_ACCEL_EN          (1U << 0)      /* 0x01 bit0               */
#define ICM45686_FIFO_GYRO_EN           (1U << 1)      /* 0x02 bit1               */
#define ICM45686_FIFO_HIRES_EN          (1U << 2)      /* 0x04 bit2               */

/* ============================================================================
 *  FIFO_CONFIG4 (0x22)
 *  bit3: fifo_tmst_fsync_en
 *  bit2: fifo_comp_en
 * ========================================================================== */

#define ICM45686_FIFO_TMST_FSYNC_EN     (1U << 3)      /* 0x08 bit3               */
#define ICM45686_FIFO_COMP_EN           (1U << 2)      /* 0x04 bit2               */

/* ============================================================================
 *  FIFO header bits (первый байт каждого пакета)
 * ========================================================================== */

#define ICM45686_FIFO_HEADER_EXT        (1U << 7)
#define ICM45686_FIFO_HEADER_ACCEL_BIT  (1U << 6)
#define ICM45686_FIFO_HEADER_GYRO_BIT   (1U << 5)
#define ICM45686_FIFO_HEADER_HIRES_BIT  (1U << 4)
#define ICM45686_FIFO_HEADER_TMST_BIT   (1U << 3)
#define ICM45686_FIFO_HEADER_FSYNC_BIT  (1U << 2)

/* Размеры FIFO пакетов */
#define ICM45686_FIFO_PACKET_SIZE_16BIT 16U
#define ICM45686_FIFO_PACKET_SIZE_HIRES 20U
#define ICM45686_FIFO_SIZE_BYTES        8192U

/* ============================================================================
 *  INT1_CONFIG1 (0x17) / INT1_STATUS1 (0x1A)
 * ========================================================================== */

#define ICM45686_INT1_PLL_RDY_EN        (1U << 0)      /* bit0: разрешить PLL_RDY IRQ  */
#define ICM45686_INT1_STATUS_PLL_RDY    (1U << 0)      /* bit0: PLL захватил клок      */

/* ============================================================================
 *  RTC_CONFIG (0x26)
 *
 *  ВНИМАНИЕ: в старом icm45686_regs.h RTC_MODE_EN был (1U<<5) = 0x20 — НЕВЕРНО.
 *  Правильно: bit1 = 0x02  (TDK inv_imu_driver.c: rtc_config.rtc_mode_en = 1)
 * ========================================================================== */

#define ICM45686_RTC_MODE_EN            (1U << 1)      /* 0x02 bit1               */
#define ICM45686_RTC_ALIGN_EN           (1U << 6)      /* 0x40 bit6               */

/* ============================================================================
 *  REG_MISC1 (0x39) — osc_id_ovrd[3:0]
 *  (TDK: reg_misc1_osc_id_ovrd_t)
 * ========================================================================== */

#define ICM45686_OSC_ID_OVRD_OFF        0x00U
#define ICM45686_OSC_ID_OVRD_EDOSC      0x01U
#define ICM45686_OSC_ID_OVRD_RCOSC      0x02U
#define ICM45686_OSC_ID_OVRD_PLL        0x04U
#define ICM45686_OSC_ID_OVRD_EXT_CLK    0x08U          /* внешний тактовый сигнал */

/* ============================================================================
 *  REG_MISC2 (0x7F)
 * ========================================================================== */

#define ICM45686_SOFT_RESET             (1U << 1)      /* 0x02 soft reset         */

/* ============================================================================
 *  Косвенный доступ IREG — IPREG_TOP1
 *
 *  Адрес: 0xA200 + смещение.
 *  IOC_PAD_SCENARIO_OVRD: 0xA231 (H=0xA2, L=0x31).
 *  SMC_CONTROL_0:          0xA258 (H=0xA2, L=0x58).
 * ========================================================================== */

#define ICM45686_IREG_TOP1_ADDR_H               0xA2U

#define ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L   0x31U
#define ICM45686_IREG_SMC_CONTROL_0_L           0x58U

/*
 * IOC_PAD_SCENARIO_OVRD:
 * pads_int2_cfg_ovrd_val[1:0] = 2 → CLKIN на пине INT2
 * pads_int2_cfg_ovrd_en = 1   → bit2
 * Итоговое значение: 0x06
 */
#define ICM45686_CLKIN_ENABLE_VAL               0x06U

/*
 * SMC_CONTROL_0:
 * bit0: tmst_en
 * bit4: accel_lp_clk_sel (1=RCOSC)
 */
#define ICM45686_SMC_TMST_EN                    (1U << 0)  /* 0x01 */
#define ICM45686_SMC_ACCEL_LP_CLK_SEL           (1U << 4)  /* 0x10 */
#define ICM45686_SMC_CONTROL_0_VALUE \
    (ICM45686_SMC_ACCEL_LP_CLK_SEL | ICM45686_SMC_TMST_EN) /* 0x11 */

/* ============================================================================
 *  Задержки
 * ========================================================================== */

#define ICM45686_RESET_DELAY_US         2000U
#define ICM45686_IREG_DELAY_US          10U
#define ICM45686_PLL_TIMEOUT_US         10000U
#define ICM45686_STARTUP_DELAY_MS       200U

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_REGS_H */
