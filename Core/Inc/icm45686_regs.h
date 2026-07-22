#ifndef ICM45686_REGS_H
#define ICM45686_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "inv_imu_regmap_le.h"
#include "inv_imu_defs.h"

/* ========================================================================== */
/*                         USER BANK 0                                        */
/* ========================================================================== */

/* Данные и питание */
#define ICM45686_REG_PWR_MGMT0               0x10U
#define ICM45686_REG_FIFO_COUNT_0            0x12U
#define ICM45686_REG_FIFO_COUNT_1            0x13U
#define ICM45686_REG_FIFO_DATA               0x14U

/* INT1 */
#define ICM45686_REG_INT1_CONFIG0            0x16U
#define ICM45686_REG_INT1_CONFIG1            0x17U
#define ICM45686_REG_INT1_CONFIG2            0x18U
#define ICM45686_REG_INT1_STATUS0            0x19U
#define ICM45686_REG_INT1_STATUS1            0x1AU

/* Конфигурация датчиков */
#define ICM45686_REG_ACCEL_CONFIG0           0x1BU
#define ICM45686_REG_GYRO_CONFIG0            0x1CU

/* FIFO */
#define ICM45686_REG_FIFO_CONFIG0            0x1DU
#define ICM45686_REG_FIFO_CONFIG10           0x1EU
#define ICM45686_REG_FIFO_CONFIG11           0x1FU
#define ICM45686_REG_FIFO_CONFIG2            0x20U
#define ICM45686_REG_FIFO_CONFIG3            0x21U
#define ICM45686_REG_FIFO_CONFIG4            0x22U

/* RTC / clock */
#define ICM45686_REG_TMST_WOM_CONFIG         0x23U
#define ICM45686_REG_RTC_CONFIG              0x26U
#define ICM45686_REG_REG_MISC1               0x35U

/* Идентификация и IREG */
#define ICM45686_REG_WHO_AM_I                0x72U
#define ICM45686_REG_IREG_ADDR_15_8          0x7CU
#define ICM45686_REG_IREG_ADDR_7_0           0x7DU
#define ICM45686_REG_IREG_DATA               0x7EU
#define ICM45686_REG_REG_MISC2               0x7FU

#define ICM45686_WHO_AM_I_VALUE              0xE9U
#define ICM45686_SPI_READ_BIT                0x80U

/* ========================================================================== */
/*                         PWR_MGMT0                                           */
/* ========================================================================== */

#define ICM45686_PWR_GYRO_MODE_OFF           (0x00U << 2)
#define ICM45686_PWR_GYRO_MODE_STANDBY       (0x01U << 2)
#define ICM45686_PWR_GYRO_MODE_LP            (0x02U << 2)
#define ICM45686_PWR_GYRO_MODE_LN            (0x03U << 2)

#define ICM45686_PWR_ACCEL_MODE_OFF          0x00U
#define ICM45686_PWR_ACCEL_MODE_LP           0x02U
#define ICM45686_PWR_ACCEL_MODE_LN           0x03U

/* ========================================================================== */
/*                         ACCEL_CONFIG0                                      */
/* ========================================================================== */

/* ACCEL_UI_FS_SEL: биты [6:4] */
#define ICM45686_ACCEL_FS_32G                (0x00U << 4)
#define ICM45686_ACCEL_FS_16G                (0x01U << 4)
#define ICM45686_ACCEL_FS_8G                 (0x02U << 4)
#define ICM45686_ACCEL_FS_4G                 (0x03U << 4)
#define ICM45686_ACCEL_FS_2G                 (0x04U << 4)

/* ACCEL_ODR: биты [3:0] */
#define ICM45686_ACCEL_ODR_6400HZ            0x03U
#define ICM45686_ACCEL_ODR_3200HZ            0x04U
#define ICM45686_ACCEL_ODR_1600HZ            0x05U
#define ICM45686_ACCEL_ODR_800HZ             0x06U
#define ICM45686_ACCEL_ODR_400HZ             0x07U
#define ICM45686_ACCEL_ODR_200HZ             0x08U
#define ICM45686_ACCEL_ODR_100HZ             0x09U
#define ICM45686_ACCEL_ODR_50HZ              0x0AU

/* ========================================================================== */
/*                         GYRO_CONFIG0                                       */
/* ========================================================================== */

/* GYRO_UI_FS_SEL: биты [7:4] */
#define ICM45686_GYRO_FS_4000DPS             (0x00U << 4)
#define ICM45686_GYRO_FS_2000DPS             (0x01U << 4)
#define ICM45686_GYRO_FS_1000DPS             (0x02U << 4)
#define ICM45686_GYRO_FS_500DPS              (0x03U << 4)
#define ICM45686_GYRO_FS_250DPS              (0x04U << 4)
#define ICM45686_GYRO_FS_125DPS              (0x05U << 4)
#define ICM45686_GYRO_FS_62_5DPS             (0x06U << 4)
#define ICM45686_GYRO_FS_31_25DPS            (0x07U << 4)

#define ICM45686_GYRO_ODR_6400HZ             0x03U
#define ICM45686_GYRO_ODR_3200HZ             0x04U
#define ICM45686_GYRO_ODR_1600HZ             0x05U
#define ICM45686_GYRO_ODR_800HZ              0x06U
#define ICM45686_GYRO_ODR_400HZ              0x07U
#define ICM45686_GYRO_ODR_200HZ              0x08U
#define ICM45686_GYRO_ODR_100HZ              0x09U
#define ICM45686_GYRO_ODR_50HZ               0x0AU

/* ========================================================================== */
/*                         FIFO                                                */
/* ========================================================================== */

/* FIFO_CONFIG0 */
#define ICM45686_FIFO_MODE_BYPASS            (0x00U << 6)
#define ICM45686_FIFO_MODE_STREAM            (0x01U << 6)
#define ICM45686_FIFO_MODE_STOP_ON_FULL      (0x02U << 6)
#define ICM45686_FIFO_DEPTH_2K               (0x00U << 4)
#define ICM45686_FIFO_DEPTH_8K               (0x07U << 4)

/* FIFO_CONFIG2 */
#define ICM45686_FIFO_FLUSH                  (1U << 7)
#define ICM45686_FIFO_WM_GREATER_EQUAL       (1U << 3)

/* FIFO_CONFIG3 */
#define ICM45686_FIFO_IF_EN                  (1U << 0)
#define ICM45686_FIFO_ACCEL_EN               (1U << 1)
#define ICM45686_FIFO_GYRO_EN                (1U << 2)
#define ICM45686_FIFO_HIRES_EN               (1U << 3)

/* FIFO_CONFIG4 */
#define ICM45686_FIFO_COMP_EN                (1U << 2)
#define ICM45686_FIFO_TMST_FSYNC_EN          (1U << 1)

/* FIFO header */
#define ICM45686_FIFO_HEADER_ACCEL_BIT       (1U << 6)
#define ICM45686_FIFO_HEADER_GYRO_BIT        (1U << 5)
#define ICM45686_FIFO_HEADER_HIRES_BIT       (1U << 4)
#define ICM45686_FIFO_HEADER_TMST_BIT        (1U << 3)
#define ICM45686_FIFO_HEADER_FSYNC_BIT       (1U << 2)

#define ICM45686_FIFO_PACKET_SIZE_16BIT      16U
#define ICM45686_FIFO_PACKET_SIZE_HIRES      20U
#define ICM45686_FIFO_SIZE_BYTES             2048U

/* ========================================================================== */
/*                         Reset / external clock                             */
/* ========================================================================== */

/* REG_MISC2 */
#define ICM45686_SOFT_RESET                  (1U << 1)

/* REG_MISC1, поле OSC_ID_OVRD[3:0] */
#define ICM45686_OSC_ID_OVRD_DEFAULT         0x00U
#define ICM45686_OSC_ID_OVRD_RC              0x02U
#define ICM45686_OSC_ID_OVRD_EXT_CLK         0x08U

/* INT1_CONFIG1 / INT1_STATUS1 */
#define ICM45686_INT1_PLL_RDY_EN             (1U << 0)
#define ICM45686_INT1_STATUS_PLL_RDY         (1U << 0)

/* RTC_CONFIG */
#define ICM45686_RTC_MODE_EN                 (1U << 5)
#define ICM45686_RTC_ALIGN_EN                (1U << 6)

/* ========================================================================== */
/*                         Косвенный доступ IREG                              */
/* ========================================================================== */

/*
 * IPREG_TOP1 имеет базовый адрес 0xA200.
 * IOC_PAD_SCENARIO_OVRD: 0xA230.
 * SMC_CONTROL_0:          0xA258.
 */
#define ICM45686_IREG_TOP1_ADDR_H             0xA2U

#define ICM45686_IREG_IOC_PAD_SCENARIO_OVRD_L 0x30U
#define ICM45686_IREG_SMC_CONTROL_0_L         0x58U

/* IOC_PAD_SCENARIO_OVRD: включить функцию CLKIN на pin 9 */
#define ICM45686_CLKIN_ENABLE_VAL             0x06U

/*
 * SMC_CONTROL_0:
 * bit 4: ACCEL_LP_CLK_SEL;
 * bit 0: TMST_EN.
 */
#define ICM45686_SMC_ACCEL_LP_CLK_SEL         (1U << 4)
#define ICM45686_SMC_TMST_EN                  (1U << 0)
#define ICM45686_SMC_CONTROL_0_VALUE \
    (ICM45686_SMC_ACCEL_LP_CLK_SEL | ICM45686_SMC_TMST_EN)

/* ========================================================================== */
/*                         Задержки                                           */
/* ========================================================================== */

#define ICM45686_RESET_DELAY_US               2000U
#define ICM45686_IREG_DELAY_US                10U
#define ICM45686_PLL_TIMEOUT_US               10000U
#define ICM45686_STARTUP_DELAY_MS             200U

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_REGS_H */
