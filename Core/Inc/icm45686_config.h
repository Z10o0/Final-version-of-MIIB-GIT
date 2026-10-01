#ifndef ICM45686_CONFIG_H
#define ICM45686_CONFIG_H

#include "icm45686_regs.h"

/* ============================================================================
 * Топология шин
 * ========================================================================== */
#define ICM_SPI_BUS_COUNT     6U
#define ICM_SENSORS_PER_BUS   6U
#define ICM_TOTAL_SENSORS \
    (ICM_SPI_BUS_COUNT * ICM_SENSORS_PER_BUS) /* 36 */

/* ============================================================================
 * Единственный источник частот acquisition/output
 * ========================================================================== */
#define ICM_SENSOR_ODR_HZ     3200U
#define ICM_OUTPUT_RATE_HZ    400U

#define ICM_DECIMATION_FACTOR \
    (ICM_SENSOR_ODR_HZ / ICM_OUTPUT_RATE_HZ)

#define ICM_GYRO_ODR_VALUE    ICM45686_GYRO_ODR_3200HZ
#define ICM_ACCEL_ODR_VALUE   ICM45686_ACCEL_ODR_3200HZ

#define ICM_GYRO_FS_VALUE     ICM45686_GYRO_FS_4000DPS
#define ICM_ACCEL_FS_VALUE    ICM45686_ACCEL_FS_32G

/* ============================================================================
 * FIFO: один raw-батч формирует один выходной отсчёт
 * ========================================================================== */
#define ICM_FIFO_POLL_PACKETS       ICM_DECIMATION_FACTOR
#define ICM_FIFO_SAMPLES_PER_READ   ICM_FIFO_POLL_PACKETS

#define ICM_FIFO_PACKET_BYTES \
    ICM45686_FIFO_PACKET_SIZE_HIRES

#define ICM_FIFO_PAYLOAD_BYTES \
    (ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES)

#define ICM_FIFO_DMA_BUF_SIZE \
    (ICM_FIFO_PAYLOAD_BYTES + 1U)

#define ICM_FIFO_WATERMARK_PACKETS  ICM_FIFO_POLL_PACKETS
#define ICM_FIFO_WATERMARK_BYTES    ICM_FIFO_PAYLOAD_BYTES

#define ICM_POLL_RATE_HZ            ICM_OUTPUT_RATE_HZ

/* ============================================================================
 * Готовые маски для записи в регистры
 * ========================================================================== */

/* FIFO_CONFIG3: FIFO interface, accel, gyro, HIRES. */
#define ICM_FIFO_CONFIG3_MASK \
    (ICM45686_FIFO_IF_EN | ICM45686_FIFO_ACCEL_EN | \
     ICM45686_FIFO_GYRO_EN | ICM45686_FIFO_HIRES_EN)

/* FIFO_CONFIG4: timestamp/FSYNC. */
#define ICM_FIFO_CONFIG4_MASK \
    ICM45686_FIFO_TMST_FSYNC_EN

/* PWR_MGMT0: gyro Low Noise + accel Low Noise. */
#define ICM_PWR_MGMT0_MASK \
    (ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN)

/* ============================================================================
 * Compile-time invariants
 * ========================================================================== */
#if (ICM_OUTPUT_RATE_HZ == 0U)
#error "ICM output rate must be nonzero"
#endif

#if ((ICM_SENSOR_ODR_HZ % ICM_OUTPUT_RATE_HZ) != 0U)
#error "ICM sensor ODR must be an integer multiple of output rate"
#endif

#ifdef __cplusplus

static_assert(ICM_DECIMATION_FACTOR == 8U,
              "3200 Hz to 400 Hz requires decimation factor 8");

static_assert(ICM_FIFO_PACKET_BYTES == 20U,
              "ICM45686 HIRES packet size must be 20 bytes");

static_assert(ICM_FIFO_PAYLOAD_BYTES == 160U,
              "Eight FIFO packets must occupy 160 bytes");

static_assert(ICM_FIFO_DMA_BUF_SIZE == 161U,
              "DMA transaction must include one command byte plus 160 payload bytes");

#else

_Static_assert(ICM_DECIMATION_FACTOR == 8U,
               "3200 Hz to 400 Hz requires decimation factor 8");

_Static_assert(ICM_FIFO_PACKET_BYTES == 20U,
               "ICM45686 HIRES packet size must be 20 bytes");

_Static_assert(ICM_FIFO_PAYLOAD_BYTES == 160U,
               "Eight FIFO packets must occupy 160 bytes");

_Static_assert(ICM_FIFO_DMA_BUF_SIZE == 161U,
               "DMA transaction must include one command byte plus 160 payload bytes");

#endif

#endif /* ICM45686_CONFIG_H */
