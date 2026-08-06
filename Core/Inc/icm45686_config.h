#ifndef ICM45686_CONFIG_H
#define ICM45686_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "icm45686_regs.h"

/* ============================================================================
 *  Топология шин
 * ========================================================================== */
#define ICM_SPI_BUS_COUNT                   3U
#define ICM_SENSORS_PER_BUS                 6U

/* ============================================================================
 *  ODR и FSR — 6400 Гц
 * ========================================================================== */
#define ICM_GYRO_ODR_VALUE                  ICM45686_GYRO_ODR_6400HZ   /* 0x03 */
#define ICM_ACCEL_ODR_VALUE                 ICM45686_ACCEL_ODR_6400HZ  /* 0x03 */
#define ICM_GYRO_FS_VALUE                   ICM45686_GYRO_FS_2000DPS   /* 0x10 */
#define ICM_ACCEL_FS_VALUE                  ICM45686_ACCEL_FS_16G      /* 0x10 */

/* ============================================================================
 *  FIFO-геометрия
 *  ODR 6400 Гц, TIM6 640 Гц → 10 пакетов за период опроса
 *  [FIX] Пакет 20-байтный HIRES:
 *    header(1) + accel(6) + gyro(6) + temp(1) + tmst(2) + hires_nibbles(4) = 20 байт
 * ========================================================================== */
#define ICM_FIFO_POLL_PACKETS               10U
#define ICM_FIFO_PACKET_BYTES               ICM45686_FIFO_PACKET_SIZE_HIRES  /* 20 */
#define ICM_FIFO_PAYLOAD_BYTES              (ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES) /* 200 */
#define ICM_FIFO_DMA_BUF_SIZE               (ICM_FIFO_PAYLOAD_BYTES + 1U)   /* 201 */

#define ICM_FIFO_WATERMARK_PACKETS          ICM_FIFO_POLL_PACKETS          /* 10  */
#define ICM_FIFO_WATERMARK_BYTES            ICM_FIFO_PAYLOAD_BYTES         /* 160 */

/* ============================================================================
 *  Частота поллинга TIM6: 6400 / 10 = 640 Гц
 * ========================================================================== */
#define ICM_POLL_RATE_HZ                    640U

/* ============================================================================
 *  Готовые маски для записи в регистры
 * ========================================================================== */

/* FIFO_CONFIG3 (0x21): IF_EN=bit0, ACCEL_EN=bit1, GYRO_EN=bit2 */
#define ICM_FIFO_CONFIG3_MASK \
    (ICM45686_FIFO_IF_EN | ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN)

/* FIFO_CONFIG4 (0x22): TMST_FSYNC_EN=bit1 */
#define ICM_FIFO_CONFIG4_MASK               ICM45686_FIFO_TMST_FSYNC_EN

/* PWR_MGMT0 (0x10): Gyro LN (0x0C) + Accel LN (0x03) = 0x0F */
#define ICM_PWR_MGMT0_MASK \
    (ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN)

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_CONFIG_H */
