#ifndef ICM45686_CONFIG_H
#define ICM45686_CONFIG_H

/*
 * icm45686_config.h — рабочая конфигурация 18× ICM-45686.
 * Все символы берутся исключительно из icm45686_regs.h.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "icm45686_regs.h"

/* ============================================================================
 *  Топология шин
 * ========================================================================== */

#define ICM_SPI_BUS_COUNT               3U
#define ICM_SENSORS_PER_BUS             6U

/* ============================================================================
 *  ODR и FSR — 3200 Гц
 * ========================================================================== */

#define ICM_GYRO_ODR_VALUE              ICM45686_GYRO_ODR_3200HZ   /* 0x04 */
#define ICM_ACCEL_ODR_VALUE             ICM45686_ACCEL_ODR_3200HZ  /* 0x04 */
#define ICM_GYRO_FS_VALUE               ICM45686_GYRO_FS_2000DPS   /* 0x10 */
#define ICM_ACCEL_FS_VALUE              ICM45686_ACCEL_FS_16G       /* 0x10 */

/* ============================================================================
 *  FIFO-геометрия
 *  ODR 3200 Гц, TIM6 320 Гц → 10 пакетов за период
 *  Пакет: header(1)+accel(6)+gyro(6)+temp(1)+tmst(2) = 16 байт
 * ========================================================================== */

#define ICM_FIFO_POLL_PACKETS           10U
#define ICM_FIFO_PACKET_BYTES           ICM45686_FIFO_PACKET_SIZE_16BIT  /* 16 */
#define ICM_FIFO_PAYLOAD_BYTES          (ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES)  /* 160 */

/* DMA-буфер: +1 мусорный байт (адрес FIFO_DATA в SPI-транзакции) */
#define ICM_FIFO_DMA_BUF_SIZE           (ICM_FIFO_PAYLOAD_BYTES + 1U)   /* 161 */

#define ICM_FIFO_WATERMARK_PACKETS      ICM_FIFO_POLL_PACKETS            /* 10  */
#define ICM_FIFO_WATERMARK_BYTES        ICM_FIFO_PAYLOAD_BYTES           /* 160 */

/* ============================================================================
 *  WHO_AM_I
 * ========================================================================== */

#define ICM_WHOAMI_EXPECTED             ICM45686_WHO_AM_I_VALUE          /* 0xE9 */

/* ============================================================================
 *  Частота поллинга TIM6: 3200 / 10 = 320 Гц
 * ========================================================================== */

#define ICM_POLL_RATE_HZ                320U

/* ============================================================================
 *  Готовые маски для записи в регистры
 * ========================================================================== */

/* FIFO_CONFIG3 (0x21): 0x23 */
#define ICM_FIFO_CONFIG3_MASK \
    (ICM45686_FIFO_IF_EN | ICM45686_FIFO_ACCEL_EN | ICM45686_FIFO_GYRO_EN)

/* FIFO_CONFIG4 (0x22): 0x08 */
#define ICM_FIFO_CONFIG4_MASK           ICM45686_FIFO_TMST_FSYNC_EN

/* PWR_MGMT0 (0x10): 0x0F */
#define ICM_PWR_MGMT0_MASK \
    (ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN)

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_CONFIG_H */
