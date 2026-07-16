#ifndef ICM45686_CONFIG_H
#define ICM45686_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "icm45686_regs.h"

/* Число IMU */
#define ICM_SENSORS_PER_BUS                   6U
#define ICM_TOTAL_SENSORS                     18U
#define ICM_SPI_BUS_COUNT                     3U

/* Текущая рабочая конфигурация */
#define ICM_GYRO_ODR_VALUE                    ICM45686_GYRO_ODR_3200HZ
#define ICM_ACCEL_ODR_VALUE                   ICM45686_ACCEL_ODR_3200HZ

#define ICM_GYRO_FS_VALUE                     ICM45686_GYRO_FS_2000DPS
#define ICM_ACCEL_FS_VALUE                    ICM45686_ACCEL_FS_16G

/*
 * При ODR 3200 Гц и TIM6 = 320 Гц:
 * 10 измерений в FIFO за 3.125 мс.
 *
 * Для 6400 Гц при прежнем TIM6 = 320 Гц:
 * изменить на 20U.
 */
#define ICM_FIFO_POLL_PACKETS                 10U

/*
 * accel XYZ + gyro XYZ + temperature + timestamp:
 * header(1) + accel(6) + gyro(6) + temp(1) + tmst(2) = 16 байт.
 */
#define ICM_FIFO_PACKET_BYTES                 ICM45686_FIFO_PACKET_SIZE_16BIT

#define ICM_FIFO_PAYLOAD_BYTES \
    (ICM_FIFO_POLL_PACKETS * ICM_FIFO_PACKET_BYTES)

/*
 * Один дополнительный байт обязателен:
 * rx[0] — мусорный байт, принятый во время передачи адреса FIFO_DATA.
 * Полезные FIFO данные: rx[1]...rx[ICM_FIFO_PAYLOAD_BYTES].
 */
#define ICM_FIFO_DMA_BUF_SIZE                 (ICM_FIFO_PAYLOAD_BYTES + 1U)

#define ICM_FIFO_WATERMARK_BYTES              ICM_FIFO_PAYLOAD_BYTES

#define ICM_WHOAMI_EXPECTED                   ICM45686_WHO_AM_I_VALUE

/*
 * TIM6 должен вызывать ICM_StartBurstRead() с периодом:
 * 3200 / 10 = 320 Гц, то есть раз в 3125 мкс.
 */
#define ICM_POLL_RATE_HZ                      320U

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_CONFIG_H */
