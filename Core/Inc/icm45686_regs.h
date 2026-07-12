/**
 * @file    icm45686_regs.h
 * @brief   Адресная карта регистров ICM-45686.
 *          Источник: TDK InvenSense Reference Driver
 *          (tdk-invn-oss/motion.mcu.icm45686.driver)
 *          Используются только те регистры, которые нужны для
 *          инициализации, чтения FIFO и конфигурации датчика.
 */

#ifndef ICM45686_REGS_H
#define ICM45686_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * Блок 0 — основные регистры
 * ================================================================ */

/* Идентификатор устройства */
#define ICM45686_REG_WHO_AM_I                   0x72U
#define ICM45686_WHO_AM_I_VALUE                 0xE9U   /* ожидаемое значение */

/* Управление питанием и режимами */
#define ICM45686_REG_PWR_MGMT0                  0x10U

/* Конфигурация акселерометра */
#define ICM45686_REG_ACCEL_CONFIG0              0x1BU

/* Конфигурация гироскопа */
#define ICM45686_REG_GYRO_CONFIG0               0x1CU

/* Конфигурация FIFO */
#define ICM45686_REG_FIFO_CONFIG0               0x1DU
#define ICM45686_REG_FIFO_CONFIG1_0             0x28U
#define ICM45686_REG_FIFO_CONFIG1_1             0x29U

/* Счётчик байт в FIFO (2 байта, little-endian) */
#define ICM45686_REG_FIFO_COUNT_0               0x12U   /* LSB */
#define ICM45686_REG_FIFO_COUNT_1               0x13U   /* MSB */

/* Порт данных FIFO */
#define ICM45686_REG_FIFO_DATA                  0x14U

/* Конфигурация прерываний */
#define ICM45686_REG_INT_CONFIG0                0x06U
#define ICM45686_REG_INT_CONFIG1                0x07U
#define ICM45686_REG_INT_SOURCE0                0x65U
#define ICM45686_REG_INT_SOURCE1                0x66U

/* Статус прерываний */
#define ICM45686_REG_INT_STATUS                 0x19U
#define ICM45686_REG_INT_STATUS2                0x1AU
#define ICM45686_REG_INT_STATUS3                0x1BU   /* FIFO threshold */

/* Регистр сброса устройства */
#define ICM45686_REG_DEVICE_CONFIG              0x01U

/* Регистр выбора банка */
#define ICM45686_REG_BANK_SEL                   0x76U

/* ================================================================
 * Битовые поля REG_PWR_MGMT0 (0x10)
 * ================================================================ */
/* Режим гироскопа */
#define ICM45686_PWR_GYRO_MODE_OFF              (0x00U << 2)
#define ICM45686_PWR_GYRO_MODE_STANDBY          (0x01U << 2)
#define ICM45686_PWR_GYRO_MODE_LP               (0x02U << 2)
#define ICM45686_PWR_GYRO_MODE_LN               (0x03U << 2)

/* Режим акселерометра */
#define ICM45686_PWR_ACCEL_MODE_OFF             0x00U
#define ICM45686_PWR_ACCEL_MODE_LP              0x02U
#define ICM45686_PWR_ACCEL_MODE_LN              0x03U

/* ================================================================
 * Битовые поля ACCEL_CONFIG0 (0x1B)
 * ================================================================ */
/* Диапазон акселерометра [7:5] */
#define ICM45686_ACCEL_FS_16G                   (0x00U << 4)
#define ICM45686_ACCEL_FS_8G                    (0x01U << 4)
#define ICM45686_ACCEL_FS_4G                    (0x02U << 4)
#define ICM45686_ACCEL_FS_2G                    (0x03U << 4)

/* Частота акселерометра ODR [3:0] */
#define ICM45686_ACCEL_ODR_6400HZ               0x03U
#define ICM45686_ACCEL_ODR_3200HZ               0x04U
#define ICM45686_ACCEL_ODR_1600HZ               0x05U
#define ICM45686_ACCEL_ODR_800HZ                0x06U
#define ICM45686_ACCEL_ODR_400HZ                0x07U
#define ICM45686_ACCEL_ODR_200HZ                0x08U
#define ICM45686_ACCEL_ODR_100HZ                0x09U
#define ICM45686_ACCEL_ODR_50HZ                 0x0AU

/* ================================================================
 * Битовые поля GYRO_CONFIG0 (0x1C)
 * ================================================================ */
/* Диапазон гироскопа [7:4] */
#define ICM45686_GYRO_FS_4000DPS                (0x00U << 4)
#define ICM45686_GYRO_FS_2000DPS                (0x01U << 4)
#define ICM45686_GYRO_FS_1000DPS                (0x02U << 4)
#define ICM45686_GYRO_FS_500DPS                 (0x03U << 4)
#define ICM45686_GYRO_FS_250DPS                 (0x04U << 4)
#define ICM45686_GYRO_FS_125DPS                 (0x05U << 4)
#define ICM45686_GYRO_FS_62_5DPS                (0x06U << 4)
#define ICM45686_GYRO_FS_31_25DPS               (0x07U << 4)

/* Частота гироскопа ODR [3:0] */
#define ICM45686_GYRO_ODR_6400HZ                0x03U
#define ICM45686_GYRO_ODR_3200HZ                0x04U
#define ICM45686_GYRO_ODR_1600HZ                0x05U
#define ICM45686_GYRO_ODR_800HZ                 0x06U
#define ICM45686_GYRO_ODR_400HZ                 0x07U
#define ICM45686_GYRO_ODR_200HZ                 0x08U
#define ICM45686_GYRO_ODR_100HZ                 0x09U
#define ICM45686_GYRO_ODR_50HZ                  0x0AU

/* ================================================================
 * Битовые поля FIFO_CONFIG0 (0x1D)
 * ================================================================ */
/* Режим FIFO [7:6] */
#define ICM45686_FIFO_MODE_BYPASS               (0x00U << 6)
#define ICM45686_FIFO_MODE_STREAM               (0x01U << 6)
#define ICM45686_FIFO_MODE_STOP_ON_FULL         (0x02U << 6)

/* Выбор данных в FIFO: гироскоп + акселерометр + температура */
#define ICM45686_FIFO_SEL_GYRO                  (1U << 4)
#define ICM45686_FIFO_SEL_ACCEL                 (1U << 3)
#define ICM45686_FIFO_SEL_TEMP                  (1U << 2)
#define ICM45686_FIFO_SEL_TMST                  (1U << 1)
#define ICM45686_FIFO_SEL_HIRES                 (1U << 0)

/* ================================================================
 * Прочие константы
 * ================================================================ */
/* Заголовок пакета FIFO */
#define ICM45686_FIFO_HEADER_ACCEL_BIT          (1U << 6)
#define ICM45686_FIFO_HEADER_GYRO_BIT           (1U << 5)
#define ICM45686_FIFO_HEADER_TWENTYBITS_BIT     (1U << 3)
#define ICM45686_FIFO_HEADER_TIMESTAMP_BIT      (1U << 2)
#define ICM45686_FIFO_HEADER_FSYNC_BIT          (1U << 1)

/* Размер одного FIFO-пакета (16-бит режим): 1(header)+6(gyro)+6(accel)+1(temp)+2(ts) */
#define ICM45686_FIFO_PACKET_SIZE_16BIT         16U

/* Размер пакета HIGH-RES (20-бит): 1(header)+6(gyro)+6(accel)+3(hires)+1(temp)+2(ts) */
#define ICM45686_FIFO_PACKET_SIZE_HIRES         20U

/* Бит READ для SPI-транзакции */
#define ICM45686_SPI_READ_BIT                   0x80U

/* Задержка после сброса (мс) */
#define ICM45686_RESET_DELAY_MS                 2U

/* Задержка перехода в активный режим (мс) */
#define ICM45686_STARTUP_DELAY_MS               100U

/* Максимальная ёмкость FIFO (байт) */
#define ICM45686_FIFO_SIZE_BYTES                2048U

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_REGS_H */
