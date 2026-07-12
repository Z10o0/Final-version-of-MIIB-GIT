/**
 * @file    icm45686_data.h
 * @brief   Структуры для хранения распакованных данных IMU
 *          и прототип функции разбора FIFO-пакетов.
 */

#ifndef ICM45686_DATA_H
#define ICM45686_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "icm45686_config.h"

/* ================================================================
 * Один распакованный пакет измерений (16-бит режим)
 * ================================================================ */
typedef struct {
    int16_t accel_x;    /* Акселерометр X (сырые LSB) */
    int16_t accel_y;    /* Акселерометр Y */
    int16_t accel_z;    /* Акселерометр Z */
    int16_t gyro_x;     /* Гироскоп X (сырые LSB) */
    int16_t gyro_y;     /* Гироскоп Y */
    int16_t gyro_z;     /* Гироскоп Z */
    int8_t  temp;       /* Температура (сырые LSB) */
    uint16_t timestamp; /* Временная метка (20 мкс/тик) */
} ICM_Sample_t;

/* ================================================================
 * Буфер пакетов для одного датчика за один цикл опроса
 * ================================================================ */
typedef struct {
    ICM_Sample_t samples[ICM_FIFO_POLL_PACKETS]; /* ICM_FIFO_POLL_PACKETS пакетов */
    uint8_t      count;                          /* Реально распакованных пакетов */
    uint8_t      sensor_id;                      /* Глобальный ID датчика 0..17 */
} ICM_SensorBatch_t;

/* ================================================================
 * Сводный результат одного цикла опроса всех 18 датчиков
 * ================================================================ */
extern ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/* ================================================================
 * Публичные функции
 * ================================================================ */

/**
 * @brief  Разбирает FIFO-буферы всех 18 датчиков и заполняет g_sensor_batches.
 *         Вызывать в main-loop при g_fifo_batch_ready == 1.
 */
void ICM_ParseAllFIFO(void);

/**
 * @brief  Разбирает один FIFO-буфер одного датчика.
 * @param  raw_buf   Указатель на RX-буфер (начиная с байта после адреса)
 * @param  buf_len   Длина данных в байтах
 * @param  batch     Указатель на структуру результата
 */
void ICM_ParseFIFOBuffer(const uint8_t *raw_buf,
                         uint16_t       buf_len,
                         ICM_SensorBatch_t *batch);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_DATA_H */
