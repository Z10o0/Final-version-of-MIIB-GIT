#ifndef ICM45686_DATA_H
#define ICM45686_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "icm45686_spi.h"
#include "icm45686_config.h"

/*
 * Представление одного raw HIRES-пакета.
 *
 * accel/gyro: знаковые 20-битные значения в int32_t,
 * диапазон -524288 ... +524287.
 *
 * temp_raw: исходное знаковое 16-битное значение.
 * timestamp: исходное 16-битное поле FIFO.
 *
 * Та же структура используется для усреднённого output sample.
 */
typedef struct
{
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;

    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;

    int16_t temp_raw;
    uint16_t timestamp;
} ICM_Sample_t;

/*
 * Raw-батч одного датчика.
 *
 * samples[] сохраняет исходные FIFO-пакеты.
 * Усреднённые результаты не записываются поверх samples[].
 *
 * count: фактическое количество валидных HIRES-пакетов,
 * не более ICM_FIFO_POLL_PACKETS.
 */
typedef struct
{
    ICM_Sample_t samples[ICM_FIFO_POLL_PACKETS];

    uint8_t count;
    uint8_t sensor_id;
} ICM_SensorBatch_t;

typedef enum
{
    ICM_AVG_STATUS_OK = 0U,
    ICM_AVG_STATUS_NO_DATA = 1U,
    ICM_AVG_STATUS_SHORT_BATCH = 2U,
    ICM_AVG_STATUS_FAULT = 3U
} ICM_AverageStatus_t;

/* Исходные FIFO-батчи всех датчиков. */
extern ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

/* Один усреднённый output sample на датчик. */
extern ICM_Sample_t g_sensor_averaged[ICM_TOTAL_SENSORS];

/* Значения из ICM_AverageStatus_t. */
extern uint8_t g_sensor_average_status[ICM_TOTAL_SENSORS];

void ICM_ParseAllFIFO(void);

void ICM_ParseFIFOBuffer(const uint8_t *raw_buf,
                         uint16_t buf_len,
                         ICM_SensorBatch_t *batch);

/*
 * Вызывается из main loop после ICM_ParseAllFIFO().
 * Неполные батчи не усредняются: output обнуляется.
 */
void ICM_AverageAllBatches(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_DATA_H */
