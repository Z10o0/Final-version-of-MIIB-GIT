#ifndef ICM45686_DATA_H
#define ICM45686_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "icm45686_spi.h"

/* ================================================================
 * ICM_Sample_t — один 20-байтный HIRES-семпл.
 *
 * Оси хранятся как int32_t, выровненные по биту 31 (MSB = знак).
 * Для получения физического 20-битного значения: val = accel_x >> 12
 * Для перевода в g (при FS=16G): g = (accel_x >> 12) / 32768.0f * 16.0f
 *
 * temp_raw: (int8_t)pkt[13] → °C = temp_raw / 2.0f + 25.0f
 * ================================================================ */
typedef struct {
    int32_t  accel_x;
    int32_t  accel_y;
    int32_t  accel_z;
    int32_t  gyro_x;
    int32_t  gyro_y;
    int32_t  gyro_z;
    int16_t  temp_raw;
    uint16_t timestamp;
} ICM_Sample_t;

typedef struct {
    ICM_Sample_t samples[ICM_FIFO_POLL_PACKETS];
    uint8_t      count;
    uint8_t      sensor_id;
} ICM_SensorBatch_t;

extern ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

void ICM_ParseAllFIFO(void);
void ICM_ParseFIFOBuffer(const uint8_t    *raw_buf,
                         uint16_t          buf_len,
                         ICM_SensorBatch_t *batch);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_DATA_H */
