#ifndef ICM45686_DATA_H
#define ICM45686_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "icm45686_spi.h"   /* <- даёт ICM_TOTAL_SENSORS, ICM_FIFO_POLL_PACKETS */

typedef struct {
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int8_t   temp;
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
