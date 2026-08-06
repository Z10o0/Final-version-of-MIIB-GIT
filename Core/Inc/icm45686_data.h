#ifndef ICM45686_DATA_H
#define ICM45686_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "icm45686_spi.h"

/* ================================================================
 * ICM_Sample_t — один HIRES-семпл из 20-байтного FIFO пакета.
 *
 * Оси: int32_t, 20-битное знаковое значение (sign-extended).
 * Для 16-битной совместимости при передаче через UART используй
 * правый сдвиг: accel_x >> 4 даёт 16-битный эквивалент.
 * ================================================================ */
typedef struct {
    int32_t  accel_x;   /* 20-bit знаковое, в единицах LSB */
    int32_t  accel_y;
    int32_t  accel_z;
    int32_t  gyro_x;
    int32_t  gyro_y;
    int32_t  gyro_z;
    int16_t  temp_raw;  /* 16-bit: [19:12] в MSB + [11:4] в LSB → сдвиг вправо на 4 */
    uint16_t timestamp; /* Timestamp из байт 14-15, единица: ~1/32768 с при внеш. клоке */
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
