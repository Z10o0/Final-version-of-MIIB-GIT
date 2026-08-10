#ifndef ICM45686_DATA_H
#define ICM45686_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "icm45686_spi.h"
#include "icm45686_config.h"

/* ================================================================
 * ICM_Sample_t — один 20-байтный HIRES-семпл.
 *
 * Оси хранятся как int32_t, знаковое 20-битное значение (не сдвинутое
 * влево): диапазон -524288 ... +524287.
 *
 * temp_raw: Big Endian int16_t → °C = temp_raw / 128.0f + 25.0f
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

/* ================================================================
 * ICM_SensorBatch_t — все пакеты, извлечённые из FIFO одного датчика
 * за один цикл опроса.
 *
 * samples[] размерность параметризована через ICM_FIFO_POLL_PACKETS
 * (icm45686_config.h) — при изменении числа пакетов в опросе структура
 * подстраивается автоматически без ручной правки.
 * ================================================================ */
typedef struct {
    ICM_Sample_t samples[ICM_FIFO_POLL_PACKETS];  /* было: [1], теперь [16] */
    uint8_t      count;      /* фактическое число валидных HIRES-пакетов, до 16 */
    uint8_t      sensor_id;
} ICM_SensorBatch_t;

extern ICM_SensorBatch_t g_sensor_batches[ICM_TOTAL_SENSORS];

void ICM_ParseAllFIFO(void);
void ICM_ParseFIFOBuffer(const uint8_t *raw_buf,
                          uint16_t buf_len,
                          ICM_SensorBatch_t *batch);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_DATA_H */
