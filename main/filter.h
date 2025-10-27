// filter.h
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t t_us;    // timestamp en microsegundos
    float   roll_deg;
    float   pitch_deg;
} AttitudeSample;

/**
 * Arranca la tarea de actitud (lee IMU, corre Kalman y publica AttitudeSample)
 * @param core       Núcleo (0/1)
 * @param priority   Prioridad FreeRTOS
 * @param q_angle    Q del ángulo (ruido de proceso)
 * @param q_bias     Q del bias (ruido de proceso)
 * @param r_measure  R de medición (acelerómetro)
 */
bool attitude_start(int core, int priority, float q_angle, float q_bias, float r_measure);

/**
 * Publica (write) la última actitud (llámalo SOLO desde la tarea de actitud).
 */
void attitude_publish(const AttitudeSample *a);

/**
 * Obtiene (read) la última actitud publicada. Devuelve true si había dato.
 */
bool attitude_get_latest(AttitudeSample *out);

#ifdef __cplusplus
}
#endif
