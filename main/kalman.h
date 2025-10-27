#pragma once
#include <stdint.h>

typedef struct {
    // Estado
    float angle;   // ángulo estimado (salida)
    float bias;    // sesgo del giroscopio
    // Matriz de covarianzas P
    float P00, P01;
    float P10, P11;
    // Parámetros (sintonía)
    float Q_angle;    // ruido de proceso del ángulo
    float Q_bias;     // ruido de proceso del sesgo
    float R_measure;  // ruido de la medición (acelerómetro)
} Kalman;

/** Inicializa estados y parámetros */
void kalman_init(Kalman* kf, float q_angle, float q_bias, float r_measure);

/** Re-inicia sólo el estado y P (útil si cambias el arranque) */
void kalman_reset(Kalman* kf, float angle0, float bias0);

/**
 * Paso del filtro:
 *   newAngle: medición (p.ej. ángulo por acelerómetro) [grados]
 *   newRate : velocidad del giroscopio (ya corregida por bias si quieres) [°/s]
 *   dt      : paso de tiempo [s]
 * Devuelve el ángulo estimado (kf->angle).
 */
float kalman_update(Kalman* kf, float newAngle, float newRate, float dt);
