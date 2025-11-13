#include "kalman.h"
#include "led.h"

void kalman_init(Kalman *kf, float q_angle, float q_bias, float r_measure)
{
    kf->angle = 0.0f;
    kf->bias = 0.0f;

    kf->P00 = 1.0f;
    kf->P01 = 0.0f;
    kf->P10 = 0.0f;
    kf->P11 = 1.0f;

    kf->Q_angle = q_angle;     // ~0.001f típico
    kf->Q_bias = q_bias;       // ~0.003f típico
    kf->R_measure = r_measure; // ~0.03f  típico
}

void kalman_reset(Kalman *kf, float angle0, float bias0)
{
    kf->angle = angle0;
    kf->bias = bias0;

    // Reset razonable de P (identidad)
    kf->P00 = 1.0f;
    kf->P01 = 0.0f;
    kf->P10 = 0.0f;
    kf->P11 = 1.0f;
}

float kalman_update(Kalman *kf, float newAngle, float newRate, float dt)
{
    // === Predicción ===
    float rate = newRate - kf->bias;
    kf->angle += dt * rate;

    // P = A P A' + Q con A = [[1, -dt],[0,1]], Q = [[Q_angle, 0],[0, Q_bias]]
    kf->P00 += dt * (dt * kf->P11 - kf->P01 - kf->P10 + kf->Q_angle);
    kf->P01 += -dt * kf->P11;
    kf->P10 += -dt * kf->P11;
    kf->P11 += kf->Q_bias * dt;

    // === Actualización (medición de ángulo) ===
    // S = H P H' + R,  H = [1, 0]
    float S = kf->P00 + kf->R_measure;
    // K = P H' S^-1  → K = [P00/S, P10/S]'
    float K0 = kf->P00 / S;
    float K1 = kf->P10 / S;

    // y = z - H x = newAngle - angle
    float y = newAngle - kf->angle;

    // x = x + K y
    kf->angle += K0 * y;
    kf->bias += K1 * y;

    // P = (I - K H) P
    float P00 = kf->P00;
    float P01 = kf->P01;

    kf->P00 -= K0 * P00;
    kf->P01 -= K0 * P01;
    kf->P10 -= K1 * P00;
    kf->P11 -= K1 * P01;

    return kf->angle;
}
