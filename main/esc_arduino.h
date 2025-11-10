#pragma once
#include "esc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Llamar DESPUÉS de initArduino()
void esc_setup(void);

// Pulso en microsegundos para el motor idx=[0..MOTOR_COUNT-1]
void esc_set_pulse_us(int idx, int us);

// Opcional: helpers
void esc_set_all_us(int us);

#ifdef __cplusplus
}
#endif