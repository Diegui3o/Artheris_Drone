#pragma once
#include <stdint.h>
#include <stdbool.h>

// Definir constantes si no están en motor_ctrl.h
#ifndef MOTOR_PWM_MIN_US
#define MOTOR_PWM_MIN_US 1000
#endif

#ifndef MOTOR_PWM_MAX_US  
#define MOTOR_PWM_MAX_US 2000
#endif

#ifndef MOTOR_PWM_IDLE_US
#define MOTOR_PWM_IDLE_US 1100
#endif

#ifndef MOTOR_COUNT
#define MOTOR_COUNT 4
#endif

// Pines de motores
#define MOTOR1_PIN 10
#define MOTOR2_PIN 11
#define MOTOR3_PIN 12
#define MOTOR4_PIN 13


// Inicializa MCPWM, pines y timers
void motor_ctrl_init(void);

// Secuencia 2000us → 1000us para calibración
void motor_ctrl_calibrate_esc(void);

// Secuencia de armado (1000 → 1150)
void motor_ctrl_arm_sequence(void);

// Apaga todos los motores (1000us)
void motor_ctrl_stop_all(void);

// Setea un motor específico
void motor_ctrl_set_speed(uint8_t motor_id, uint16_t pulse_us);

// Setea todos los motores
void motor_ctrl_set_all_speed(uint16_t pulse_us);

// Enciende/apaga motores (lleva a idle o min automáticamente)
void motor_ctrl_set_state(uint8_t id, bool on);
void motor_ctrl_set_all(bool on);

// Aplica mezclado del dron (LQR/PID)
void motor_ctrl_apply_control(float tau_x, float tau_y, float tau_z, float inputThrottle);
void motor_get_inputs(uint16_t out[MOTOR_COUNT]);