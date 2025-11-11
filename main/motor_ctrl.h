#pragma once
#include <stdint.h>
#include <stdbool.h>

//
// ===== CONFIGURACIÓN =====
//
#define MOTOR_COUNT 4

// Valores típicos estándar ESC
#define MOTOR_PWM_MIN_US  1000
#define MOTOR_PWM_MAX_US  1990
#define MOTOR_PWM_IDLE_US 1170

#define MOTOR_PWM_FREQ_HZ      200     // los ESC trabajan a 50Hz

//
// ===== API PÚBLICA REAL =====
//

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
