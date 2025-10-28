#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_COUNT       4
#define MOTOR_PWM_MIN_US  1000
#define MOTOR_PWM_MAX_US  2000
#define MOTOR_PWM_IDLE_US 1170
#define MOTOR_PWM_FREQ_HZ 500

bool motor_ctrl_init(void);
void motor_ctrl_calibrate_esc(void);
void motor_ctrl_arm_sequence(void);
void motor_ctrl_stop_all(void);
void motor_ctrl_apply_control(float tau_x, float tau_y, float tau_z, float inputThrottle);
void motor_ctrl_set_all(bool on);
void motor_ctrl_set_all_speed(uint16_t us);
void motor_ctrl_set_state(uint8_t id, bool on);
void motor_ctrl_set_speed(uint8_t id, uint16_t us);