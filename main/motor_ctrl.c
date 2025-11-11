#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "motor_ctrl.h"

#define MCPWM0A MCPWM_OPR_A
#define MCPWM1A MCPWM_OPR_B
#define MCPWM2A MCPWM_OPR_A

#define TAG "ESC"
#define MOTOR1_PIN 10
#define MOTOR2_PIN 11
#define MOTOR3_PIN 12
#define MOTOR4_PIN 13
typedef struct
{
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_operator_t op;
    int pin;
} motor_cfg_t;

static const motor_cfg_t motor_cfg[MOTOR_COUNT] = {
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 10}, // motor 1
    {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 11}, // motor 2
    {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 12}, // motor 3
    {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 13}, // motor 4
};

static bool motor_state[MOTOR_COUNT] = {false};

void pwm_init_pin(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_io_signals_t signal, int pin)
{
    mcpwm_gpio_init(unit, signal, pin);

    mcpwm_config_t cfg = {
        .frequency = 50, // 50Hz -> periodo 20ms
        .cmpr_a = 0,     // duty arranca 0%
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(unit, timer, &cfg);
}

void esc_write_us(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, int us)
{
    if (us < 1000)
        us = 1000;
    if (us > 2000)
        us = 2000;
    mcpwm_set_duty_in_us(unit, timer, op, us);
}

void motor_ctrl_init(void)
{
    // Motor 1 en UNIT0, TIMER0
    pwm_init_pin(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MOTOR1_PIN);

    // Motor 2 en UNIT0, TIMER1
    pwm_init_pin(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MOTOR2_PIN);

    // Motor 3 en UNIT1, TIMER0
    pwm_init_pin(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MOTOR3_PIN);

    // Motor 4 en UNIT1, TIMER1
    pwm_init_pin(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MOTOR4_PIN);

    ESP_LOGI(TAG, "Enviando 2000us a todos para CALIBRACION");

    esc_write_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2000);
    esc_write_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 2000);
    esc_write_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 2000);
    esc_write_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 2000);

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Enviando 1000us para terminar CALIBRACION");

    esc_write_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
    esc_write_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
    esc_write_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
    esc_write_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);

    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Calibración completada");
}
void motor_set_us(int id, int us)
{
    if (id < 0 || id >= MOTOR_COUNT)
        return;

    esc_write_us(
        motor_cfg[id].unit,
        motor_cfg[id].timer,
        motor_cfg[id].op,
        us);
}
// =========================================================
// === Encendido, apagado y control ===
// =========================================================
void motor_ctrl_set_state(uint8_t id, bool on)
{
    if (id == 0 || id > MOTOR_COUNT)
        return;

    // Cuando se enciende, primero va a mínimo y luego a idle
    if (on)
    {
        motor_set_us(id - 1, MOTOR_PWM_MIN_US);
        vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa para estabilizar
        motor_set_us(id - 1, MOTOR_PWM_IDLE_US);
    }
    else
    {
        motor_set_us(id - 1, MOTOR_PWM_MIN_US);
    }
    motor_state[id - 1] = on;
}

void motor_ctrl_set_all(bool on)
{
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++)
        motor_ctrl_set_state(i, on);
}

void motor_ctrl_set_speed(uint8_t id, uint16_t us)
{
    if (id == 0 || id > MOTOR_COUNT)
        return;
    motor_set_us(id - 1, us);
    motor_state[id - 1] = (us > MOTOR_PWM_MIN_US);
}

void motor_ctrl_set_all_speed(uint16_t us)
{
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++)
        motor_ctrl_set_speed(i, us);
}

void motor_ctrl_stop_all(void)
{
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++)
        motor_set_us(i - 1, MOTOR_PWM_MIN_US);
    ESP_LOGI(TAG, "Motores apagados");
}

// =========================================================
// ===  Control colectivo con saturación (LQR / PID) ===
// =========================================================
void motor_ctrl_apply_control(float tau_x, float tau_y, float tau_z, float inputThrottle)
{
    float f[MOTOR_COUNT];
    f[0] = inputThrottle - tau_x - tau_y - tau_z;
    f[1] = inputThrottle - tau_x + tau_y + tau_z;
    f[2] = inputThrottle + tau_x + tau_y - tau_z;
    f[3] = inputThrottle + tau_x - tau_y + tau_z;

    const float f_min = MOTOR_PWM_MIN_US;
    const float f_max = MOTOR_PWM_MAX_US;
    bool saturado = false;

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (f[i] < f_min || f[i] > f_max)
        {
            saturado = true;
            break;
        }
    }

    if (saturado)
    {
        float gamma = 1.0f;
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (f[i] > f_max)
                gamma = f_max / f[i];
            else if (f[i] < f_min)
                gamma = f_min / f[i];
        }
        for (int i = 0; i < MOTOR_COUNT; i++)
            f[i] *= gamma;
    }

    for (int i = 0; i < MOTOR_COUNT; i++)
        motor_set_us(i, (int)roundf(f[i]));
}
