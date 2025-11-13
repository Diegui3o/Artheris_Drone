#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "motor_ctrl.h"
#include "motor_state.h"

#define TAG "ESC"

typedef struct
{
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_operator_t op;
    mcpwm_io_signals_t io_signal;
    int pin;
} motor_cfg_t;

// Configuración CORREGIDA - cada motor usa su propia combinación unit/timer/op
static const motor_cfg_t motor_cfg[MOTOR_COUNT] = {
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM0A, MOTOR1_PIN}, // Motor 1
    {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM1A, MOTOR2_PIN}, // Motor 2
    {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM0A, MOTOR3_PIN}, // Motor 3
    {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM1A, MOTOR4_PIN}, // Motor 4
};

static bool motor_state[MOTOR_COUNT] = {false};

void pwm_init_pin(const motor_cfg_t *cfg)
{
    ESP_LOGI(TAG, "Inicializando Motor - Unit:%d, Timer:%d, Signal:%d, Pin:%d",
             cfg->unit, cfg->timer, cfg->io_signal, cfg->pin);

    // Configurar el pin GPIO para el PWM
    esp_err_t ret = mcpwm_gpio_init(cfg->unit, cfg->io_signal, cfg->pin);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error inicializando GPIO: %s", esp_err_to_name(ret));
        rgb_led_set(128, 0, 255);
        return;
    }

    // Configurar el timer PWM (solo una vez por timer)
    static bool timer_initialized[2][3] = {false}; // Para UNIT_0 y UNIT_1

    if (!timer_initialized[cfg->unit][cfg->timer])
    {
        mcpwm_config_t pwm_config = {
            .frequency = 200,
            .cmpr_a = 0,
            .cmpr_b = 0,
            .counter_mode = MCPWM_UP_COUNTER,
            .duty_mode = MCPWM_DUTY_MODE_0,
        };

        esp_err_t init_ret = mcpwm_init(cfg->unit, cfg->timer, &pwm_config);
        if (init_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error inicializando PWM: %s", esp_err_to_name(init_ret));
            rgb_led_set(128, 0, 255);
        }
        else
        {
            ESP_LOGI(TAG, "Timer inicializado - Unit:%d, Timer:%d", cfg->unit, cfg->timer);
            timer_initialized[cfg->unit][cfg->timer] = true;
        }
    }
}

void esc_write_us(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, int us)
{
    if (us < MOTOR_PWM_MIN_US)
        us = MOTOR_PWM_MIN_US;
    if (us > MOTOR_PWM_MAX_US)
        us = MOTOR_PWM_MAX_US;

    // Asegurar que el duty cycle esté en modo correcto
    mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_in_us(unit, timer, op, us);
}

void motor_ctrl_init(void)
{
    ESP_LOGI(TAG, "Inicializando controladores de motor...");

    // Inicializar todos los motores
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        pwm_init_pin(&motor_cfg[i]);
        ESP_LOGI(TAG, "Motor %d inicializado - Unit:%d, Timer:%d, Op:%d, Pin:%d",
                 i + 1, motor_cfg[i].unit, motor_cfg[i].timer, motor_cfg[i].op, motor_cfg[i].pin);
    }

    // Espera para asegurar inicialización
    vTaskDelay(pdMS_TO_TICKS(1000));

    // CALIBRACIÓN - Paso 1: Señal máxima (2000us)
    ESP_LOGI(TAG, "CALIBRACIÓN: Enviando 2000us a todos los motores");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        esc_write_us(motor_cfg[i].unit, motor_cfg[i].timer, motor_cfg[i].op, 2000);
    }
    vTaskDelay(pdMS_TO_TICKS(3000)); // Espera para calibración

    // CALIBRACIÓN - Paso 2: Señal mínima (1000us)
    ESP_LOGI(TAG, "CALIBRACIÓN: Enviando 1000us a todos los motores");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        esc_write_us(motor_cfg[i].unit, motor_cfg[i].timer, motor_cfg[i].op, 1000);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Calibración completada para todos los motores");
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

    // Guardar valor actual
    if (us < MOTOR_PWM_MIN_US)
        us = MOTOR_PWM_MIN_US;
    if (us > MOTOR_PWM_MAX_US)
        us = MOTOR_PWM_MAX_US;

    motor_inputs[id] = (uint16_t)us;

    // ✅ DEBUG CRÍTICO: Verificar actualización de motor_vals
    uint16_t old_val = motor_vals[id];
    motor_vals[id] = (uint16_t)us;

    static int debug_count = 0;
    if (debug_count++ % 30 == 0)
    {
        ESP_LOGI("MOTOR_VALS_UPDATE", "Motor%d: %d -> %d us", id, old_val, motor_vals[id]);
    }
    if (id < 0 || id >= MOTOR_COUNT)
    {
        ESP_LOGE(TAG, "motor_set_us: id inválido %d", id);
        rgb_led_set(128, 0, 255);
        return;
    }
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

    static TickType_t last_log_time = 0;
    const TickType_t LOG_PERIOD = pdMS_TO_TICKS(70000); // 30 segundos

    if ((xTaskGetTickCount() - last_log_time) >= LOG_PERIOD)
    {
        ESP_LOGI(TAG, "Motores apagados");
        last_log_time = xTaskGetTickCount();
    }
}

void motor_get_inputs(uint16_t out[MOTOR_COUNT])
{
    // Copiamos los valores actuales a la salida para evitar lecturas parciales
    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        out[i] = motor_inputs[i];
    }
}
// =========================================================
// ===  Control colectivo con saturación (LQR / PID) ===
// =========================================================
void motor_ctrl_apply_control(float tau_x, float tau_y, float tau_z, float effective_throttle)
{
    int f[MOTOR_COUNT];
    f[0] = effective_throttle - tau_x - tau_y - tau_z;
    f[1] = effective_throttle - tau_x + tau_y + tau_z;
    f[2] = effective_throttle + tau_x + tau_y - tau_z;
    f[3] = effective_throttle + tau_x - tau_y + tau_z;

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
        float max_violation = 0;
        int worst_motor = 0;

        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            float violation = 0;
            if (f[i] > f_max)
                violation = f[i] - f_max;
            else if (f[i] < f_min)
                violation = f_min - f[i];

            if (violation > max_violation)
            {
                max_violation = violation;
                worst_motor = i;
            }
        }

        float gamma = 1.0f;
        if (f[worst_motor] > f_max)
            gamma = f_max / f[worst_motor];
        else if (f[worst_motor] < f_min)
            gamma = f_min / f[worst_motor];

        // Aplicar a todos los motores
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            f[i] = f[i] * gamma;
        }
    }

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (f[i] < f_min)
            f[i] = f_min;
        if (f[i] > f_max)
            f[i] = f_max;

        motor_set_us(i, (int)roundf(f[i]));
    }
}