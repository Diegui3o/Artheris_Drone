#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
// Opcional: secuencia de pitidos en un buzzer pasivo/activo
#include "driver/gpio.h"

#define TAG "MOTOR"
#define MOTOR_COUNT 4
static const int motor_pins[MOTOR_COUNT] = {10, 11, 12, 13};

#define PWM_FREQ_HZ 500
#define PWM_MIN_US 1000
#define PWM_MAX_US 2000
#define PWM_IDLE_US 1170

#define PWM_PERIOD_US (1000000 / PWM_FREQ_HZ) // 2000 µs

static mcpwm_cmpr_handle_t comparator[MOTOR_COUNT];
static bool motor_state[MOTOR_COUNT] = {false};

// =========================================================
// === Helpers ===
// =========================================================
static inline void set_pulse_us(int id, int us)
{
    if (id < 0 || id >= MOTOR_COUNT)
        return;
    if (us < PWM_MIN_US)
        us = PWM_MIN_US;
    if (us > PWM_MAX_US)
        us = PWM_MAX_US;

    // aquí el valor de comparación va directo en µs
    mcpwm_comparator_set_compare_value(comparator[id], us);
}

// =========================================================
// === Inicialización MCPWM ===
// =========================================================
bool motor_ctrl_init(void)
{
    ESP_LOGI(TAG, "Inicializando MCPWM para %d motores...", MOTOR_COUNT);

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t oper;
        mcpwm_timer_config_t timer_config = {
            .group_id = i / 2,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000, // 1 MHz → 1 tick = 1 µs
            .period_ticks = PWM_PERIOD_US,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        mcpwm_new_timer(&timer_config, &timer);

        mcpwm_operator_config_t operator_config = {.group_id = i / 2};
        mcpwm_new_operator(&operator_config, &oper);
        mcpwm_operator_connect_timer(oper, timer);

        mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
        mcpwm_new_comparator(oper, &comparator_config, &comparator[i]);

        mcpwm_gen_handle_t generator;
        mcpwm_generator_config_t gen_config = {.gen_gpio_num = motor_pins[i]};
        mcpwm_new_generator(oper, &gen_config, &generator);

        mcpwm_generator_set_action_on_timer_event(generator,
                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(generator,
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[i], MCPWM_GEN_ACTION_LOW));

        mcpwm_timer_enable(timer);
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    }

    vTaskDelay(pdMS_TO_TICKS(1500)); // esperar alimentación estable

    ESP_LOGI(TAG, "==== Calibrando y armando ESCs (50 Hz PWM) ====");

    // ✅ Dejar en mínimo seguro (no en idle)
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_MIN_US);
    ESP_LOGI(TAG, "Calibración finalizada. Motores en reposo (PWM_MIN).");
    return true;
}

// Secuencia de calibración/arming que el usuario puede invocar cuando
void motor_ctrl_calibrate_esc(void)
{
    ESP_LOGI(TAG, "Iniciando secuencia de arming/calibración de ESCs...");

    // Paso 1: minimo -> asegurar estado conocido
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_MIN_US);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Paso 2: máximo (algunos ESC requieren batería conectada ahora)
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_MAX_US);
    ESP_LOGI(TAG, "Manteniendo PWM MAX (%d us) - conecta la batería ahora si tu ESC lo requiere", PWM_MAX_US);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Paso 3: volver a mínimo para completar calibración
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_MIN_US);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Paso 4: idle (ligeramente > mínimo)
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_IDLE_US);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Paso final: dejar en mínimo seguro
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_MIN_US);
    vTaskDelay(pdMS_TO_TICKS(500));
    // Paso 3: volver a mínimo para completar calibración
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_MIN_US);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // ✅ Dejar en mínimo seguro (no en idle)
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_pulse_us(i, PWM_MIN_US);
    ESP_LOGI(TAG, "Calibración finalizada. Motores en reposo (PWM_MIN).");
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
        set_pulse_us(id - 1, PWM_MIN_US);
        vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa para estabilizar
        set_pulse_us(id - 1, PWM_IDLE_US);
    }
    else
    {
        set_pulse_us(id - 1, PWM_MIN_US);
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
    set_pulse_us(id - 1, us);
    motor_state[id - 1] = (us > PWM_MIN_US);
}

void motor_ctrl_set_all_speed(uint16_t us)
{
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++)
        motor_ctrl_set_speed(i, us);
}

void motor_ctrl_stop_all(void)
{
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++)
        set_pulse_us(i - 1, PWM_MIN_US);
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

    const float f_min = PWM_MIN_US;
    const float f_max = PWM_MAX_US;
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
        set_pulse_us(i, (int)roundf(f[i]));
}
