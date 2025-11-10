#include "motor_ctrl.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#include "servo_esc.h"
#include "esc_config.h"
#include "led.h"

#define ESC_MIN_HOLD_MS 2000
#define TAG "MOTOR"

// Estado interno de cada motor
static bool motor_state[MOTOR_COUNT] = {0};

// Prototipo de initArduino() (si tu proyecto usa Arduino + IDF)
#ifdef __cplusplus
extern "C"
{
#endif
    void initArduino(void);
#ifdef __cplusplus
}
#endif

bool motor_ctrl_init(void)
{
    // Si NO has llamado initArduino() en otra parte, hazlo aquí:
    // initArduino();

    rgb_led_set(255, 255, 255); // Blanco: inicializando

    esc_setup();

    char report[128] = {0};
    bool ok = esc_self_test(ESC_MIN_HOLD_MS, report, sizeof(report));

    ESP_LOGI(TAG, "Self-test: %s | %s", ok ? "OK" : "FAIL", report);

    if (!ok)
    {
        rgb_led_set(255, 0, 0); // Rojo: error
        return false;
    }

    // OK → LED tenue o blanco suave
    // rgb_led_set(16, 16, 16);

    return true;
}

void motor_ctrl_set_state(uint8_t id, bool on)
{
    if (id < 1 || id > MOTOR_COUNT)
        return;

    int idx = id - 1;

    if (on)
    {
        esc_set_pulse_us(idx, PWM_MIN_US);
        vTaskDelay(pdMS_TO_TICKS(100));
        esc_set_pulse_us(idx, PWM_IDLE_US);
    }
    else
    {
        esc_set_pulse_us(idx, PWM_MIN_US);
    }

    motor_state[idx] = on;
}

void motor_ctrl_set_all(bool on)
{
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++)
        motor_ctrl_set_state(i, on);
}

void motor_ctrl_set_speed(uint8_t id, uint16_t us)
{
    if (id < 1 || id > MOTOR_COUNT)
        return;

    int idx = id - 1;
    esc_set_pulse_us(idx, us);
    motor_state[idx] = (us > PWM_MIN_US);
}

void motor_ctrl_set_all_speed(uint16_t us)
{
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++)
        motor_ctrl_set_speed(i, us);
}

void motor_ctrl_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        esc_set_pulse_us(i, PWM_MIN_US);

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
        esc_set_pulse_us(i, (int)roundf(f[i]));
}
