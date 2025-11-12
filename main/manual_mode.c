#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "manual_mode.h"
#include "mode_control.h"
#include "imu.h"
#include "filter.h"
#include "motor_ctrl.h"
#include <stdint.h>

// --- Pines RC (entradas del receptor) ---
static const int channel_1_pin = 4;
static const int channel_2_pin = 5;
static const int channel_3_pin = 6;
static const int channel_4_pin = 7;
static const int channel_5_pin = 15;
static const int channel_6_pin = 16;

#define TORQUE_SCALE 120.0f
#define MAX_INTEGRAL_ROLL_PITCH 100.0f
#define MAX_INTEGRAL_YAW 300.0f

static const char *TAG_MAN = "MANUAL";

// ReceiverValue: variable que usa el resto del programa
static volatile uint32_t ReceiverValue[6] = {1500, 1500, 1000, 1500, 1000, 1000};

// === LQR gains (will be updated online per throttle) ===
static float Ki_at[3][3] = {
    {0.65f, 0.0f, 0.0f},
    {0.0f, 0.65f, 0.0f},
    {0.0f, 0.0f, 0.0f}};
static float Kc_at[3][6] = {
    {1.63f, 0, 0, 0.353f, 0, 0},
    {0, 2.30f, 0, 0, 0.10f, 0},
    {0, 0, 5.3f, 0, 0, 0.2f}};

static volatile uint8_t last_lvl[6] = {0};
static volatile int64_t t_rise[6] = {0};

// Map logical index 0..5 to GPIO numbers
static const gpio_num_t ch_gpio[6] = {
    (gpio_num_t)channel_1_pin,
    (gpio_num_t)channel_2_pin,
    (gpio_num_t)channel_3_pin,
    (gpio_num_t)channel_4_pin,
    (gpio_num_t)channel_5_pin,
    (gpio_num_t)channel_6_pin,
};

// Quick helper to find channel index by GPIO
static inline int ch_index_from_gpio(gpio_num_t gpio)
{
    for (int i = 0; i < 6; i++)
        if (ch_gpio[i] == gpio)
            return i;
    return -1;
}

static void IRAM_ATTR rc_gpio_isr(void *arg)
{
    gpio_num_t gpio = (gpio_num_t)(uint32_t)arg;
    int idx = ch_index_from_gpio(gpio);
    if (idx < 0)
        return;

    int lvl = gpio_get_level(gpio);
    int64_t now = esp_timer_get_time(); // us

    if (lvl && !last_lvl[idx])
    {
        last_lvl[idx] = 1;
        t_rise[idx] = now;
    }
    else if (!lvl && last_lvl[idx])
    {
        // Falling edge
        last_lvl[idx] = 0;
        int64_t pw = now - t_rise[idx];
        if (pw >= 800 && pw <= 2500)
        {
            __atomic_store_n(&ReceiverValue[idx], (uint32_t)pw, __ATOMIC_RELAXED);
        }
    }
}

static void rc_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    for (int i = 0; i < 6; i++)
        io.pin_bit_mask |= (1ULL << ch_gpio[i]);
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (int i = 0; i < 6; i++)
    {
        ESP_ERROR_CHECK(gpio_isr_handler_add(ch_gpio[i], rc_gpio_isr, (void *)(uint32_t)ch_gpio[i]));
    }
}

// ====== 1 kHz scheduler via esp_timer ======
static TaskHandle_t s_manual_task = NULL;
static esp_timer_handle_t s_tick_1khz = NULL;

static void tick_1khz_cb(void *arg)
{
    if (s_manual_task)
    {
        xTaskNotifyGive(s_manual_task);
    }
}

// Integrators
static float integral_phi = 0, integral_theta = 0, integral_psi = 0;
static float tau_x = 0, tau_y = 0, tau_z = 0;

static inline float constrain_f(float x, float mn, float mx)
{
    return x < mn ? mn : (x > mx ? mx : x);
}

void manual_loop_task(void *pvParameters)
{
    // Variables de estado
    static bool motors_armed = false;
    static bool was_low_throttle = false;
    static int64_t prev_us = 0;
    static float yaw_deg_int = 0.0f;
    static int debug_counter = 0;

    ESP_LOGI(TAG_MAN, "Manual loop task started");

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Leer valores RC de forma atómica
        uint32_t localRV[6];
        for (int i = 0; i < 6; ++i)
        {
            localRV[i] = __atomic_load_n(&ReceiverValue[i], __ATOMIC_RELAXED);
        }

        // Debug: mostrar valores cada 2 segundos
        if (debug_counter++ % 2000 == 0)
        {
            ESP_LOGI(TAG_MAN, "RC: AIL:%d ELE:%d THR:%d RUD:%d",
                     localRV[0], localRV[1], localRV[2], localRV[3]);
        }

        // === 1) Parsear valores RC ===
        float DesiredAngleRoll = 0.1f * ((int)localRV[0] - 1500);
        float DesiredAnglePitch = 0.1f * ((int)localRV[1] - 1500);
        uint32_t InputThrottle = localRV[2];
        float DesiredAngleYaw = 0.15f * ((int)localRV[3] - 1500);

        // === 2) Lógica de arming ===
        const uint32_t THROTTLE_DEADZONE_MIN = 1010;
        const uint32_t THROTTLE_DEADZONE_MAX = 1030;

        if (InputThrottle < THROTTLE_DEADZONE_MIN)
        {
            was_low_throttle = true;
        }

        if (!motors_armed && was_low_throttle && InputThrottle > THROTTLE_DEADZONE_MAX)
        {
            motors_armed = true;
            was_low_throttle = false;
            integral_phi = integral_theta = integral_psi = 0;
            yaw_deg_int = 0;
            ESP_LOGI(TAG_MAN, "Motores armados! Throttle=%d", InputThrottle);
        }

        if (motors_armed && InputThrottle < THROTTLE_DEADZONE_MIN)
        {
            motors_armed = false;
            was_low_throttle = true;
            ESP_LOGI(TAG_MAN, "Motores desarmados!");
        }

        // === 3) Control activo solo si motores armados ===
        if (motors_armed && InputThrottle > THROTTLE_DEADZONE_MAX)
        {
            // Aplicar deadzone al throttle
            uint32_t effective_throttle = InputThrottle;
            if (InputThrottle < THROTTLE_DEADZONE_MAX)
            {
                effective_throttle = THROTTLE_DEADZONE_MAX;
            }

            // === 3) Gain scheduling (as in your sketch) ===
            float k1 = 3.10f - (1.2f / (1.0f + expf(((float)effective_throttle - 1429.0f) / -47.0f))) - (1.0f / (1.0f + expf(((float)effective_throttle - 1609.0f) / -52.8f)));
            float k2 = k1;
            float k3 = 2.10f - (1.2f / (1.0f + expf(((float)effective_throttle - 1429.0f) / -47.0f))) - (1.0f / (1.0f + expf(((float)effective_throttle - 1609.0f) / -52.8f)));
            float g1 = 2.10f - (-0.15f / (1.0f + expf(((float)effective_throttle - 1549.0f) / -26.6f))) - (1.35f / (1.0f + expf(((float)effective_throttle - 1399.0f) / -65.3f)));
            float g2 = g1;
            float g3 = 15.3f - (0.3f / (1.0f + expf(((float)effective_throttle - 1539.0f) / -39.0f))) - (1.35f / (1.0f + expf(((float)effective_throttle - 1369.0f) / -40.1f)));

            Kc_at[0][0] = k1;
            Kc_at[1][1] = k2;
            Kc_at[2][2] = k3;
            Kc_at[0][3] = g1;
            Kc_at[1][4] = g2;
            Kc_at[2][5] = g3;

            // === 4) References (deg -> rad) with dead‑zone ===
            float phi_ref = (DesiredAngleRoll / 2.8f) * (float)M_PI / 180.0f;
            float theta_ref = (DesiredAnglePitch / 2.8f) * (float)M_PI / 180.0f;
            float psi_ref = (DesiredAngleYaw / 2.8f) * (float)M_PI / 180.0f;

            const float DZ = 1.5f * (float)M_PI / 180.0f;
            if (fabsf(phi_ref) < DZ)
                phi_ref = 0.0f;
            if (fabsf(theta_ref) < DZ)
                theta_ref = 0.0f;
            if (fabsf(psi_ref) < DZ)
                psi_ref = 0.0f;

            // === 5) Get actual IMU data ===
            AttitudeSample a;
            ImuSample s;

            if (!imu_get_latest(&s) || !attitude_get_latest(&a))
            {
                continue;
            }

            // dt a partir de IMU
            static int64_t prev_us = 0;
            const int64_t now_us = s.t_us;
            float dt = (prev_us > 0) ? (now_us - prev_us) / 1e6f : 0.001f;
            if (dt < 0.0002f || dt > 0.01f)
                dt = 0.001f;
            prev_us = now_us;

            // === 6) Estados actuales (rad) ===
            float roll_rad = a.roll_deg * (float)M_PI / 180.0f;
            float pitch_rad = a.pitch_deg * (float)M_PI / 180.0f;

            // Yaw: integra gyro z
            static float yaw_deg_int = 0.0f;
            yaw_deg_int += s.gyro_z_dps * dt;
            if (yaw_deg_int > 180.f)
                yaw_deg_int -= 360.f;
            if (yaw_deg_int < -180.f)
                yaw_deg_int += 360.f;

            float yaw_rad = yaw_deg_int * (float)M_PI / 180.0f;
            float gyroRoll_rad = s.gyro_x_dps * (float)M_PI / 180.0f;
            float gyroPitch_rad = s.gyro_y_dps * (float)M_PI / 180.0f;
            float gyroYaw_rad = s.gyro_z_dps * (float)M_PI / 180.0f;

            // === 7) Errores (rad) ===
            float error_phi = phi_ref - roll_rad;
            float error_theta = theta_ref - pitch_rad;
            float error_psi = psi_ref - yaw_rad;

            // === 8) Integradores con anti-windup ===
            integral_phi = constrain_f(integral_phi + error_phi * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
            integral_theta = constrain_f(integral_theta + error_theta * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
            integral_psi = constrain_f(integral_psi + error_psi * dt, -MAX_INTEGRAL_YAW, MAX_INTEGRAL_YAW);

            // === 9) LQR ===
            tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * gyroRoll_rad;
            tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][4] * gyroPitch_rad;
            tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi - Kc_at[2][5] * gyroYaw_rad;

            // === 10) Escala torques y aplica ===
            tau_x *= TORQUE_SCALE;
            tau_y *= TORQUE_SCALE;
            tau_z *= TORQUE_SCALE;

            motor_ctrl_apply_control(tau_x, tau_y, tau_z, (float)effective_throttle);
        }
        else
        {
            // Motores no armados
            motor_ctrl_stop_all();
        }
    }
}

void manual_mode_start(void)
{
    rc_gpio_init();
    ESP_LOGI(TAG_MAN, "GPIO RC initialized");

    // Crear timer de 1kHz
    if (!s_tick_1khz)
    {
        const esp_timer_create_args_t args = {
            .callback = &tick_1khz_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "man_1khz"};
        ESP_ERROR_CHECK(esp_timer_create(&args, &s_tick_1khz));
    }

    // Crear tarea
    UBaseType_t prio = uxTaskPriorityGet(NULL);
    xTaskCreatePinnedToCore(manual_loop_task, "manual_loop", 4096, NULL, prio + 3, &s_manual_task, 0);

    // Iniciar temporizador
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_tick_1khz, 1000));
    ESP_LOGI(TAG_MAN, "Manual mode started");
}

void manual_mode_stop(void)
{
    if (s_tick_1khz)
        esp_timer_stop(s_tick_1khz);

    motor_ctrl_stop_all();
    ESP_LOGI(TAG_MAN, "Manual mode stopped");
}