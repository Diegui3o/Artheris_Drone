#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "filter.h"
#include "imu.h"
#include "kalman.h"
#include <math.h>
#include "esp_rom_sys.h"
#include "freertos/portmacro.h"

static const char *TAG = "ATT";

static esp_timer_handle_t s_att_timer = NULL;
static TaskHandle_t s_att_task = NULL;

// Kalman internos (uno por eje)
static Kalman kalmanRoll;
static Kalman kalmanPitch;
static Kalman kalmanYaw;

static AttitudeSample s_att_latest = {0};
static volatile uint32_t s_att_seq = 0;
static portMUX_TYPE s_att_mux = portMUX_INITIALIZER_UNLOCKED;

void attitude_publish(const AttitudeSample *a)
{
    if (!a)
        return;
    portENTER_CRITICAL(&s_att_mux);
    s_att_latest = *a;
    s_att_seq++;
    portEXIT_CRITICAL(&s_att_mux);
}

bool attitude_get_latest(AttitudeSample *out)
{
    if (!out)
        return false;
    bool ok = false;
    portENTER_CRITICAL(&s_att_mux);
    if (s_att_seq != 0)
    {
        *out = s_att_latest;
        ok = true;
    }
    portEXIT_CRITICAL(&s_att_mux);
    return ok;
}

static void IRAM_ATTR att_tick_cb(void *arg)
{
    TaskHandle_t t = (TaskHandle_t)arg;
    if (t)
    {
        BaseType_t xHigher = pdFALSE;
        vTaskNotifyGiveFromISR(t, &xHigher);
        if (xHigher)
            portYIELD_FROM_ISR();
    }
}

static void attitude_task(void *arg)
{
    // Crear y arrancar el timer SOLO UNA VEZ
    if (s_att_timer == NULL)
    {
        const esp_timer_create_args_t targs = {
            .callback = att_tick_cb,
            .arg = xTaskGetCurrentTaskHandle(),
#if defined(ESP_TIMER_ISR)
            .dispatch_method = ESP_TIMER_ISR,
#else
            .dispatch_method = ESP_TIMER_TASK,
#endif
            .name = "att_tick",
        };
        ESP_ERROR_CHECK(esp_timer_create(&targs, &s_att_timer));
        // 4300 us ≈ 0.0043 s  → ~232 Hz como en tu Arduino
        ESP_ERROR_CHECK(esp_timer_start_periodic(s_att_timer, 4300));
        ESP_LOGI(TAG, "attitude timer started at ~232 Hz (dt = 4.3 ms)");
    }

    // Estado interno para yaw
    static float yaw_int_deg = 0.0f;  // integración pura (opcional)
    static float yaw_comp_deg = 0.0f; // filtro complementario

    for (;;)
    {
        // Espera el tick
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 1) Obtener muestra IMU más reciente
        ImuSample s;
        if (!imu_get_latest(&s))
        {
            continue;
        }

        // 2) Usar directamente los ángulos por ACC que ya calculaste en imu.c
        const float roll_acc = s.roll_acc_deg;
        const float pitch_acc = s.pitch_acc_deg;

        // 3) dt FIJO como en tu código Arduino
        const float dt = 0.0043f;

        // 4) Kalman roll/pitch
        const float roll_deg = (float)kalman_update(&kalmanRoll, roll_acc, s.gyro_x_dps, dt);
        const float pitch_deg = (float)kalman_update(&kalmanPitch, pitch_acc, s.gyro_y_dps, dt);

        // 5) Yaw a partir del gyro Z (RateYaw = s.gyro_z_dps)
        const float RateYaw = s.gyro_z_dps; // °/s

        // 5.1 Integración pura
        yaw_int_deg += RateYaw * dt;

        // 5.2 Filtro complementario
        // yaw = 0.98 * yaw + 0.02 * (RateYaw * dt);
        yaw_comp_deg = 0.98f * yaw_comp_deg + 0.02f * (RateYaw * dt);

        // 5.3 Kalman sobre yaw
        const float yaw_deg = (float)kalman_update(&kalmanYaw, yaw_comp_deg, RateYaw, dt);

        // 6) Timestamp solo para referencia (no para dt)
        const int64_t now_us = esp_timer_get_time();

        // 7) Publicar para consumidores (control/telemetría)
        const AttitudeSample a = {
            .t_us = now_us,
            .roll_deg = roll_deg,
            .pitch_deg = pitch_deg,
            .yaw_deg = yaw_deg,
        };
        attitude_publish(&a);
    }
}

bool attitude_start(int core, int priority, float q_angle, float q_bias, float r_measure)
{
    if (s_att_task)
        return true;

    kalman_init(&kalmanRoll, q_angle, q_bias, r_measure);
    kalman_init(&kalmanPitch, q_angle, q_bias, r_measure);
    kalman_init(&kalmanYaw, q_angle, q_bias, r_measure);

    const BaseType_t ok = xTaskCreatePinnedToCore(
        attitude_task, "attitude", 4096, NULL, priority, &s_att_task, core);

    return ok == pdPASS;
}
