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

static AttitudeSample s_att_latest = {0};
static volatile uint32_t s_att_seq = 0;
static portMUX_TYPE s_att_mux = portMUX_INITIALIZER_UNLOCKED;

// -----------------------------------------------------------------------------
// API de publicación/lectura (resolve undefined references)
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Ticker ISR (1 kHz) para despertar la tarea de actitud
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Tarea de actitud: lee IMU -> calcula roll/pitch -> actualiza Kalman -> publica
// -----------------------------------------------------------------------------
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
        // 1000 us = 1 kHz
        ESP_ERROR_CHECK(esp_timer_start_periodic(s_att_timer, 1000));
        ESP_LOGI(TAG, "attitude timer started at 1 kHz");
    }

    int64_t prev_us = 0;

    for (;;)
    {
        // Espera el tick de 1 kHz
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 1) Obtener muestra IMU más reciente (producida por tu tarea imu_start_1khz)
        ImuSample s;
        if (!imu_get_latest(&s))
        {
            continue;
        }

        // 2) Angulos por acelerómetro (grados)
        const float roll_acc = atan2f(s.acc_y_g,
                                      sqrtf(s.acc_x_g * s.acc_x_g + s.acc_z_g * s.acc_z_g)) *
                               57.2957795f;
        const float pitch_acc = -atan2f(s.acc_x_g,
                                        sqrtf(s.acc_y_g * s.acc_y_g + s.acc_z_g * s.acc_z_g)) *
                                57.2957795f;

        // 3) dt desde esp_timer
        const int64_t now_us = esp_timer_get_time();
        const float dt = (prev_us > 0) ? (now_us - prev_us) / 1e6f : 0.0043f;
        prev_us = now_us;

        // 4) Kalman (tu implementación en kalman.c)
        const float roll_deg = (float)kalman_update(&kalmanRoll, roll_acc, s.gyro_x_dps, dt);
        const float pitch_deg = (float)kalman_update(&kalmanPitch, pitch_acc, s.gyro_y_dps, dt);

        // 5) Publicar para consumidores (comms/control/etc.)
        const AttitudeSample a = {
            .t_us = now_us,
            .roll_deg = roll_deg,
            .pitch_deg = pitch_deg,
        };
        attitude_publish(&a);
    }
}

// -----------------------------------------------------------------------------
// Arranque público
// -----------------------------------------------------------------------------
bool attitude_start(int core, int priority, float q_angle, float q_bias, float r_measure)
{
    if (s_att_task)
        return true;

    kalman_init(&kalmanRoll, q_angle, q_bias, r_measure);
    kalman_init(&kalmanPitch, q_angle, q_bias, r_measure);

    const BaseType_t ok = xTaskCreatePinnedToCore(
        attitude_task, "attitude", 4096, NULL, priority, &s_att_task, core);

    return ok == pdPASS;
}
