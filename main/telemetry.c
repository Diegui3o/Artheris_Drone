#include "telemetry.h"
#include "mode_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include "filter.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "motor_ctrl.h"
#include "motor_state.h"
#include "imu.h"

static const char *TAGT = "TEL";
static TaskHandle_t s_tel_task = NULL;

typedef struct
{
    uint32_t ip;
    uint16_t port;
} dest_t;

static void telemetry_task(void *arg)
{

    dest_t local = *(dest_t *)arg;
    heap_caps_free(arg);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAGT, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in to = {0};
    to.sin_family = AF_INET;
    to.sin_port = htons(local.port);
    to.sin_addr.s_addr = local.ip;

    ESP_LOGI(TAGT, "UDP listo a %s:%u", inet_ntoa(to.sin_addr), (unsigned)ntohs(to.sin_port));

    int64_t last_sent_ts = -1;
    drone_mode_t last_sent_mode = -1;
    uint16_t last_motor_vals[4] = {0};

    for (;;)
    {
        AttitudeSample a;
        if (attitude_get_latest(&a))
        {
            bool send_now = false;

            if (a.t_us != last_sent_ts)
            {
                last_sent_ts = a.t_us;
                send_now = true;
            }

            drone_mode_t current_mode = getMode();
            if (current_mode != last_sent_mode)
            {
                last_sent_mode = current_mode;
                send_now = true;
            }

            bool motors_changed = false;
            for (int i = 0; i < 4; i++)
            {
                if (motor_vals[i] != last_motor_vals[i])
                {
                    motors_changed = true;
                    last_motor_vals[i] = motor_vals[i];
                }
            }

            if (motors_changed)
            {
                send_now = true;
            }
            if (send_now)
            {
                ImuSample s;
                if (!imu_get_latest(&s))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }

                char json[384];
                int n = snprintf(json, sizeof(json),
                                 "{"
                                 "\"AngleRoll\":%.3f,"
                                 "\"AnglePitch\":%.3f,"

                                 "\"AngleRoll_est\":%.3f,"
                                 "\"AnglePitch_est\":%.3f,"
                                 "\"AngleYaw\":%.3f,"

                                 "\"gyroRateRoll\":%.3f,"
                                 "\"gyroRatePitch\":%.3f,"
                                 "\"RateYaw\":%.3f,"

                                 "\"AccX\":%.3f,"
                                 "\"AccY\":%.3f,"
                                 "\"AccZ\":%.3f,"

                                 "\"modoActual\":%d,"
                                 "\"MotorInput1\":%u,"
                                 "\"MotorInput2\":%u,"
                                 "\"MotorInput3\":%u,"
                                 "\"MotorInput4\":%u"
                                 "}\n",
                                 // filtrados
                                 a.roll_deg,
                                 a.pitch_deg,

                                 // “no filtrados” (IMU)
                                 s.roll_acc_deg,
                                 s.pitch_acc_deg,

                                 // Yaw filtrado
                                 a.yaw_deg,

                                 // Gyros
                                 s.gyro_x_dps,
                                 s.gyro_y_dps,
                                 s.gyro_z_dps,

                                 // ACC
                                 s.acc_x_g,
                                 s.acc_y_g,
                                 s.acc_z_g,

                                 (int)current_mode,
                                 (unsigned)motor_vals[0],
                                 (unsigned)motor_vals[1],
                                 (unsigned)motor_vals[2],
                                 (unsigned)motor_vals[3]);

                if (n > 0 && n < (int)sizeof(json))
                {
                    (void)sendto(sock, json, n, 0, (struct sockaddr *)&to, sizeof(to));
                }
                else
                {
                    ESP_LOGE("TEL_SEND", "❌ JSON too long: %d/%d", n, (int)sizeof(json));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool telemetry_start_core0(const char *remote_ip, uint16_t remote_port, int priority)
{
    if (s_tel_task)
        return true;

    if (priority <= 0)
        priority = 3;

    dest_t *d = heap_caps_malloc(sizeof(*d), MALLOC_CAP_DEFAULT);
    if (!d)
        return false;

    d->ip = inet_addr(remote_ip);
    d->port = remote_port;

    BaseType_t ok = xTaskCreatePinnedToCore(
        telemetry_task, "telemetry", 4096, d, priority, &s_tel_task, 0 /* core 0 */);

    if (ok != pdPASS)
    {
        heap_caps_free(d);
        return false;
    }
    return true;
}
