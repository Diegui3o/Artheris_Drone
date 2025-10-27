#include "telemetry.h"
#include "mode_control.h" // <-- para acceder a getMode()
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

static const char *TAGT = "TEL";
static TaskHandle_t s_tel_task = NULL;

static void telemetry_task(void *arg)
{
    struct Dest
    {
        uint32_t ip;
        uint16_t port;
    };
    struct Dest local = *(struct Dest *)arg;
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
    drone_mode_t last_sent_mode = -1; // para evitar enviar modo repetido

    for (;;)
    {
        AttitudeSample a;
        if (attitude_get_latest(&a))
        {
            bool send_now = false;

            // Verifica cambio en timestamp (nueva lectura de actitud)
            if (a.t_us != last_sent_ts)
            {
                last_sent_ts = a.t_us;
                send_now = true;
            }

            // Verifica cambio de modo
            drone_mode_t current_mode = getMode();
            if (current_mode != last_sent_mode)
            {
                last_sent_mode = current_mode;
                send_now = true;
            }

            if (send_now)
            {
                char json[128];
                int n = snprintf(json, sizeof(json),
                                 "{\"AngleRoll\":%.3f, \"AnglePitch\":%.3f, \"modoActual\":%d}\n",
                                 a.roll_deg, a.pitch_deg, (int)current_mode);
                if (n > 0)
                {
                    sendto(sock, json, n, 0, (struct sockaddr *)&to, sizeof(to));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz de tasa de envío
    }
}

bool telemetry_start_core0(const char *remote_ip, uint16_t remote_port, int priority)
{
    if (s_tel_task)
        return true;

    struct Dest
    {
        uint32_t ip;
        uint16_t port;
    };
    struct Dest *d = (struct Dest *)heap_caps_malloc(sizeof(struct Dest), MALLOC_CAP_DEFAULT);
    if (!d)
        return false;

    d->ip = inet_addr(remote_ip);
    d->port = remote_port;

    BaseType_t ok = xTaskCreatePinnedToCore(
        telemetry_task, "telemetry", 4096, d, priority, &s_tel_task, 0 /* core 0 */);

    return ok == pdPASS;
}
