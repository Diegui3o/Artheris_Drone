// main/cmd_led.c
#include "cmd_led.h"
#include "led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "cJSON.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"

static const char *TAG = "CMD_LED";
static TaskHandle_t s_cmd_task = NULL;

static void apply_leds_many_ids(cJSON *ids, bool on)
{
    cJSON *it = NULL;
    cJSON_ArrayForEach(it, ids)
    {
        if (cJSON_IsNumber(it))
        {
            int id = it->valueint;
            (void)led_set((uint8_t)id, on);
        }
    }
}

// Devuelve true si manejó algún comando válido
static bool handle_json(const char *msg, int len)
{
    bool handled = false;

    cJSON *root = cJSON_ParseWithLength(msg, len);
    if (!root)
        return false;

    // 1) {"led": true/false}  -> todos
    cJSON *led = cJSON_GetObjectItemCaseSensitive(root, "led");
    if (cJSON_IsBool(led))
    {
        bool st = cJSON_IsTrue(led);
        for (uint8_t i = 1; i <= leds_count(); ++i)
            led_set(i, st);
        handled = true;
        goto out;
    }

    // 2) {"led": {"id":1, "state":true}}
    if (cJSON_IsObject(led))
    {
        cJSON *id = cJSON_GetObjectItemCaseSensitive(led, "id");
        cJSON *state = cJSON_GetObjectItemCaseSensitive(led, "state");
        if (cJSON_IsNumber(id) && cJSON_IsBool(state))
        {
            (void)led_set((uint8_t)id->valueint, cJSON_IsTrue(state));
            handled = true;
            goto out;
        }
    }

    // 3) {"leds": {"ids":[1,2,3], "state":false}}
    cJSON *leds = cJSON_GetObjectItemCaseSensitive(root, "leds");
    if (cJSON_IsObject(leds))
    {
        cJSON *ids = cJSON_GetObjectItemCaseSensitive(leds, "ids");
        cJSON *state = cJSON_GetObjectItemCaseSensitive(leds, "state");
        if (cJSON_IsArray(ids) && cJSON_IsBool(state))
        {
            apply_leds_many_ids(ids, cJSON_IsTrue(state));
            handled = true;
            goto out;
        }
    }

out:
    cJSON_Delete(root);
    return handled;
}
static void send_ack(int sock, const struct sockaddr_in *to, bool ok)
{
    char resp[48];
    int n = snprintf(resp, sizeof(resp), "{\"ok\":%s}", ok ? "true" : "false");
    if (n > 0)
    {
        sendto(sock, resp, n, 0, (const struct sockaddr *)to, sizeof(*to));
    }
}

static void cmd_led_task(void *arg)
{
    uint16_t listen_port = (uint16_t)(uintptr_t)arg;

    ESP_LOGI(TAG, "cmd_led_task: inicio (listen_port=%u)", (unsigned)listen_port);
    ESP_LOGI(TAG, "cmd_led_task: core=%d", esp_cpu_get_core_id());

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "socket failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in me = {0};
    me.sin_family = AF_INET;
    me.sin_port = htons(listen_port);
    me.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&me, sizeof(me)) < 0)
    {
        ESP_LOGE(TAG, "bind %u failed", listen_port);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "escuchando UDP en 0.0.0.0:%u", (unsigned)listen_port);

    char buf[512];

    for (;;)
    {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &flen);
        if (n <= 0)
            continue;

        buf[n] = '\0'; // por si quieres ver el string
        ESP_LOGI(TAG, "rx %dB de %s:%u -> %s",
                 n, inet_ntoa(from.sin_addr), (unsigned)ntohs(from.sin_port), buf);

        bool ok = handle_json(buf, n);

        // SIEMPRE responder un ACK para que tu UI no “revienta”
        send_ack(sock, &from, ok);
    }
}

bool cmd_led_start_core0(uint16_t listen_port, int priority)
{
    ESP_LOGI(TAG, "cmd_led_start_core0: llamada (port=%u, prio=%d)", (unsigned)listen_port, priority);
    if (s_cmd_task)
        return true;
    BaseType_t ok = xTaskCreatePinnedToCore(
        cmd_led_task, "cmd_led", 4096, (void *)(uintptr_t)listen_port, priority, &s_cmd_task, 0 /* core 0 */);
    return ok == pdPASS;
}
