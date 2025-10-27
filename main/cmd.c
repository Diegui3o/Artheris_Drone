#include "cmd.h"
#include "led.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <fcntl.h>
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "mode_control.h"

static const char *TAG = "CMD_LED";
static TaskHandle_t s_cmd_task = NULL;

static inline int _to_level(bool on)
{
    if (LED_ACTIVE_HIGH)
        return on ? 1 : 0;
    else
        return on ? 0 : 1;
}

static void apply_leds_many_ids(cJSON *ids, bool on)
{
    cJSON *it = NULL;
    cJSON_ArrayForEach(it, ids)
    {
        if (cJSON_IsNumber(it))
        {
            int id = it->valueint;
            // Si el id corresponde al LED RGB integrado (1), usar la API RGB
            if (id == 1)
            {
                if (on)
                    rgb_led_set(0, 255, 0); // verde
                else
                    rgb_led_set(255, 255, 255); // volver a blanco al "apagar"
            }
            else
            {
                // Control directo de pines definidos en LED_PINS
                size_t cnt = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
                if (id >= 1 && (size_t)id <= cnt)
                {
                    gpio_set_level(LED_PINS[id - 1], _to_level(on));
                }
            }
        }
    }
}

// Devuelve true si manejó algún comando válido
static bool handle_json(const char *msg, int len, char *out_info, size_t out_info_len)
{
    bool handled = false;

    cJSON *root = cJSON_ParseWithLength(msg, len);
    if (!root)
    {
        if (out_info && out_info_len)
            strncpy(out_info, "json_parse_error", out_info_len - 1);
        return false;
    }

    // Localiza el objeto que contiene la clave "led".
    // Soporta varios formatos: root.led, root.payload.led, root.payload.payload.led
    cJSON *led = cJSON_GetObjectItemCaseSensitive(root, "led");
    if (!led)
    {
        cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
        if (cJSON_IsObject(payload))
            led = cJSON_GetObjectItemCaseSensitive(payload, "led");
        if (!led && cJSON_IsObject(payload))
        {
            cJSON *payload2 = cJSON_GetObjectItemCaseSensitive(payload, "payload");
            if (cJSON_IsObject(payload2))
                led = cJSON_GetObjectItemCaseSensitive(payload2, "led");
        }
    }
    if (cJSON_IsBool(led))
    {
        bool st = cJSON_IsTrue(led);
        size_t cnt = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
        for (uint8_t i = 1; i <= (uint8_t)cnt; ++i)
        {
            if (i == 1)
            {
                if (st)
                    rgb_led_set(0, 255, 0);
                else
                    rgb_led_set(255, 255, 255);
            }
            else
            {
                gpio_set_level(LED_PINS[i - 1], _to_level(st));
            }
        }
        handled = true;
        if (out_info && out_info_len)
            strncpy(out_info, "led all", out_info_len - 1);
        goto out;
    }

    // 2) {"led": {"id":1, "state":true}}
    if (cJSON_IsObject(led))
    {
        cJSON *id = cJSON_GetObjectItemCaseSensitive(led, "id");
        cJSON *state = cJSON_GetObjectItemCaseSensitive(led, "state");
        if (cJSON_IsNumber(id) && cJSON_IsBool(state))
        {
            int iid = (int)id->valueint;
            bool st = cJSON_IsTrue(state);
            if (iid == 1)
            {
                if (st)
                    rgb_led_set(0, 255, 0);
                else
                    rgb_led_set(255, 255, 255);
            }
            else
            {
                size_t cnt = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
                if (iid >= 1 && (size_t)iid <= cnt)
                {
                    gpio_set_level(LED_PINS[iid - 1], _to_level(st));
                }
            }
            handled = true;
            if (out_info && out_info_len)
                strncpy(out_info, "led one", out_info_len - 1);
            goto out;
        }
    }

    // 3) {"leds": {"ids":[1,2,3], "state":false}}  (soportar también nested payload)
    cJSON *leds = cJSON_GetObjectItemCaseSensitive(root, "leds");
    if (!leds)
    {
        cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
        if (cJSON_IsObject(payload))
        {
            leds = cJSON_GetObjectItemCaseSensitive(payload, "leds");
            if (!leds)
            {
                cJSON *payload2 = cJSON_GetObjectItemCaseSensitive(payload, "payload");
                if (cJSON_IsObject(payload2))
                    leds = cJSON_GetObjectItemCaseSensitive(payload2, "leds");
            }
        }
    }
    if (cJSON_IsObject(leds))
    {
        cJSON *ids = cJSON_GetObjectItemCaseSensitive(leds, "ids");
        cJSON *state = cJSON_GetObjectItemCaseSensitive(leds, "state");
        if (cJSON_IsArray(ids) && cJSON_IsBool(state))
        {
            apply_leds_many_ids(ids, cJSON_IsTrue(state));
            handled = true;
            if (out_info && out_info_len)
                strncpy(out_info, "leds many", out_info_len - 1);
            goto out;
        }
    }
    // --- MODE ---
    cJSON *mode = cJSON_GetObjectItemCaseSensitive(root, "mode");
    if (!mode)
    {
        cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
        if (cJSON_IsObject(payload))
        {
            mode = cJSON_GetObjectItemCaseSensitive(payload, "mode");
        }
    }
    if (cJSON_IsNumber(mode) || cJSON_IsString(mode))
    {
        int newMode = -1;
        if (cJSON_IsNumber(mode))
        {
            newMode = mode->valueint;
        }
        else
        {
            const char *s = mode->valuestring;
            if (s)
            {
                if (strcasecmp(s, "piloto") == 0 || strcasecmp(s, "pilot") == 0)
                    newMode = 0;
                else if (strcasecmp(s, "manual") == 0)
                    newMode = 2;
                else if (strcasecmp(s, "espera") == 0 || strcasecmp(s, "idle") == 0)
                    newMode = 1;
            }
        }
        if (newMode >= 0)
        {
            changeMode((mode_t)newMode);
            handled = true;
            if (out_info && out_info_len)
                strncpy(out_info, "mode_changed", out_info_len - 1);
            goto out;
        }
    }

out:
    cJSON_Delete(root);
    if (!handled && out_info && out_info_len)
        strncpy(out_info, "unknown_command_or_schema", out_info_len - 1);
    return handled;
}
static void send_ack(int sock, const struct sockaddr_in *to, bool ok, const char *request_id, const char *info)
{
    char resp[256];
    const char *info_safe = (info && info[0]) ? info : "";
    if (request_id && request_id[0] != '\0')
    {
        int n = snprintf(resp, sizeof(resp), "{\"type\":\"ack\",\"request_id\":\"%s\",\"ok\":%s,\"info\":\"%s\"}", request_id, ok ? "true" : "false", info_safe);
        if (n > 0)
            sendto(sock, resp, n, 0, (const struct sockaddr *)to, sizeof(*to));
    }
    else
    {
        int n = snprintf(resp, sizeof(resp), "{\"type\":\"ack\",\"ok\":%s,\"info\":\"%s\"}", ok ? "true" : "false", info_safe);
        if (n > 0)
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
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGI(TAG, "escuchando UDP en 0.0.0.0:%u", (unsigned)listen_port);

    char buf[512];

    for (;;)
    {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &flen);

        if (n > 0)
        {
            buf[n] = '\0';
            ESP_LOGI(TAG, "rx %dB de %s:%u -> %s",
                     n, inet_ntoa(from.sin_addr), (unsigned)ntohs(from.sin_port), buf);

            // extraer request_id si existe en root.payload.request_id o root.request_id
            char reqid[64] = {0};
            cJSON *root = cJSON_ParseWithLength(buf, n);
            if (root)
            {
                cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
                cJSON *rid = NULL;
                if (cJSON_IsObject(payload))
                    rid = cJSON_GetObjectItemCaseSensitive(payload, "request_id");
                if (!rid)
                    rid = cJSON_GetObjectItemCaseSensitive(root, "request_id");
                if (rid && cJSON_IsString(rid) && rid->valuestring)
                {
                    strncpy(reqid, rid->valuestring, sizeof(reqid) - 1);
                }
                cJSON_Delete(root);
            }

            char info[128] = {0};
            bool ok = handle_json(buf, n, info, sizeof(info));
            send_ack(sock, &from, ok, reqid[0] ? reqid : NULL, info);
        }

        // Añade un pequeño delay para cooperar con el scheduler
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool cmd_led_start_core0(uint16_t listen_port, int priority)
{
    ESP_LOGI(TAG, "cmd_led_start_core0: llamada (port=%u, prio=%d)", (unsigned)listen_port, priority);
    if (s_cmd_task)
        return true;
    BaseType_t ok = xTaskCreatePinnedToCore(
        cmd_led_task, "cmd_led", 4096, (void *)(uintptr_t)listen_port, priority, &s_cmd_task, 1);

    return ok == pdPASS;
}
