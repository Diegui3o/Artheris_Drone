#include "motor_ctrl.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>

#define TAG "CMD_MOTOR"
#define CMD_PORTS_COUNT 2

// Function declarations
static void cmd_motor_task(void *arg);
bool cmd_motor_start(uint16_t port, int prio);

void handle_motor_cmd(cJSON *root)
{
    cJSON *payload = cJSON_GetObjectItem(root, "payload");
    if (payload)
        root = payload;

    ESP_LOGI(TAG, "handle_motor_cmd(): JSON recibido correctamente");

    // --- {"motor":{...}} ---
    cJSON *motor = cJSON_GetObjectItem(root, "motor");
    if (cJSON_IsObject(motor))
    {
        ESP_LOGI(TAG, "Detectado objeto 'motor'");

        cJSON *id_obj = cJSON_GetObjectItem(motor, "id");
        cJSON *state = cJSON_GetObjectItem(motor, "state");
        cJSON *speed = cJSON_GetObjectItem(motor, "speed");

        if (!id_obj)
        {
            ESP_LOGE(TAG, "Error: falta 'id' en motor");
            return;
        }

        int id = id_obj->valueint;
        ESP_LOGI(TAG, "Motor ID = %d", id);

        if (state)
            ESP_LOGI(TAG, "Campo 'state' detectado tipo=%d valor=%s",
                     state->type, cJSON_IsTrue(state) ? "true" : "false");

        if (speed)
            ESP_LOGI(TAG, "Campo 'speed' detectado tipo=%d valor=%d",
                     speed->type, speed->valueint);

        if (state && speed)
        {
            ESP_LOGE(TAG, "Error: 'state' y 'speed' no pueden coexistir");
            return;
        }

        if (state && cJSON_IsBool(state))
        {
            ESP_LOGI(TAG, "→ motor_ctrl_set_state(%d, %d)", id, cJSON_IsTrue(state));
            motor_ctrl_set_state(id, cJSON_IsTrue(state));
        }
        else if (speed && cJSON_IsNumber(speed))
        {
            ESP_LOGI(TAG, "→ motor_ctrl_set_speed(%d, %d)", id, speed->valueint);
            motor_ctrl_set_speed(id, speed->valueint);
        }
    }
    else
    {
        ESP_LOGW(TAG, "No se encontró objeto 'motor'");
    }

    // --- {"motors":{...}} ---
    cJSON *motors = cJSON_GetObjectItem(root, "motors");
    if (cJSON_IsObject(motors))
    {
        ESP_LOGI(TAG, "Detectado objeto 'motors'");

        cJSON *ids = cJSON_GetObjectItem(motors, "ids");
        cJSON *state = cJSON_GetObjectItem(motors, "state");
        cJSON *speed = cJSON_GetObjectItem(motors, "speed");

        if (cJSON_IsArray(ids))
        {
            ESP_LOGI(TAG, "Procesando arreglo de IDs");
            cJSON *it = NULL;
            cJSON_ArrayForEach(it, ids)
            {
                int id = it->valueint;
                ESP_LOGI(TAG, "ID actual: %d", id);

                if (cJSON_IsBool(state))
                {
                    ESP_LOGI(TAG, "→ motor_ctrl_set_state(%d, %d)", id, cJSON_IsTrue(state));
                    motor_ctrl_set_state(id, cJSON_IsTrue(state));
                }
                if (cJSON_IsNumber(speed))
                {
                    ESP_LOGI(TAG, "→ motor_ctrl_set_speed(%d, %d)", id, speed->valueint);
                    motor_ctrl_set_speed(id, speed->valueint);
                }
            }
        }
        else if (cJSON_IsBool(state))
        {
            ESP_LOGI(TAG, "→ motor_ctrl_set_all(%d)", cJSON_IsTrue(state));
            motor_ctrl_set_all(cJSON_IsTrue(state));
        }
        else if (cJSON_IsNumber(speed))
        {
            ESP_LOGI(TAG, "→ motor_ctrl_set_all_speed(%d)", speed->valueint);
            motor_ctrl_set_all_speed(speed->valueint);
        }
    }
    else
    {
        ESP_LOGW(TAG, "No se encontró objeto 'motors'");
    }
}

static void cmd_motor_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in me = {0};
    me.sin_family = AF_INET;
    me.sin_port = htons(8889);
    me.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(sock, (struct sockaddr *)&me, sizeof(me));
    ESP_LOGI(TAG, "✅ Escuchando UDP en 0.0.0.0:8889");

    char buf[512];
    struct sockaddr_in from;
    socklen_t flen = sizeof(from);

    for (;;) // <--- bucle infinito principal
    {
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &flen);
        if (n <= 0)
            continue;

        buf[n] = '\0';
        ESP_LOGI(TAG, "📩 Recibido (%d bytes) de %s:%u -> %s",
                 n, inet_ntoa(from.sin_addr), ntohs(from.sin_port), buf);

        cJSON *root = cJSON_Parse(buf);
        if (!root)
        {
            ESP_LOGE(TAG, "❌ Error parseando JSON");
            continue;
        }

        handle_motor_cmd(root);
        cJSON_Delete(root);
    }

    close(sock); // nunca se alcanza normalmente
    vTaskDelete(NULL);
}

bool cmd_motor_start(uint16_t port /*(opcional, ya no lo usas)*/, int prio)
{
    // Si vas a escuchar en 8887 y 8889 siempre, el 'port' no es necesario.
    (void)port;
    return xTaskCreatePinnedToCore(cmd_motor_task, "cmd_motor",
                                   4096, NULL, prio, NULL, 0) == pdPASS;
}
