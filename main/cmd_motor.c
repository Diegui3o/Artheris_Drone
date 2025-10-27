#include "motor_ctrl.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>

#define TAG "CMD_MOTOR"

// Function declarations
static void cmd_motor_task(void *arg);
bool cmd_motor_start(uint16_t port, int prio);

void handle_motor_cmd(cJSON *root)
{
    // Comando rápido: {"arm":true} para iniciar la secuencia de calibración/arming
    cJSON *arm_cmd = cJSON_GetObjectItemCaseSensitive(root, "arm");
    if (arm_cmd && cJSON_IsBool(arm_cmd) && cJSON_IsTrue(arm_cmd))
    {
        ESP_LOGI(TAG, "Recibido comando ARM -> iniciando motor_ctrl_arm_sequence()");
        motor_ctrl_arm_sequence();
        return;
    }

    // --- {"motor":{"id":1,"state":true}} ---
    cJSON *motor = cJSON_GetObjectItemCaseSensitive(root, "motor");
    if (cJSON_IsObject(motor))
    {
        cJSON *id_obj = cJSON_GetObjectItem(motor, "id");
        cJSON *state = cJSON_GetObjectItem(motor, "state");
        cJSON *speed = cJSON_GetObjectItem(motor, "speed");

        if (!id_obj)
        {
            ESP_LOGE(TAG, "Error: motor command missing id");
            return;
        }
        int id = id_obj->valueint;

        if (state && speed)
        {
            ESP_LOGE(TAG, "Error: both state and speed specified");
            return;
        }

        if (state && cJSON_IsBool(state))
        {
            ESP_LOGI(TAG, "Setting motor %d state to %s", id, cJSON_IsTrue(state) ? "ON" : "OFF");
            motor_ctrl_set_state(id, cJSON_IsTrue(state));
        }
        else if (speed && cJSON_IsNumber(speed))
        {
            ESP_LOGI(TAG, "Setting motor %d speed to %d us", id, speed->valueint);
            motor_ctrl_set_speed(id, speed->valueint);
        }

        // --- {"motors":{"ids":[1,2,3],"state":false}} ---
        cJSON *motors = cJSON_GetObjectItemCaseSensitive(root, "motors");
        if (cJSON_IsObject(motors))
        {
            cJSON *ids = cJSON_GetObjectItem(motors, "ids");
            cJSON *state = cJSON_GetObjectItem(motors, "state");
            cJSON *speed = cJSON_GetObjectItem(motors, "speed");

            if (cJSON_IsArray(ids))
            {
                cJSON *it = NULL;
                cJSON_ArrayForEach(it, ids)
                {
                    int id = it->valueint;
                    if (cJSON_IsBool(state))
                        motor_ctrl_set_state(id, cJSON_IsTrue(state));
                    if (cJSON_IsNumber(speed))
                        motor_ctrl_set_speed(id, speed->valueint);
                }
            }
            else if (cJSON_IsBool(state))
                motor_ctrl_set_all(cJSON_IsTrue(state));
            else if (cJSON_IsNumber(speed))
                motor_ctrl_set_all_speed(speed->valueint);
        }
    }
}

static void cmd_motor_task(void *arg)
{
    uint16_t port = (uint16_t)(uintptr_t)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in me = {0};
    me.sin_family = AF_INET;
    me.sin_port = htons(port);
    me.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (struct sockaddr *)&me, sizeof(me));

    char buf[512];
    ESP_LOGI(TAG, "Listening UDP on port %u", port);

    for (;;)
    {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &flen);
        if (n > 0)
        {
            buf[n] = '\0';
            ESP_LOGI(TAG, "RX from %s:%u -> %s", inet_ntoa(from.sin_addr), ntohs(from.sin_port), buf);
            cJSON *root = cJSON_Parse(buf);
            if (root)
            {
                handle_motor_cmd(root);
                cJSON_Delete(root);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool cmd_motor_start(uint16_t port, int prio)
{
    return xTaskCreatePinnedToCore(cmd_motor_task, "cmd_motor", 4096, (void *)(uintptr_t)port, prio, NULL, 0) == pdPASS;
}