#include "motor_ctrl.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>
#include "cmd_motor.h"

#define TAG "CMD_MOTOR"
#define CMD_PORTS_COUNT 2

void handle_motor_cmd(cJSON *root)
{
    cJSON *payload = cJSON_GetObjectItem(root, "payload");
    if (payload)
        root = payload;

    cJSON *motor = cJSON_GetObjectItem(root, "motor");
    if (cJSON_IsObject(motor))
    {
        cJSON *id = cJSON_GetObjectItem(motor, "id");
        cJSON *state = cJSON_GetObjectItem(motor, "state");
        cJSON *speed = cJSON_GetObjectItem(motor, "speed");

        if (!id)
            return;

        int mid = id->valueint;
        if (state && cJSON_IsBool(state))
            motor_ctrl_set_state(mid, cJSON_IsTrue(state));
        else if (speed && cJSON_IsNumber(speed))
            motor_ctrl_set_speed(mid, speed->valueint);
        return;
    }

    cJSON *motors = cJSON_GetObjectItem(root, "motors");
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
                int mid = it->valueint;
                if (state && cJSON_IsBool(state))
                    motor_ctrl_set_state(mid, cJSON_IsTrue(state));
                else if (speed && cJSON_IsNumber(speed))
                    motor_ctrl_set_speed(mid, speed->valueint);
            }
        }
        else if (cJSON_IsBool(state))
            motor_ctrl_set_all(cJSON_IsTrue(state));
        else if (cJSON_IsNumber(speed))
            motor_ctrl_set_all_speed(speed->valueint);
    }
}
