
#include "motor_state.h"
#include "esp_log.h"

uint16_t motor_inputs[NUM_MOTORS] = {0};
uint16_t motor_vals[NUM_MOTORS] = {0};

// Función para verificar estado
void motor_state_debug(void)
{
    ESP_LOGI("MOTOR_STATE", "motor_vals: [%u, %u, %u, %u]",
             motor_vals[0], motor_vals[1], motor_vals[2], motor_vals[3]);
}