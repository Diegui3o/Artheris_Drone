#include "mode_control.h"
#include "led.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MODE_CTRL";

static volatile drone_mode_t modoActual = MODE_WAIT;
static drone_mode_t lastMode = MODE_WAIT;

drone_mode_t getMode(void)
{
    return modoActual;
}

void changeMode(drone_mode_t newMode)
{
    if (newMode == modoActual)
        return;
    drone_mode_t prev = modoActual;
    modoActual = newMode;
    ESP_LOGI(TAG, "Modo cambiado: %d -> %d", prev, newMode);

    // Ajuste visual inmediato:
    switch (modoActual)
    {
    case MODE_PILOT:
        rgb_led_set(255, 0, 0); // color rojo
        ESP_LOGI(TAG, "Modo piloto activado");
        break;
    case MODE_MANUAL:
        rgb_led_set(0, 0, 255); // color azul
        ESP_LOGI(TAG, "Modo manual activado");
        break;
    case MODE_WAIT:
    default:
        rgb_led_set(255, 255, 255);
        ESP_LOGI(TAG, "Modo espera activado");
        break;
    }
}

static void mode_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Tarea de modo iniciada en core 1");

    for (;;)
    {
        if (modoActual != lastMode)
        {
            lastMode = modoActual;
            ESP_LOGI(TAG, "Detectado cambio de modo: %d", modoActual);
            // Aquí lógica adicional por modo
            switch (modoActual)
            {
            case MODE_PILOT:
                ESP_LOGI(TAG, "Acción: piloto activado");
                // setup_pilote_mode();
                break;
            case MODE_MANUAL:
                ESP_LOGI(TAG, "Acción: manual activado");
                // setup_manual_mode();
                break;
            case MODE_WAIT:
            default:
                ESP_LOGI(TAG, "Acción: espera activado");
                // apagarMotores();
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

void mode_control_start_core1(int priority)
{
    xTaskCreatePinnedToCore(mode_control_task, "mode_ctrl", 4096, NULL, priority, NULL, 1);
}
