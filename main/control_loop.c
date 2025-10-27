#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_attr.h"

static TaskHandle_t s_ctrl_handle = NULL;

// Timer de alta resolución que despierta la tarea de control
static void IRAM_ATTR control_timer_cb(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &hp);
    if (hp)
        portYIELD_FROM_ISR();
}

void control_task(void *arg)
{
    s_ctrl_handle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(NULL);

    const esp_timer_create_args_t t_args = {
        .callback = &control_timer_cb,
        .arg = s_ctrl_handle,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ctrl_tmr"};
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&t_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000)); // 1 kHz

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // ... tu ciclo de control ...
        esp_task_wdt_reset();
    }
}