#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "control_loop.h"
#include "imu.h"
#include "driver/i2c.h"
#include "filter.h"
#include "wifi_sta.h"
#include "telemetry.h"
#include "lwip/inet.h"
#include "led.h"
#include "cmd.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mode_control.h"
#include "motor_ctrl.h"
#include "cmd_motor.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA 8
#define I2C_SCL 9

static const char *TAG = "APP";

static void system_init_task(void *arg)
{
    ESP_LOGI(TAG, "system_init_task: start (core=%d)", xPortGetCoreID());

    // --- WiFi ---
    wifi_sta_cfg_t w = {
        .ssid = "FAMILIAMYM",
        .pass = "mm221418",
        .ip = inet_addr("192.168.1.50"),
        .gw = inet_addr("192.168.1.1"),
        .netmask = inet_addr("255.255.255.0"),
        .connect_timeout_ms = 15000,
        .max_retries = 5,
    };

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = wifi_sta_start(&w);
    int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    ESP_LOGI(TAG, "wifi_sta_start -> %s (is_up=%d) in %lld ms",
             (err == ESP_OK) ? "OK" : esp_err_to_name(err),
             wifi_sta_is_up(), elapsed_ms);

    if (err != ESP_OK || !wifi_sta_is_up())
    {
        ESP_LOGE(TAG, "ERROR: WiFi no listo, abortando inicialización");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Esperando 500 ms para estabilizar red y recursos");
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- LEDs ---
    ESP_LOGI(TAG, "Inicializando LEDs...");
    // Inicializa pines digitales y el LED RGB integrado
    leds_init();
    rgb_led_init();
    // Por defecto empieza en blanco
    rgb_led_set(255, 255, 255);
    ESP_LOGI(TAG, "LEDs inicializados");

    mode_control_start_core1(10);

    motor_ctrl_init();

    // --- CMD LED (server UDP non-blocking en core 0) ---
    bool started = cmd_led_start_core0(8888, 5);
    ESP_LOGI(TAG, "cmd_led_start_core0 -> %s", started ? "OK" : "FAIL");

    // --- IMU ---
    ESP_LOGI(TAG, "Inicializando IMU...");
    imu_init(I2C_PORT, I2C_SDA, I2C_SCL, 400000);
    imu_set_acc_offsets(0, 0, 0);
    imu_set_gyro_offsets(0, 0, 0);
    imu_start_1khz(1, 18);
    attitude_start(1, 19, 0.001f, 0.003f, 0.03f);
    ESP_LOGI(TAG, "IMU iniciado");

    // --- Telemetría ---
    telemetry_start_core0("192.168.1.43", 8889, 5);
    ESP_LOGI(TAG, "Telemetry started");

    // --- Control task ---
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 20, NULL, 1);
    ESP_LOGI(TAG, "Control task creada");

    ESP_LOGI(TAG, "==== Sistema iniciado correctamente ====");

    // Si quieres que la task de init termine:
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "==== Inicio de app_main ====");
    // Creamos la task de inicialización con stack grande (8k) en core 1
    xTaskCreatePinnedToCore(system_init_task, "sys_init", 8192, NULL, 5, NULL, 1);

    // Opcional: mantener app_main viva con un loop ligero
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}