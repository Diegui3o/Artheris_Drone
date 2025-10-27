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
#include "cmd_led.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA 8
#define I2C_SCL 9

static const char *TAG = "APP";

void app_main(void)
{
    ESP_LOGI(TAG, "==== Inicio de app_main ====");
    esp_log_level_set("APP", ESP_LOG_INFO);

    esp_err_t err;
    err = esp_wifi_disconnect();
    if (err == ESP_OK)
    {
        ESP_LOGI("APP", "WiFi disconnected");
    }

    // 2️⃣ Parar WiFi
    err = esp_wifi_stop();
    if (err == ESP_OK)
    {
        ESP_LOGI("APP", "WiFi stopped");
    }

    // 3️⃣ De-inicializar WiFi driver
    err = esp_wifi_deinit();
    if (err == ESP_OK)
    {
        ESP_LOGI("APP", "WiFi deinitialized");
    }

    // 4️⃣ De-inicializar netif stack
    err = esp_netif_deinit();
    if (err == ESP_OK)
    {
        ESP_LOGI("APP", "Network interface stack deinitialized");
    }

    // 5️⃣ Eliminar el loop de eventos por defecto
    err = esp_event_loop_delete_default();
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE)
    {
        // ESP_ERR_INVALID_STATE se puede ignorar si no había loop creado
        ESP_LOGI("APP", "Default event loop deleted (or not existed)");
    }
    else
    {
        ESP_LOGW("APP", "Failed deleting event loop: 0x%x", err);
    }

    // 6️⃣ Pequeña espera para que todo se "calme"
    vTaskDelay(pdMS_TO_TICKS(100));

    // --- 1️⃣ WiFi STA inicialización ---
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
    err = wifi_sta_start(&w);
    int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    ESP_LOGI(TAG, "wifi_sta_start -> %s en %lld ms (is_up=%d)",
             (err == ESP_OK) ? "OK" : esp_err_to_name(err),
             (long long)elapsed_ms,
             wifi_sta_is_up());

    // --- 2️⃣ LED system ---
    ESP_LOGI(TAG, "Inicializando LEDs...");
    leds_init();
    bool started = cmd_led_start_core0(8888, 5);
    ESP_LOGI(TAG, "cmd_led_start_core0 -> %s", started ? "OK" : "FAIL");

    // Delay de seguridad para que no bloquee el siguiente init
    vTaskDelay(pdMS_TO_TICKS(200));

    // --- 3️⃣ IMU inicialización ---
    ESP_LOGI(TAG, "Inicializando IMU...");
    imu_init(I2C_PORT, I2C_SDA, I2C_SCL, 400000); // 400 kHz
    imu_set_acc_offsets(0, 0, 0);
    imu_set_gyro_offsets(0, 0, 0);

    ESP_LOGI(TAG, "Iniciando IMU tasks...");
    imu_start_1khz(/*core=*/1, /*prio=*/18);
    attitude_start(/*core=*/1, /*prio=*/19, 0.001f, 0.003f, 0.03f);

    // --- 4️⃣ Telemetría ---
    ESP_LOGI(TAG, "Inicializando telemetría UDP...");
    telemetry_start_core0("192.168.1.36", 8889, 5);

    // --- 5️⃣ Control Loop ---
    ESP_LOGI(TAG, "Creando tarea de control...");
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 20, NULL, 1);

    ESP_LOGI(TAG, "==== Sistema iniciado correctamente ====");

    // --- 6️⃣ Monitoreo (no dejar app_main vacío) ---
    while (1)
    {
        bool wifi_ok = wifi_sta_is_up();
        ESP_LOGI(TAG, "Heartbeat: WiFi=%d  FreeHeap=%u", wifi_ok, esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
