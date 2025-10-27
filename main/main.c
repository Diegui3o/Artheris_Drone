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

#define I2C_PORT I2C_NUM_0
#define I2C_SDA 8
#define I2C_SCL 9

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    wifi_sta_cfg_t w = {
        .ssid = "FAMILIAMYM",
        .pass = "mm221418",
        .ip = inet_addr("192.168.1.50"),
        .gw = inet_addr("192.168.1.1"),
        .netmask = inet_addr("255.255.255.0"),
        .connect_timeout_ms = 15000,
        .max_retries = 5,
    };
    ESP_ERROR_CHECK(wifi_sta_start(&w));
    ESP_LOGI("APP", "WiFi listo (is_up=%d)", wifi_sta_is_up());

    ESP_LOGI("APP", "ANTES de leds_init");
    leds_init();
    bool started = cmd_led_start_core0(8888, 5);
    ESP_LOGI("APP", "cmd_led_start_core0 -> %s", started ? "OK" : "FAIL");
    vTaskDelay(pdMS_TO_TICKS(100));

    imu_init(I2C_PORT, I2C_SDA, I2C_SCL, 400000);       // 400 kHz
    imu_set_acc_offsets(/*ax*/ 0, /*ay*/ 0, /*az*/ 0);  // pon tus offsets
    imu_set_gyro_offsets(/*gx*/ 0, /*gy*/ 0, /*gz*/ 0); // pon tus offsets
    imu_start_1khz(/*core=*/1, /*prio=*/18);            // productor a 1 kHz
    attitude_start(/*core=*/1, /*prio=*/19, /*q_angle=*/0.001f, /*q_bias=*/0.003f, /*r_measure=*/0.03f);
    telemetry_start_core0("192.168.1.36", 8889, 5);

    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 20, NULL, 1); // Core 1
}
