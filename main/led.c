#include "led.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RGB";

static led_strip_handle_t led_strip;

void rgb_led_init(void)
{
    // Configuración del LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = 48,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };

    // Configuración RMT backend
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,           // para ESP32-S3 se recomienda false  :contentReference[oaicite:1]{index=1}
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 0 // usa default
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    ESP_LOGI(TAG, "RGB LED inicializado en GPIO48");

    // Inicialmente en amarillo
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 255, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void rgb_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    ESP_LOGI(TAG, "rgb_led_set -> r=%u g=%u b=%u", (unsigned)r, (unsigned)g, (unsigned)b);
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    // small delay to ensure RMT transfer completes and reset time
    vTaskDelay(pdMS_TO_TICKS(1));
}

void rgb_led_off(void)
{
    ESP_LOGI(TAG, "rgb_led_off() called");
    // Use explicit set_pixel to ensure single-pixel boards are cleared
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(1));
}

// ------------------ API para LEDs digitales (no-RGB) ------------------
// Estado en RAM para los pines definidos en led.h
static bool s_led_state[sizeof(LED_PINS) / sizeof(LED_PINS[0])] = {false};

static inline bool led_id_ok(uint8_t n) { return (n >= 1) && (n <= leds_count()); }
static inline gpio_num_t led_pin(uint8_t n) { return LED_PINS[n - 1]; }
static inline int to_level(bool on)
{
    if (LED_ACTIVE_HIGH)
        return on ? 1 : 0;
    else
        return on ? 0 : 1;
}

uint8_t leds_count(void)
{
    return (uint8_t)(sizeof(LED_PINS) / sizeof(LED_PINS[0]));
}

void leds_init(void)
{
    for (size_t i = 0; i < sizeof(LED_PINS) / sizeof(LED_PINS[0]); ++i)
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << LED_PINS[i],
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level(LED_PINS[i], to_level(false)); // arrancar apagados
        s_led_state[i] = false;
    }
    ESP_LOGI(TAG, "LEDs init ok. count=%u", leds_count());
}

bool led_set(uint8_t id, bool on)
{
    if (!led_id_ok(id))
    {
        ESP_LOGW(TAG, "LED id invalido: %u", id);
        return false;
    }
    gpio_set_level(led_pin(id), to_level(on));
    s_led_state[id - 1] = on;
    return true;
}

bool led_toggle(uint8_t id)
{
    if (!led_id_ok(id))
    {
        ESP_LOGW(TAG, "LED id invalido: %u", id);
        return false;
    }
    return led_set(id, !s_led_state[id - 1]);
}