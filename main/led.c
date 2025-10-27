#include "led.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LED";

// RMT-based minimal driver para un solo LED WS2812 (GRB)
static rmt_channel_t s_rmt_channel = RMT_CHANNEL_0;
static gpio_num_t s_rmt_gpio = GPIO_NUM_48; // pin del LED RGB integrado

// Timings en ticks para resolución de 10MHz (0.1us por tick)
// WS2812 timings aproximados: T0H=0.4us, T0L=0.85us, T1H=0.8us, T1L=0.45us
#define RMT_TICKS_PER_US 10 // 10MHz -> 10 ticks/us
#define WS2812_T0H (4 * RMT_TICKS_PER_US)
#define WS2812_T0L (9 * RMT_TICKS_PER_US)
#define WS2812_T1H (8 * RMT_TICKS_PER_US)
#define WS2812_T1L (5 * RMT_TICKS_PER_US)

void rgb_led_init(void)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(s_rmt_gpio, s_rmt_channel);
    // ajustamos clk_div para tener ~10MHz: APB_CLK (80MHz) / clk_div -> ticks
    // RMT_DEFAULT_CONFIG_TX establece clk_div = 80; en su lugar dejamos por defecto y usamos duraciones en ticks
    config.clk_div = 8; // 80MHz/8 = 10MHz
    esp_err_t err = rmt_config(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_config failed: %d", err);
        return;
    }
    err = rmt_driver_install(config.channel, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_driver_install failed: %d", err);
        return;
    }
}

static void ws2812_write_pixel(uint8_t g, uint8_t r, uint8_t b)
{
    // WS2812 usa orden GRB
    rmt_item32_t items[24];
    uint32_t val = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    for (int i = 0; i < 24; ++i)
    {
        // bit más significativo primero
        bool bit = (val & (1u << (23 - i))) != 0;
        if (bit)
        {
            items[i].level0 = 1;
            items[i].duration0 = WS2812_T1H;
            items[i].level1 = 0;
            items[i].duration1 = WS2812_T1L;
        }
        else
        {
            items[i].level0 = 1;
            items[i].duration0 = WS2812_T0H;
            items[i].level1 = 0;
            items[i].duration1 = WS2812_T0L;
        }
    }
    // Enviar y esperar a que termine
    rmt_write_items(s_rmt_channel, items, 24, true);
    // Reset >50us
    vTaskDelay(pdMS_TO_TICKS(1));
}

void rgb_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_write_pixel(g, r, b);
}

void rgb_led_off(void)
{
    ws2812_write_pixel(0, 0, 0);
}

// Estado en RAM
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
    // ESP_LOGI(TAG, "LED %u -> %s", id, on ? "ON" : "OFF");
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
