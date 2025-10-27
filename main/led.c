#include "led.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LED";

// ====== Config ======
static const gpio_num_t LED_PINS[] = {
    GPIO_NUM_43, // LED 1 -> GPIO 43
};

// Si tu hardware enciende con nivel alto:
static const bool LED_ACTIVE_HIGH = true;

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
