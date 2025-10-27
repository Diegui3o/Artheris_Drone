#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

// NOTE: Si tu placa tiene LEDs físicos distintos del LED RGB integrado,
// ajusta aquí los pines. Por simplicidad y para evitar conflictos con
// el controlador RMT usado por el LED RGB (pin 48 en este proyecto),
// dejamos un pin de ejemplo (GPIO2). Si no tienes LEDs físicos,
// esta definición sirve para compilar pero no afecta al LED RGB.
#define LED_ACTIVE_HIGH 1
static const gpio_num_t LED_PINS[] = { GPIO_NUM_2 };

// RGB integrado (usa led_strip RMT en led.c)
void rgb_led_init(void);
void rgb_led_set(uint8_t r, uint8_t g, uint8_t b);
void rgb_led_off(void);

// API para LEDs digitales (led.c)
void leds_init(void);
uint8_t leds_count(void);
bool led_set(uint8_t id, bool on);
bool led_toggle(uint8_t id);

#endif