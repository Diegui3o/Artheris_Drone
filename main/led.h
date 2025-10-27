#pragma once
#include <stdbool.h>
#include <stdint.h>

// Inicializa los GPIOs de los LEDs (todos apagados)
void leds_init(void);

// Cuenta de LEDs disponibles (por ahora 1)
uint8_t leds_count(void);

// Enciende/apaga un LED por id (1..N)
bool led_set(uint8_t id, bool on);

// Toggle de un LED por id (1..N)
bool led_toggle(uint8_t id);
