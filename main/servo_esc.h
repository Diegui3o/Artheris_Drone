#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void esc_setup(void);
/**
 * Mantiene PWM MIN por `min_hold_ms` para permitir armado (sin calibrar),
 * y llena un reporte compacto. Devuelve true si todos los ESC están OK.
 */
bool esc_self_test(uint32_t min_hold_ms, char* report_buf, size_t report_len);

bool esc_all_attached(void);
void esc_set_pulse_us(int idx, int us);
void esc_set_all_us(int us);

#ifdef __cplusplus
}
#endif
