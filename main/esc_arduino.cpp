#include <Arduino.h>
#include <ESP32Servo.h>
#include "esc_config.h"

extern "C" {
  void esc_setup(void);
  bool esc_self_test(uint32_t min_hold_ms, char* report_buf, size_t report_len);
  bool esc_all_attached(void);
  void esc_set_pulse_us(int idx, int us);
  void esc_set_all_us(int us);
}

static const int ESC_PINS[MOTOR_COUNT] = {
  MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4
};

static Servo esc[MOTOR_COUNT];

typedef struct {
  bool  attached;
  int   channel;     // canal devuelto por attach(); 0 => fallo en esta lib
  int   last_us;     // último pulso escrito en us
  int   pin;
} EscStatus;

static EscStatus s[MOTOR_COUNT];

static inline int clamp_us(int us) {
  if (us < PWM_MIN_US) return PWM_MIN_US;
  if (us > PWM_MAX_US) return PWM_MAX_US;
  return us;
}

extern "C" void esc_set_pulse_us(int idx, int us) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
  us = clamp_us(us);
  if (s[idx].attached) {
    esc[idx].writeMicroseconds(us);
    s[idx].last_us = us;
  }
}

extern "C" void esc_set_all_us(int us) {
  us = clamp_us(us);
  for (int i = 0; i < MOTOR_COUNT; ++i) esc_set_pulse_us(i, us);
}

extern "C" bool esc_all_attached(void) {
  for (int i = 0; i < MOTOR_COUNT; ++i)
    if (!s[i].attached || s[i].channel == 0) return false;
  return true;
}

extern "C" void esc_setup(void)
{
  // Reservar timers de Servo una sola vez (idempotente)
  static bool timers_ready = false;
  if (!timers_ready) {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    timers_ready = true;
  }

  // Inicializa estados
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    s[i].attached = false;
    s[i].channel  = 0;
    s[i].last_us  = 0;
    s[i].pin      = ESC_PINS[i];

    // Asegura que el pin NO esté tomado por LEDC de antes
    ledcDetach(ESC_PINS[i]);
    if (esc[i].attached()) esc[i].detach();

    // Configura 50 Hz y adjunta con rango (3 parámetros)
    esc[i].setPeriodHertz(ESC_FREQ_HZ); // típicamente 50 Hz
    int ch = esc[i].attach(ESC_PINS[i], PWM_MIN_US, PWM_MAX_US);
    s[i].channel  = ch;
    s[i].attached = (ch != 0);

    // Escribe mínimo siempre; si no está adjunto, no enviará
    if (s[i].attached) {
      esc[i].writeMicroseconds(PWM_MIN_US);
      s[i].last_us = PWM_MIN_US;
    }
  }

  // Mantén mínimo un tiempo para el armado (pitidos)
  delay(1000);
  esc_set_all_us(PWM_MIN_US);
}

/**
 * Self-test: NO calibra, solo verifica:
 * - Todos adjuntos (canal != 0)
 * - Mantiene mínimo por `min_hold_ms` (recomendado 1500–2500 ms)
 * Devuelve true si todo OK. Llena un pequeño reporte (opcional).
 */
extern "C" bool esc_self_test(uint32_t min_hold_ms, char* report_buf, size_t report_len) {
  bool ok = true;
  // 1) Todos adjuntos
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    if (!s[i].attached || s[i].channel == 0) {
      ok = false;
    }
  }

  // 2) Mantener mínimo estable el tiempo requerido
  uint32_t t0 = millis();
  esc_set_all_us(PWM_MIN_US);
  while ((millis() - t0) < min_hold_ms) {
    // Reafirma mínimo por seguridad (por si otra parte del código lo cambió)
    esc_set_all_us(PWM_MIN_US);
    delay(20);
  }

  // 3) Estado final: todos siguen adjuntos y en mínimo
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    if (!s[i].attached || s[i].channel == 0 || s[i].last_us != PWM_MIN_US) {
      ok = false;
    }
  }

  if (report_buf && report_len) {
    // Reporte compacto
    // Ej: "M0:OK(ch=5) M1:OK(ch=6) M2:FAIL(ch=0) M3:OK(ch=7)"
    size_t off = 0;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      int n = snprintf(report_buf + off, (off < report_len) ? (report_len - off) : 0,
                       "M%d:%s(ch=%d) ", i,
                       (s[i].attached && s[i].channel != 0) ? "OK" : "FAIL",
                       s[i].channel);
      if (n > 0) off += (size_t)n;
    }
    // Asegurar terminación
    if (report_len) report_buf[report_len - 1] = '\0';
  }

  return ok;
}
