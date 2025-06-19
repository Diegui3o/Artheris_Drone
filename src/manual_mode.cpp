#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "manual_mode.h"
#include "motores.h"

// Límites de integral por eje (ajustados a las ganancias actuales)
#define MAX_INTEGRAL_ROLL_PITCH 100.0f // Para Ki=0.6: permite τ máximo de ~60
#define MAX_INTEGRAL_YAW 300.0f        // Para Ki=0.1: permite τ máximo de ~30

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {0.6, 0, 0},
    {0, 0.6, 0},
    {0, 0, 0.1}};

const float Kc_at[3][6] = {
    {5.5, 0, 0, 3.6, 0, 0},
    {0, 5.5, 0, 0, 3.6, 0},
    {0, 0, 5.3, 0, 0, 1.6}};

void channelInterrupHandler()
{
  current_time = micros();
  if (digitalRead(channel_1_pin))
  {
    if (last_channel_1 == 0)
    {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if (last_channel_1 == 1)
  {
    last_channel_1 = 0;
    ReceiverValue[0] = current_time - timer_1;
  }
  if (digitalRead(channel_2_pin))
  {
    if (last_channel_2 == 0)
    {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if (last_channel_2 == 1)
  {
    last_channel_2 = 0;
    ReceiverValue[1] = current_time - timer_2;
  }
  if (digitalRead(channel_3_pin))
  {
    if (last_channel_3 == 0)
    {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if (last_channel_3 == 1)
  {
    last_channel_3 = 0;
    ReceiverValue[2] = current_time - timer_3;
  }
  if (digitalRead(channel_4_pin))
  {
    if (last_channel_4 == 0)
    {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if (last_channel_4 == 1)
  {
    last_channel_4 = 0;
    ReceiverValue[3] = current_time - timer_4;
  }
  if (digitalRead(channel_5_pin))
  {
    if (last_channel_5 == 0)
    {
      last_channel_5 = 1;
      timer_5 = current_time;
    }
  }
  else if (last_channel_5 == 1)
  {
    last_channel_5 = 0;
    ReceiverValue[4] = current_time - timer_5;
  }
  if (digitalRead(channel_6_pin))
  {
    if (last_channel_6 == 0)
    {
      last_channel_6 = 1;
      timer_6 = current_time;
    }
  }
  else if (last_channel_6 == 1)
  {
    last_channel_6 = 0;
    ReceiverValue[5] = current_time - timer_6;
  }
}

// === SETUP INICIAL ===
void setup_manual_mode()
{
  pinMode(pinLed, OUTPUT);
  delay(50);
  Serial.begin(115200);
  Serial.println("Iniciando modo manual...");

  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);
  pinMode(channel_5_pin, INPUT_PULLUP);
  pinMode(channel_6_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterrupHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterrupHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterrupHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterrupHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterrupHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterrupHandler, CHANGE);

  Serial.println("Setup completado.");
}

void loop_manual_mode(float dt)
{
  DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = ReceiverValue[2];
  DesiredAngleYaw = 0.15 * (ReceiverValue[3] - 1500);

  // 1. PRIMERO: Actualizar las referencias
  phi_ref = DesiredAngleRoll / 2.5;
  theta_ref = DesiredAnglePitch / 2.5;
  psi_ref = DesiredAngleYaw / 2.5;

  // 2. SEGUNDO: Estado del sistema
  float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};

  // 3. TERCERO: Calcular errores
  error_phi = phi_ref - x_c[0];
  error_theta = theta_ref - x_c[1];
  error_psi = psi_ref - x_c[2];
  
  // 4. CUARTO: Actualizar integrales con saturación específica por eje
  integral_phi = constrain(integral_phi + error_phi * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
  integral_theta = constrain(integral_theta + error_theta * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
  integral_psi = constrain(integral_psi + error_psi * dt, -MAX_INTEGRAL_YAW, MAX_INTEGRAL_YAW);

  // 5. QUINTO: Control LQR usando las integrales actualizadas
  tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
  tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
  tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5];

  if (InputThrottle > 1020)
  {
    applyControl(tau_x, tau_y, tau_z);
  }
  else
  {
    applyControl(0, 0, 0);
    apagarMotores();
    // Resetear integrales cuando el throttle está bajo
    integral_phi = integral_theta = integral_psi = 0;
  }
}