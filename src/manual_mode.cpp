#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "manual_mode.h"
#include "motores.h"

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {10.00, 0, 0},
    {0, 10.00, 0},
    {0, 0, 3.162}};

const float Kc_at[3][6] = {
    {6.4879, 0, 0, 3.209, 0, 0},
    {0, 6.4959, 0, 0, 3.219, 0},
    {0, 0, 2.97864, 0, 0, 1.00}};

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
  digitalWrite(pinLed, HIGH);
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
  digitalWrite(pinLed, LOW);
}

void loop_manual_mode(float dt)
{

  DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

  // Estado del sistema
  float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
  float x_i[3] = {integral_phi, integral_theta, integral_psi};

  // Control LQR
  tau_x = Ki_at[0][0] * x_i[0] + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
  tau_y = Ki_at[1][1] * x_i[1] + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
  tau_z = Ki_at[2][2] * x_i[2] + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5] - 1;

  error_phi = phi_ref - x_c[0];
  error_theta = theta_ref - x_c[1];
  error_psi = 0;

  // Actualizar integrales
  x_i[0] += error_phi * dt;
  x_i[1] += error_theta * dt;
  x_i[2] += error_psi * dt;

  if (InputThrottle > 1050)
  {
    applyControl(tau_x, tau_y, tau_z);
  }
  else
  {
    applyControl(0, 0, 0);
    apagarMotores();
  }
}