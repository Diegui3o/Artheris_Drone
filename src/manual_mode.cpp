#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "manual_mode.h"
#include "motores.h"
#include <math.h>

// Límites de integral por eje (ajustados a las ganancias actuales)
#define MAX_INTEGRAL_ROLL_PITCH 100.0f // Para Ki=0.6: permite τ máximo de ~60
#define MAX_INTEGRAL_YAW 300.0f        // Para Ki=0.1: permite τ máximo de ~30
#define TORQUE_SCALE 120.0

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
    ReceiverValue[0] = constrain(current_time - timer_1, 1000, 2000);
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
    ReceiverValue[1] = constrain(current_time - timer_2, 1000, 2000);
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
    ReceiverValue[2] = constrain(current_time - timer_3, 1000, 2000);
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
    ReceiverValue[3] = constrain(current_time - timer_4, 1000, 2000);
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
    ReceiverValue[4] = constrain(current_time - timer_5, 1000, 2000);
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
    ReceiverValue[5] = constrain(current_time - timer_6, 1000, 2000);
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
  // Limitar valores de receptor antes de usarlos
  ReceiverValue[0] = constrain(ReceiverValue[0], 1000, 2000);
  ReceiverValue[1] = constrain(ReceiverValue[1], 1000, 2000);
  ReceiverValue[2] = constrain(ReceiverValue[2], 1000, 2000);
  ReceiverValue[3] = constrain(ReceiverValue[3], 1000, 2000);
  ReceiverValue[4] = constrain(ReceiverValue[4], 1000, 2000);
  ReceiverValue[5] = constrain(ReceiverValue[5], 1000, 2000);

  DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = constrain(ReceiverValue[2], 1000, 2000);
  DesiredAngleYaw = 0.15 * (ReceiverValue[3] - 1500);

  if (InputThrottle > 1020 && InputThrottle < 2000)
  {
    k1 = 2.1 + (1.001 / (1 + pow((InputThrottle / 1355.318), 42.52)));
    Kc_at[0][0] = k1;
    k2 = 1.92 + (1.001 / (1 + pow((InputThrottle / 1355.318), 42.52)));
    Kc_at[1][1] = k2;

    g1 = 0.58 + (1 / (1 + pow((InputThrottle / 1355.318), 42.52)));
    Kc_at[0][3] = g1;
    g2 = 0.38 + (1 / (1 + pow((InputThrottle / 1355.318), 42.52)));
    Kc_at[1][4] = g2;

    // 2. Convertir TODO a radianes (usar macro de Arduino)
    phi_ref = (DesiredAngleRoll / 2.8) * DEG_TO_RAD;
    theta_ref = (DesiredAnglePitch / 2.8) * DEG_TO_RAD;
    psi_ref = (DesiredAngleYaw / 2.8) * DEG_TO_RAD;

    // Aplicar zona muerta de ±3° (en radianes ~0.052 rad)
    if (abs(phi_ref) < 3.0 * DEG_TO_RAD)
    {
      phi_ref = 0.00;
    }
    if (abs(theta_ref) < 3.0 * DEG_TO_RAD)
    {
      theta_ref = 0.00;
    }
    if (abs(psi_ref) < 3.0 * DEG_TO_RAD)
    {
      psi_ref = 0.00;
    }

    // Estados actuales (convertidos)
    float roll_rad = AngleRoll * DEG_TO_RAD;
    float pitch_rad = AnglePitch * DEG_TO_RAD;
    float yaw_rad = AngleYaw * DEG_TO_RAD;
    float gyroRoll_rad = gyroRateRoll * DEG_TO_RAD;
    float gyroPitch_rad = gyroRatePitch * DEG_TO_RAD;
    float gyroYaw_rad = RateYaw * DEG_TO_RAD;

    float x_c[6] = {
        roll_rad, pitch_rad, yaw_rad,
        gyroRoll_rad, gyroPitch_rad, gyroYaw_rad};

    // 3. Calcular errores (en radianes)
    error_phi = phi_ref - roll_rad;
    error_theta = theta_ref - pitch_rad;
    error_psi = psi_ref - AngleYaw;

    // 4. CUARTO: Actualizar integrales con saturación específica por eje
    integral_phi = constrain(integral_phi + error_phi * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
    integral_theta = constrain(integral_theta + error_theta * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
    integral_psi = constrain(integral_psi + error_psi * dt, -MAX_INTEGRAL_YAW, MAX_INTEGRAL_YAW);

    // 5. QUINTO: Control LQR usando las integrales actualizadas
    tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
    tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
    tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi - Kc_at[2][5] * RateYaw;

    // 5. Escalar torques a PWM (ejemplo: 500 μs/N·m)
    tau_x *= TORQUE_SCALE;
    tau_y *= TORQUE_SCALE;

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