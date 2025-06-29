#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include <esp_task_wdt.h>
#include "mpu.h"
#include "motores.h"

// Límites de integral por eje (ajustados a las ganancias actuales)
#define MAX_INTEGRAL_ROLL_PITCH 100.0f // Para Ki=0.6: permite τ máximo de ~60
#define MAX_INTEGRAL_YAW 300.0f        // Para Ki=0.1: permite τ máximo de ~30
#define TORQUE_SCALE 20.0f

// Variable to track MPU calibration status
bool mpu_ready = false;

// === Función de saturación para modo deslizante ===
float sat(float value, float threshold)
{
    if (value > threshold)
        return 1.0;
    else if (value < -threshold)
        return -1.0;
    else
        return value / threshold;
}

// === Matrices LQR ===
float Ki_at[3][3] = {
    {0.1, 0, 0},
    {0, 0.1, 0},
    {0, 0, 0.01}};

float Kc_at[3][6] = {
    {5.5, 0, 0, 3.6, 0, 0},
    {0, 5.5, 0, 0, 3.6, 0},
    {0, 0, 5.3, 0, 0, 1.6}};

// === SETUP INICIAL ===
void setup_pilote_mode()
{
    pinMode(pinLed, OUTPUT);
    Serial.begin(115200);
    Serial.println("Iniciando modo pilote...");
    delay(100);
    Serial.println("Setup completado.");
}

void loop_pilote_mode(float dt)
{
    // 1. Leer valores del receptor (igual)
    DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
    InputThrottle = ReceiverValue[2];
    DesiredAngleYaw = 0.15 * (ReceiverValue[3] - 1500);

    if (InputThrottle > 1020 && InputThrottle < 2000)
    {
        // === ESCALADO ADAPTATIVO ===
        const float throttle_nominal = 1600.0;
        const float gainSlope = 0.002;
        float gain_scale = constrain(1.0 + gainSlope * (throttle_nominal - InputThrottle), 0.7, 1.3);

        // Actualiza las matrices directamente
        Kc_at[0][0] = 1.92 * gain_scale;
        Kc_at[0][3] = 0.38 * gain_scale;
        Kc_at[1][1] = 1.92 * gain_scale;
        Kc_at[1][4] = 0.38 * gain_scale;
        Kc_at[2][2] = 0.5 * gain_scale;
        Kc_at[2][5] = 0.15 * gain_scale;

        Ki_at[0][0] = 0.04 * gain_scale;
        Ki_at[1][1] = 0.04 * gain_scale;
        Ki_at[2][2] = 0.01 * gain_scale;

        // 2. Convertir TODO a radianes
        phi_ref = (DesiredAngleRoll / 2.5) * DEG_TO_RAD;
        theta_ref = (DesiredAnglePitch / 2.5) * DEG_TO_RAD;
        psi_ref = (DesiredAngleYaw / 2.5) * DEG_TO_RAD;

        // Estados actuales
        float roll_rad = AngleRoll * DEG_TO_RAD;
        float pitch_rad = AnglePitch * DEG_TO_RAD;
        float yaw_rad = AngleYaw * DEG_TO_RAD;
        float gyroRoll_rad = gyroRateRoll * DEG_TO_RAD;
        float gyroPitch_rad = gyroRatePitch * DEG_TO_RAD;
        float gyroYaw_rad = RateYaw * DEG_TO_RAD;

        float x_c[6] = {
            roll_rad, pitch_rad, yaw_rad,
            gyroRoll_rad, gyroPitch_rad, gyroYaw_rad};

        // 3. Calcular errores
        error_phi = phi_ref - roll_rad;
        error_theta = theta_ref - pitch_rad;
        error_psi = psi_ref - yaw_rad;

        // 4. Integrales con saturación
        integral_phi = constrain(integral_phi + error_phi * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
        integral_theta = constrain(integral_theta + error_theta * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
        integral_psi = constrain(integral_psi + error_psi * dt, -MAX_INTEGRAL_YAW, MAX_INTEGRAL_YAW);

        // Aplicar control LQR adaptado
        tau_x = gain_scale * (Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi) - Kc_at[0][3] * x_c[3];
        tau_y = gain_scale * (Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta) - Kc_at[1][4] * x_c[4];
        tau_z = gain_scale * (Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi) - Kc_at[2][5] * x_c[5];

        // Escalar torques a PWM
        tau_x *= TORQUE_SCALE;
        tau_y *= TORQUE_SCALE;
        tau_z *= TORQUE_SCALE;
        applyControl(tau_x, tau_y, tau_z);
    }
    else
    {
        applyControl(0, 0, 0);
        apagarMotores();
        integral_phi = integral_theta = integral_psi = 0;
    }
}