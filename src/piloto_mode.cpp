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

// Variable to track MPU calibration status
bool mpu_ready = false;

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {1.00, 0, 0},
    {0, 1.00, 0},
    {0, 0, 0.162}};

const float Kc_at[3][6] = {
    {5.98, 0, 0, 3.57, 0, 0},
    {0, 5.99, 0, 0, 3.58, 0},
    {0, 0, 0.97864, 0, 0, 1.00}};

// === Matrices LQR para altitud ===
const float Ki_alt = 31.6228;
const float Kc_alt[2] = {28.8910, 10.5624};

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
    InputThrottle = 1500;

    // Estados actuales: [ángulo, velocidad angular]
    float x_roll[2] = {AngleRoll, gyroRateRoll};
    float x_pitch[2] = {AnglePitch, gyroRatePitch};
    float x_yaw[2] = {AngleYaw, RateYaw};

    // Calcular errores
    error_phi = phi_ref - x_roll[0];
    error_theta = theta_ref - x_pitch[0];
    error_psi = psi_ref - x_yaw[0];

    // Control LQR (actitud + integral)
    tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_roll[1];
    tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][3] * x_pitch[1];
    tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_yaw[1];

    if (InputThrottle > 1020)
    {
        applyControl(tau_x, tau_y, tau_z);
    }
    else
    {
        applyControl(0, 0, 0);
        apagarMotores();
    }
}