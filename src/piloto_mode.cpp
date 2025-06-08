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
    {0.6, 0, 0},
    {0, 0.6, 0},
    {0, 0, 0.1}};

const float Kc_at[3][6] = {
    {5.5, 0, 0, 3.6, 0, 0},
    {0, 5.5, 0, 0, 3.6, 0},
    {0, 0, 5.3, 0, 0, 1.6}};

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
    // Estado del sistema
    float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
    float x_i[3] = {integral_phi, integral_theta, integral_psi};

    // Control LQR
    tau_x = Ki_at[0][0] * x_i[0] + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
    tau_y = Ki_at[1][1] * x_i[1] + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
    tau_z = Ki_at[2][2] * x_i[2] + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5];

    error_phi = phi_ref - x_c[0];
    error_theta = theta_ref - x_c[1];
    error_psi = psi_ref - x_c[2];

    // Actualizar integrales
    x_i[0] += error_phi * dt;
    x_i[1] += error_theta * dt;
    x_i[2] += error_psi * dt;

    InputThrottle = 1000 * dt;

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