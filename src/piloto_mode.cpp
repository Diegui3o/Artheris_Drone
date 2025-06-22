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
    {4.3205, 0, 0},
    {0, 4.3205, 0},
    {0, 0, 0.873}};

const float Kc_at[3][6] = {
    {3.6739, 0, 0, 0.3867, 0, 0},
    {0, 3.6739, 0, 0, 0.3867, 0},
    {0, 0, 2.9164, 0, 0, 1.0335}};

// === Matrices LQR para altitud ===
const float Ki_alt = 31.6228;
const float Kc_alt[2] = {28.8910, 10.5624};

// === SETUP INICIAL ===
void setup_pilote_mode()
{
    pinMode(pinLed, OUTPUT);
    Serial.begin(115200);
    Serial.println("Iniciando modo pilote...");
    InputThrottle = 1500;
    delay(100);
    Serial.println("Setup completado.");
}

void loop_pilote_mode(float dt)
{

    // Estado del sistema
    float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};

    error_phi = phi_ref - x_c[0];
    error_theta = theta_ref - x_c[1];
    error_psi = psi_ref - x_c[2];

    // Actualizar integrales (anti-windup opcional)
    integral_phi += error_phi * dt;
    integral_theta += error_theta * dt;
    integral_psi += error_psi * dt;

    // Control LQR
    tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
    tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
    tau_z = Ki_at[2][2] * integral_phi + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5];

    if (InputThrottle < 1650)
    {
        InputThrottle += 2.0;
        if (InputThrottle > 1650)
        {
            InputThrottle = 1650;
        }
    }

    if (InputThrottle > 1020)
    {
        applyControl(tau_x, tau_y, tau_z);
    }
    else
    {
        integral_phi = 0;
        integral_phi = 0;
        integral_psi = 0;
        applyControl(0, 0, 0);
        apagarMotores();
    }
}