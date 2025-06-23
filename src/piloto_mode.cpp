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

// === PARÁMETROS DEL CONTROLADOR LQR ===
const float Kc_at[3][6] = {
    {3.6739, 0, 0, 0.3867, 0, 0},
    {0, 3.6739, 0, 0, 0.3867, 0},
    {0, 0, 2.9164, 0, 0, 1.0335}};

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
    // Estados: [ϕ, θ, ψ, dϕ, dθ, dψ]
    float x[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
    float x_ref[6] = {0};

    // Calcular error
    float error[6];
    for (int i = 0; i < 6; i++)
    {
        error[i] = x_ref[i] - x[i];
    }

    // Inicializar variables de control
    tau_x = 0;
    tau_y = 0;
    tau_z = 0;

    // Aplicar ganancias LQR: u = -K * error
    // Fila 0 (Roll): Kc_at[0][*]
    tau_x -= Kc_at[0][0] * error[0]; // Ángulo Roll
    tau_x -= Kc_at[0][3] * error[3]; // Velocidad Roll

    // Fila 1 (Pitch): Kc_at[1][*]
    tau_y -= Kc_at[1][1] * error[1]; // Ángulo Pitch
    tau_y -= Kc_at[1][4] * error[4]; // Velocidad Pitch

    // Fila 2 (Yaw): Kc_at[2][*]
    tau_z -= Kc_at[2][2] * error[2]; // Ángulo Yaw
    tau_z -= Kc_at[2][5] * error[5]; // Velocidad Yaw

    // Manejo gradual del throttle
    if (InputThrottle < 1650)
    {
        InputThrottle += 2.0;
        if (InputThrottle > 1650)
            InputThrottle = 1650;
    }

    // Aplicar control si el throttle es suficiente
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