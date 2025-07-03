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
#define TORQUE_SCALE 50.0f

// Variable to track MPU calibration status
bool mpu_ready = false;

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
    // Inicializar throttle solo una vez al principio
    static bool throttle_initialized = false;
    if (!throttle_initialized)
    {
        InputThrottle = 1000; // Empezar en 1000
        throttle_initialized = true;
    }
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
        phi_ref = (DesiredAngleRoll / 2.5) * DEG_TO_RAD;
        theta_ref = (DesiredAnglePitch / 2.5) * DEG_TO_RAD;
        psi_ref = (DesiredAngleYaw / 2.5) * DEG_TO_RAD;

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
        error_psi = psi_ref - yaw_rad;

        // 4. CUARTO: Actualizar integrales con saturación específica por eje
        integral_phi = constrain(integral_phi + error_phi * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
        integral_theta = constrain(integral_theta + error_theta * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
        integral_psi = constrain(integral_psi + error_psi * dt, -MAX_INTEGRAL_YAW, MAX_INTEGRAL_YAW);

        // 5. QUINTO: Control LQR usando las integrales actualizadas
        tau_x = Ki_at[0][0] * integral_phi - Kc_at[0][0] * roll_rad - Kc_at[0][3] * x_c[3];
        tau_y = Ki_at[1][1] * integral_theta - Kc_at[1][1] * pitch_rad - Kc_at[1][4] * x_c[4];
        tau_z = Ki_at[2][2] * integral_psi - Kc_at[2][2] * AngleYaw - Kc_at[2][5] * RateYaw;

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