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
    // Inicializar throttle solo una vez al principio
    static bool throttle_initialized = false;
    if (!throttle_initialized)
    {
        InputThrottle = 1000; // Empezar en 1000
        throttle_initialized = true;
    }

    // Estado del sistema
    float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
    float x_i[3] = {integral_phi, integral_theta, integral_psi}; // Calcular errores ANTES del control LQR
    error_phi = phi_ref - x_c[0];
    error_theta = theta_ref - x_c[1];
    error_psi = psi_ref - x_c[2];

    // === 1. Feedforward (basado en cambios de referencia) ===
    ff_phi = 0.1 * (phi_ref - prev_phi_ref) / dt; // Derivada de la referencia
    ff_theta = 0.1 * (theta_ref - prev_theta_ref) / dt;
    ff_psi = 0.0; // Opcional para yaw

    // Control LQR para generar tau
    tau_x = Ki_at[0][0] * x_i[0] + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
    tau_y = Ki_at[1][1] * x_i[1] + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
    tau_z = Ki_at[2][2] * x_i[2] + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5];

    // Actualizar integrales
    x_i[0] += error_phi * dt;
    x_i[1] += error_theta * dt;
    x_i[2] += error_psi * dt;

    tau_x -= Ki_at[0][0] * x_i[0];
    tau_y -= Ki_at[1][1] * x_i[1];
    tau_z -= Ki_at[2][2] * x_i[2];

    // === 2. Modo deslizante para roll y pitch (robustez) ===
    S_phi = (gyroRateRoll) + lambda_sliding * error_phi; // Superficie deslizante
    S_theta = (gyroRatePitch) + lambda_sliding * error_theta;

    // Término de control deslizante (signo suavizado para evitar chattering)
    float sliding_term_phi = 0.5 * sat(S_phi, 0.1); // Función de saturación
    float sliding_term_theta = 0.5 * sat(S_theta, 0.1);

    // === 3. Combinar todos los términos de control ===
    tau_x += ff_phi + sliding_term_phi;     // LQR + Feedforward + Sliding Mode
    tau_y += ff_theta + sliding_term_theta; // LQR + Feedforward + Sliding Mode
    tau_z += ff_psi;                        // LQR + Feedforward

    // Guardar referencia actual para el próximo ciclo (feedforward)
    prev_phi_ref = phi_ref;
    prev_theta_ref = theta_ref;

    // Incrementar throttle gradualmente de 1000 a 1850
    if (InputThrottle < 1850)
    {
        InputThrottle += 3.0; // Incremento de 3 unidades por ciclo
        if (InputThrottle > 1850)
        {
            InputThrottle = 1850; // Limitar a máximo 1850
        }
    }

    // Aplicar control cuando el throttle esté por encima del mínimo de seguridad
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