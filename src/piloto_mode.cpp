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
const float Ki_at[3][3] = {
    {0.1, 0, 0},
    {0, 0.1, 0},
    {0, 0, 0.01}};

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
    // Inicializar throttle solo una vez al principio
    static bool throttle_initialized = false;
    if (!throttle_initialized)
    {
        InputThrottle = 1000; // Empezar en 1000
        throttle_initialized = true;
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
    error_phi = roll_rad - phi_ref;
    error_theta = pitch_rad - theta_ref;
    error_psi = yaw_rad - psi_ref;

    // 4. CUARTO: Actualizar integrales con saturación específica por eje
    integral_phi = constrain(integral_phi + error_phi * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
    integral_theta = constrain(integral_theta + error_theta * dt, -MAX_INTEGRAL_ROLL_PITCH, MAX_INTEGRAL_ROLL_PITCH);
    integral_psi = constrain(integral_psi + error_psi * dt, -MAX_INTEGRAL_YAW, MAX_INTEGRAL_YAW);

    // 5. QUINTO: Control LQR usando las integrales actualizadas
    tau_x = -Ki_at[0][0] * integral_phi - Kc_at[0][0] * roll_rad - Kc_at[0][3] * x_c[3];
    tau_y = -Ki_at[1][1] * integral_theta - Kc_at[1][1] * pitch_rad - Kc_at[1][4] * x_c[4];
    tau_z = -Ki_at[2][2] * integral_psi - Kc_at[2][2] * yaw_rad - Kc_at[2][5] * x_c[5];

    // 5. Escalar torques a PWM (ejemplo: 500 μs/N·m)
    tau_x *= TORQUE_SCALE;
    tau_y *= TORQUE_SCALE;
    tau_z *= TORQUE_SCALE;

    // Incrementar throttle gradualmente de 1000 a 1850
    if (InputThrottle < 1850)
    {
        InputThrottle += 3.0; // Incremento de 3 unidades por ciclo
        if (InputThrottle > 1550)
        {
            InputThrottle = 1550; // Limitar a máximo 1850
        }
    } // Aplicar control cuando el throttle esté por encima del mínimo de seguridad
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