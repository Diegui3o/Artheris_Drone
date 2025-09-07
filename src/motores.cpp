
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <piloto_mode.h>
#include "variables.h"
#include "mpu.h"

// Pines y canales PWM para los motores
#define MOT1_PIN 15
#define MOT2_PIN 12
#define MOT3_PIN 14
#define MOT4_PIN 27

#define MOT1_CH 0
#define MOT2_CH 1
#define MOT3_CH 2
#define MOT4_CH 3

#define PWM_FREQ 50      // 50Hz para ESC estándar
#define PWM_RES 16       // 16 bits de resolución

void writeMotorMicroseconds(uint8_t channel, uint16_t us) {
    // Para 50Hz y 16 bits: 1 ciclo = 20,000us, 2^16 = 65536
    // Duty = (us / 20000) * 65535
    uint32_t duty = (uint32_t)us * 65535 / 20000;
    ledcWrite(channel, duty);
}

// =====================
// Calibración de ESC
// =====================
// ¡ATENCIÓN! Realiza este procedimiento SIN HÉLICES por seguridad.
// Llama a esta función SOLO cuando quieras calibrar los ESC.
void calibrarESC() {
    // 1. Señal máxima (2000us) durante 4 segundos
    writeMotorMicroseconds(MOT1_CH, 2000);
    writeMotorMicroseconds(MOT2_CH, 2000);
    writeMotorMicroseconds(MOT3_CH, 2000);
    writeMotorMicroseconds(MOT4_CH, 2000);
    delay(4000); // Espera tonos de calibración

    // 2. Señal mínima (1000us) durante 4 segundos
    writeMotorMicroseconds(MOT1_CH, 1000);
    writeMotorMicroseconds(MOT2_CH, 1000);
    writeMotorMicroseconds(MOT3_CH, 1000);
    writeMotorMicroseconds(MOT4_CH, 1000);
    delay(4000); // Espera tonos de confirmación

    // 3. Listo, los ESC están calibrados
}


void setupMotores() {
    // =======
    // Si necesitas calibrar los ESC, descomenta la siguiente línea y sube el código:
    // calibrarESC();
    // Luego de calibrar, vuelve a comentar esta línea y sube el firmware normal.
    // =======
    ledcSetup(MOT1_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT1_PIN, MOT1_CH);
    ledcSetup(MOT2_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT2_PIN, MOT2_CH);
    ledcSetup(MOT3_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT3_PIN, MOT3_CH);
    ledcSetup(MOT4_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT4_PIN, MOT4_CH);

    writeMotorMicroseconds(MOT1_CH, 1000);
    writeMotorMicroseconds(MOT2_CH, 1000);
    writeMotorMicroseconds(MOT3_CH, 1000);
    writeMotorMicroseconds(MOT4_CH, 1000);
}

void apagarMotores()
{
    writeMotorMicroseconds(MOT1_CH, 1000);
    writeMotorMicroseconds(MOT2_CH, 1000);
    writeMotorMicroseconds(MOT3_CH, 1000);
    writeMotorMicroseconds(MOT4_CH, 1000);
}

void encenderMotores(int speed)
{
    writeMotorMicroseconds(MOT1_CH, speed);
    writeMotorMicroseconds(MOT2_CH, speed);
    writeMotorMicroseconds(MOT3_CH, speed);
    writeMotorMicroseconds(MOT4_CH, speed);
}

// === CONTROL A LOS MOTORES CON SATURACION COLECTIVA ===
void applyControl(float tau_x, float tau_y, float tau_z)
{
    float f[4];
    f[0] = InputThrottle - tau_x - tau_y - tau_z; // pwm1
    f[1] = InputThrottle - tau_x + tau_y + tau_z; // pwm2
    f[2] = InputThrottle + tau_x + tau_y - tau_z; // pwm3
    f[3] = InputThrottle + tau_x - tau_y + tau_z; // pwm4

    const float f_min = 1000.0;
    const float f_max = 1990.0;
    bool saturado = false;

    for (int i = 0; i < 4; i++)
    {
        if (f[i] < f_min || f[i] > f_max)
        {
            saturado = true;
            break;
        }
    }

    if (saturado)
    {
        float max_violation = 0;
        int j = 0;
        for (int i = 0; i < 4; i++)
        {
            float violation = 0;
            if (f[i] > f_max)
                violation = f[i] - f_max;
            else if (f[i] < f_min)
                violation = f_min - f[i];

            if (violation > max_violation)
            {
                max_violation = violation;
                j = i;
            }
        }

        float gamma = 1.0;
        if (f[j] > f_max)
            gamma = f_max / f[j];
        else if (f[j] < f_min)
            gamma = f_min / f[j];

        for (int i = 0; i < 4; i++)
        {
            f[i] *= gamma;
        }
    }

    // Recorte final por seguridad
    MotorInput1 = constrain(f[0], f_min, f_max);
    MotorInput2 = constrain(f[1], f_min, f_max);
    MotorInput3 = constrain(f[2], f_min, f_max);
    MotorInput4 = constrain(f[3], f_min, f_max);

    writeMotorMicroseconds(MOT1_CH, round(MotorInput1));
    writeMotorMicroseconds(MOT2_CH, round(MotorInput2));
    writeMotorMicroseconds(MOT3_CH, round(MotorInput3));
    writeMotorMicroseconds(MOT4_CH, round(MotorInput4));
}