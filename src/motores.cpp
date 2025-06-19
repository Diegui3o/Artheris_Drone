#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include "mpu.h"

void setupMotores()
{
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    mot1.attach(mot1_pin, 1000, 2000);
    mot2.attach(mot2_pin, 1000, 2000);
    mot3.attach(mot3_pin, 1000, 2000);
    mot4.attach(mot4_pin, 1000, 2000);
    delay(1000);

    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);

    mot1.attach(mot1_pin, 1000, 2000);
    delay(1000);
    mot1.setPeriodHertz(ESCfreq);
    delay(100);
    mot2.attach(mot2_pin, 1000, 2000);
    delay(1000);
    mot2.setPeriodHertz(ESCfreq);
    delay(100);
    mot3.attach(mot3_pin, 1000, 2000);
    delay(1000);
    mot3.setPeriodHertz(ESCfreq);
    delay(100);
    mot4.attach(mot4_pin, 1000, 2000);
    delay(1000);
    mot4.setPeriodHertz(ESCfreq);
    delay(100);

    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);
    Serial.println("Motores inicializados");
}

void apagarMotores()
{
    mot1.writeMicroseconds(1000); // Apagar los motores
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
}

void encenderMotores(int speed)
{
    mot1.writeMicroseconds(speed);
    mot2.writeMicroseconds(speed);
    mot3.writeMicroseconds(speed);
    mot4.writeMicroseconds(speed);
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

    mot1.writeMicroseconds(round(MotorInput1));
    mot2.writeMicroseconds(round(MotorInput2));
    mot3.writeMicroseconds(round(MotorInput3));
    mot4.writeMicroseconds(round(MotorInput4));
}
