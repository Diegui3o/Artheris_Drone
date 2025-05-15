#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "piloto_mode.h"
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

int yawOffset = 0;
#define SDA_MPU 21
#define SCL_MPU 22
#define SDA_TOF 4
#define SCL_TOF 5

// Función para el filtro de Kalman (roll)
double Kalman_filter(Kalman &kf, float newAngle, float newRate, float dt)
{
  // Predicción:
  double rate = newRate - kf.bias;
  kf.angle += dt * rate;

  // Actualización de la matriz de error
  kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + Q_angle);
  kf.P[0][1] -= dt * kf.P[1][1];
  kf.P[1][0] -= dt * kf.P[1][1];
  kf.P[1][1] += Q_bias * dt;

  // Medición:
  float S = kf.P[0][0] + R_measure;
  float K0 = kf.P[0][0] / S;
  float K1 = kf.P[1][0] / S;

  // Actualización con la medición (newAngle)
  float y = newAngle - kf.angle;
  kf.angle += K0 * y;
  kf.bias += K1 * y;

  // Actualizar la matriz de covarianza
  double P00_temp = kf.P[0][0];
  double P01_temp = kf.P[0][1];

  kf.P[0][0] -= K0 * P00_temp;
  kf.P[0][1] -= K0 * P01_temp;
  kf.P[1][0] -= K1 * P00_temp;
  kf.P[1][1] -= K1 * P01_temp;

  return kf.angle;
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  gyroRateRoll = GyroX / 131.0;
  gyroRatePitch = GyroY / 131.0;
  RateYaw = GyroZ / 131.0;

  AccX = (float)AccXLSB / 16384;
  AccY = (float)AccYLSB / 16384;
  AccZ = (float)AccZLSB / 16384;

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  AngleRoll_est = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch_est = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);

  // Cálculo del ángulo estimado a partir del acelerómetro (usando atan2 puede ser más robusto)
  accAngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;
  accAnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  // Utiliza las tasas del giroscopio
  float gyroRateRoll_local = gyroRateRoll;
  float gyroRatePitch_local = gyroRatePitch;

  // Actualización del filtro de Kalman para cada eje
  AngleRoll = Kalman_filter(kalmanRoll, accAngleRoll, gyroRateRoll_local, dt);
  AnglePitch = Kalman_filter(kalmanPitch, accAnglePitch, gyroRatePitch_local, dt);
}

void loop_yaw()
{
  compass.read();
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();

  // Check if the compass is responding
  if (x == 0 && y == 0 && z == 0)
  {
    Serial.println("Compass not responding. Reinitializing...");
    compass.init();
    compass.setMode(0x01, 0x0C, 0x10, 0x00); // Continuous mode, 10Hz, 8G range
    return;                                  // Skip the rest of the loop to allow reinitialization
  }

  int heading = compass.getAzimuth();
  int yaw = heading - yawOffset;

  if (yaw > 180)
    yaw -= 360;
  if (yaw < -180)
    yaw += 360;

  AngleYaw = yaw;
}

void setupMPU()
{
  Serial.begin(115200);
  Wire.begin();

  // Initialize the compass
  compass.init();
  Serial.println("Compass initialized.");
  compass.setMode(0x01, 0x0C, 0x10, 0x00); // Continuous mode, 10Hz, 8G range

  // Initialize the MPU6050
  accelgyro.initialize();
  if (!accelgyro.testConnection())
  {
    Serial.println("Error: No se pudo conectar con el MPU6050.");
    while (true)
    {
      delay(1000);
    }
  }
  Serial.println("MPU6050 conectado correctamente.");

  delay(2000);
  compass.read();
  delay(20);
  yawOffset = compass.getAzimuth();
  Serial.print("Calibrado. Dirección inicial = ");
  Serial.print(yawOffset);
  Serial.println("° (ahora es yaw = 0°)");
}