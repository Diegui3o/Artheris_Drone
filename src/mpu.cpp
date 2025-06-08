#include <Arduino.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <MPU6050.h>
#include "variables.h"
#include "mpu.h"

QMC5883LCompass compass;
MPU6050 mpu;

int yawOffset = 0;
const int YAW_FILTER_SIZE = 5;
float yawHistory[YAW_FILTER_SIZE] = {0};
int yawIndex = 0;
const float MAG_THRESHOLD = 1500.0; // Ajustar según pruebas

// Variable para umbral de compensación configurable
static float compensationThreshold = 2.0; // Grados mínimos para activar compensación

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

  AngleRoll_est = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch_est = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);

  // Cálculo del ángulo estimado a partir del acelerómetro (usando atan2 puede ser más robusto)
  accAngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;
  accAnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  // Utiliza las tasas del giroscopio
  float gyroRateRoll_local = gyroRateRoll;
  float gyroRatePitch_local = gyroRatePitch;

  // Actualización del filtro de Kalman para cada eje
  AngleRoll = Kalman_filter(kalmanRoll, AngleRoll_est, gyroRateRoll_local, dt);
  AnglePitch = Kalman_filter(kalmanPitch, AnglePitch_est, gyroRatePitch_local, dt);

  // Integración de la tasa de Yaw para obtener el ángulo (¡puede haber drift!)
  yaw += RateYaw * dt;
  yaw = 0.98 * yaw + 0.02 * (RateYaw * dt);
  AngleYaw = Kalman_filter(kalmanYaw, yaw, RateYaw, dt);
}

void loop_yaw()
{
  compass.read();
  float heading = compass.getAzimuth() * PI / 180.0; // Ángulo Yaw absoluto del magnetómetro (convertido a radianes)

  // Normalizar el heading entre 0 y 2*PI
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Fusión sensor con filtro complementario - más peso al magnetómetro para corregir drift
  // AngleYaw = 0.50 * (yaw + RateYaw * dt) + 0.50 * heading;
}

void setupMPU()
{
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22

  // Inicializar magnetómetro
  compass.init();
  Serial.println("Inicializando magnetómetro...");

  // Verificar si el magnetómetro responde
  compass.read();

  mpu.initialize();
  // Calibración automática al inicio
  compass.setCalibration(-1767, 1345, -1503, 1199, -1325, 1567);
  delay(2000); // Esperar estabilización

  calibrateSensors();
  Serial.println("Calibración completada.");
}

// === CALIBRACIÓN DEL MPU6050 ===
void calibrateSensors()
{
  Serial.println("\nCalibrando sensores...");

  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  meansensors();
  Serial.println("\nCalculando offsets...");
  calibration();

  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);

  Serial.println("Calibración completada.");
}

void meansensors()
{
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101))
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100))
    {
      buff_ax += ax;
      buff_ay += ay;
      buff_az += az;
      buff_gx += gx;
      buff_gy += gy;
      buff_gz += gz;
    }
    i++;
    delay(2);
  }

  mean_ax = buff_ax / buffersize;
  mean_ay = buff_ay / buffersize;
  mean_az = buff_az / buffersize;
  mean_gx = buff_gx / buffersize;
  mean_gy = buff_gy / buffersize;
  mean_gz = buff_gz / buffersize;
}

void calibration()
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (1)
  {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();

    if (abs(mean_ax) <= acel_deadzone)
      ready++;
    else
      ax_offset -= mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone)
      ready++;
    else
      ay_offset -= mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone)
      ready++;
    else
      az_offset += (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone)
      ready++;
    else
      gx_offset -= mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone)
      ready++;
    else
      gy_offset -= mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone)
      ready++;
    else
      gz_offset -= mean_gz / (giro_deadzone + 1);

    if (ready == 6)
      break;
  }
}