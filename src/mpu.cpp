#include "BNO055.h"
#include <Wire.h>
#include <Arduino.h>
#include <variables.h>
#include <mpu.h>

#define A 0X28 // I2C address selection pin LOW
#define B 0x29 //                          HIGH

BNO055 mySensor(A);

void gyro_signals(void)
{
  // Euler angles
  mySensor.readEul();
  AngleYaw_est = mySensor.euler.x;
  AngleRoll = mySensor.euler.y;
  AnglePitch = mySensor.euler.z;

  // Quaternions
  // mySensor.readQuat();
  // Serial.print("Q0: ");
  // Serial.print(mySensor.quat.q0);
  // Serial.print("  Q1: ");
  // Serial.print(mySensor.quat.q1);
  // Serial.print("  Q2: ");
  // Serial.print(mySensor.quat.q2);
  // Serial.print("  Q3: ");
  // Serial.println(mySensor.quat.q3);

  // Accelerometer
  mySensor.readAccel();
  AccX = mySensor.accel.x;
  AccY = mySensor.accel.y;
  AccZ = mySensor.accel.z;

  // Gyroscope
  mySensor.readGyro();
  gyroRateRoll = mySensor.gyro.x;
  gyroRatePitch = mySensor.gyro.y;
  RateYaw = mySensor.gyro.z;

  yaw += RateYaw * dt;
  AngleYaw = 0.98 * yaw + 0.02 * (RateYaw * dt);
}

void setupMPU()
{
  Wire.begin();
  Serial.begin(115200);
  mySensor.init();
}