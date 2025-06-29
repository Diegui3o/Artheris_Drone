#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony_DPEng.h>
#include <Madgwick_DPEng.h>
#include <DPEng_ICM20948_AK09916.h>
#include "variables.h"

// Create sensor instance.
DPEng_ICM20948 dpEng = DPEng_ICM20948(0x948A, 0x948B, 0x948C);

// Offsets applied to raw x/y/z mag values
float mag_offsets[3] = {-1.76F, 22.54F, 4.43F};

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = {{0.954, -0.019, 0.003},
                                   {-0.019, 1.059, -0.009},
                                   {0.003, -0.009, 0.990}};

float mag_field_strength = 29.85F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3] = {0.0F, 0.0F, 0.0F};

void setupMPU()
{
  Serial.begin(115200);

  Serial.println(F("DPEng AHRS Fusion Example"));
  Serial.println("");

  // Initialize the sensors.
  if (!dpEng.begin(ICM20948_ACCELRANGE_4G, GYRO_RANGE_250DPS, ICM20948_ACCELLOWPASS_50_4_HZ))
  {
    /* There was a problem detecting the ICM20948 ... check your connections */
    Serial.println("Ooops, no ICM20948/AK09916 detected ... Check your wiring!");
    while (1)
      ;
  }

  filter.begin();
}

void gyro_signals(void)
{
  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;

  // Get new data samples
  dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                my, mx, mz);

  gyroRateRoll = gx;
  gyroRatePitch = gy;
  RateYaw = gz;

  AngleRoll = filter.getRoll();
  AnglePitch = filter.getPitch();
  float heading = filter.getYaw();
}