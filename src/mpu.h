#ifndef MPU_H
#define MPU_H

void kalmanUpdateRoll(float accAngleRoll, float gyroRateRoll);
void kalmanUpdatePitch(float accAnglePitch, float gyroRatePitch);
void setupMPU();
void loop_yaw();
void gyro_signals();
void calibrateSensors();
void meansensors();
void calibration();

#endif