#ifndef MPU_H
#define MPU_H

void kalmanUpdateRoll(float accAngleRoll, float gyroRateRoll);
void kalmanUpdatePitch(float accAnglePitch, float gyroRatePitch);
void mpu_signals();
void setupMPU();
void loop_yaw();
void gyro_signals();

#endif