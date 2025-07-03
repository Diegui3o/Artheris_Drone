#include "variables.h"
#include "ICM_20948.h" // SparkFun ICM-20948 library

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1 // Dirección I2C del ICM20948: 1 por defecto, 0 si ADR está conectado a GND
#define ICM_20948_USE_DMP

ICM_20948_I2C myICM;

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

void setupMPU()
{
  SERIAL_PORT.begin(115200);
  delay(100);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000); // 400kHz I2C

  // Inicialización del sensor
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

#ifndef QUAT_ANIMATION
    SERIAL_PORT.print(F("Sensor status: "));
    SERIAL_PORT.println(myICM.statusString());
#endif

    if (myICM.status != ICM_20948_Stat_Ok)
    {
#ifndef QUAT_ANIMATION
      SERIAL_PORT.println(F("Trying again..."));
#endif
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

#ifndef QUAT_ANIMATION
  SERIAL_PORT.println(F("Device connected!"));
#endif

  // Configuración del DMP
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configurar ODR para cada sensor
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // ~5Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 10) == ICM_20948_Stat_Ok);        // ~5Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 10) == ICM_20948_Stat_Ok);         // ~5Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 10) == ICM_20948_Stat_Ok);        // ~5Hz

  // Habilitar FIFO y DMP
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled! Ready to read data."));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please ensure '#define ICM_20948_USE_DMP' is enabled in ICM_20948_C.h"));
    while (true); // Error fatal, detener ejecución
  }
}


void gyro_signals()
{
  icm_20948_DMP_data_t data;

  // Leer datos hasta vaciar FIFO
  while (myICM.readDMPdataFromFIFO(&data) == ICM_20948_Stat_FIFOMoreDataAvail ||
         myICM.status == ICM_20948_Stat_Ok)
  {
    // --- Cuaterniones (para ángulos de Euler) ---
    if (data.header & DMP_header_bitmap_Quat6)
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - (q1*q1 + q2*q2 + q3*q3)); // Calcular q0

      // Reordenar según convención deseada
      double qw = q0;
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      // Calcular ángulos de Euler
      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      AnglePitch_est = atan2(t0, t1) * 180.0 / PI;

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = constrain(t2, -1.0, 1.0);
      AngleRoll_est = asin(t2) * 180.0 / PI;

      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      // (Opcional: guardar yaw si lo necesitas)
      // AngleYaw = yaw;
    }

    // --- Aceleración cruda ---
    if (data.header & DMP_header_bitmap_Accel)
    {
      AccX = (float)data.Raw_Accel.Data.X / 4096;
      AccY = (float)data.Raw_Accel.Data.Y / 4096;
      AccZ = (float)data.Raw_Accel.Data.Z / 4096;
    }

    // --- Giroscopio crudo ---
    if (data.header & DMP_header_bitmap_Gyro)
    {
      gyroRateRoll  = (float)data.Raw_Gyro.Data.X / 65.5; // Velocidad angular en X (Roll rate)
      gyroRatePitch = (float)data.Raw_Gyro.Data.Y / 65.5; // Velocidad angular en Y (Pitch rate)
      RateYaw       = (float)data.Raw_Gyro.Data.Z / 65.5; // Velocidad angular en Z (Yaw rate)
    }

    // --- Magnetómetro (opcional) ---
    if (data.header & DMP_header_bitmap_Compass)
    {
      float magX = (float)data.Compass.Data.X;
      float magY = (float)data.Compass.Data.Y;
      float magZ = (float)data.Compass.Data.Z;

    }
    
     // Utiliza las tasas del giroscopio
    float gyroRateRoll_local = gyroRateRoll;
    float gyroRatePitch_local = gyroRatePitch;

    // Actualización del filtro de Kalman para cada eje
    AngleRoll = Kalman_filter(kalmanRoll, AngleRoll_est, gyroRateRoll_local, dt);
    AnglePitch = Kalman_filter(kalmanPitch, AnglePitch_est, gyroRatePitch_local, dt);
  }
}
