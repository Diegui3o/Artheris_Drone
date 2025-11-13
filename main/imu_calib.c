#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include "led.h"

// parámetros de tu sketch original
static const int buffersize = 1000; // nº de muestras para promedio
static const int acel_deadzone = 8; // zona muerta acelerómetro (LSB)
static const int giro_deadzone = 1; // zona muerta giroscopio (LSB)

// para ±8g -> 4096 LSB ≈ 1 g
static const int ACC_1G_LSB = 4096;

// Colores
#define LED_CYAN_R 0
#define LED_CYAN_G 255
#define LED_CYAN_B 255

#define LED_ORANGE_R 255
#define LED_ORANGE_G 128
#define LED_ORANGE_B 0

#define LED_OFF_R 0
#define LED_OFF_G 0
#define LED_OFF_B 0

// Umbrales de movimiento (puedes afinarlos)
static const int16_t ACC_MOVE_THRESH = 300;  // LSB ~0.07g
static const int16_t GYRO_MOVE_THRESH = 200; // LSB (~3 dps aprox)

// Pequeña ayuda para parpadeo anaranjado
static void blink_orange_once(TickType_t period_ticks)
{
     static bool on = false;
     if (on)
          rgb_led_set(LED_OFF_R, LED_OFF_G, LED_OFF_B);
     else
          rgb_led_set(LED_ORANGE_R, LED_ORANGE_G, LED_ORANGE_B);
     on = !on;
     vTaskDelay(period_ticks);
}

// Espera 2 minutos parpadeando naranja (para movimiento)
static void wait_two_minutes_blinking_orange(void)
{
     const TickType_t total = pdMS_TO_TICKS(60000);
     const TickType_t step = pdMS_TO_TICKS(200); // 200 ms parpadeo
     TickType_t start = xTaskGetTickCount();

     while ((xTaskGetTickCount() - start) < total)
     {
          blink_orange_once(step);
     }
}

// calcula medias crudas (sin offsets) sobre 'buffersize' muestras
// y detecta si hubo movimiento significativo
static bool imu_meansensors(int32_t *mean_ax, int32_t *mean_ay, int32_t *mean_az,
                            int32_t *mean_gx, int32_t *mean_gy, int32_t *mean_gz,
                            bool *moved_out)
{
     int64_t buff_ax = 0, buff_ay = 0, buff_az = 0;
     int64_t buff_gx = 0, buff_gy = 0, buff_gz = 0;

     int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
     int16_t last_ax = 0, last_ay = 0, last_az = 0;
     int16_t last_gx = 0, last_gy = 0, last_gz = 0;
     bool first = true;
     bool moved = false;

     // igual que tu while (i < buffersize + 101), descartando primeras 100
     for (int i = 0; i < buffersize + 101; i++)
     {
          if (!imu_read_raw_lsb(&ax, &ay, &az, &gx, &gy, &gz))
          {
               // si falla I2C, abortamos
               return false;
          }

          // Detección de movimiento entre muestras
          if (!first)
          {
               if (abs(ax - last_ax) > ACC_MOVE_THRESH ||
                   abs(ay - last_ay) > ACC_MOVE_THRESH ||
                   abs(az - last_az) > ACC_MOVE_THRESH ||
                   abs(gx - last_gx) > GYRO_MOVE_THRESH ||
                   abs(gy - last_gy) > GYRO_MOVE_THRESH ||
                   abs(gz - last_gz) > GYRO_MOVE_THRESH)
               {
                    moved = true;
               }
          }
          first = false;
          last_ax = ax;
          last_ay = ay;
          last_az = az;
          last_gx = gx;
          last_gy = gy;
          last_gz = gz;

          if (i > 100)
          {
               buff_ax += ax;
               buff_ay += ay;
               buff_az += az;
               buff_gx += gx;
               buff_gy += gy;
               buff_gz += gz;
          }

          vTaskDelay(pdMS_TO_TICKS(2)); // equivalente a delay(2)
     }

     *mean_ax = buff_ax / buffersize;
     *mean_ay = buff_ay / buffersize;
     *mean_az = buff_az / buffersize;
     *mean_gx = buff_gx / buffersize;
     *mean_gy = buff_gy / buffersize;
     *mean_gz = buff_gz / buffersize;

     if (moved_out)
          *moved_out = moved;

     return true;
}

// Calibración tipo Arduino, pero en offsets de software (LSB)
bool imu_calibrate_blocking(void)
{
     while (1) // bucle general: si se mueve o hay lío, reintentamos desde cero
     {
          // Inicio de calibración desde cero -> CELESTE
          rgb_led_set(LED_CYAN_R, LED_CYAN_G, LED_CYAN_B);

          int32_t mean_ax, mean_ay, mean_az;
          int32_t mean_gx, mean_gy, mean_gz;
          bool moved = false;

          // 1) medias iniciales sin offset
          if (!imu_meansensors(&mean_ax, &mean_ay, &mean_az,
                               &mean_gx, &mean_gy, &mean_gz,
                               &moved))
          {
               // Error I2C -> parpadeo naranja hasta que se recupere
               while (!imu_meansensors(&mean_ax, &mean_ay, &mean_az,
                                       &mean_gx, &mean_gy, &mean_gz,
                                       &moved))
               {
                    blink_orange_once(pdMS_TO_TICKS(200));
               }
          }

          // Si aquí detectamos movimiento durante esta fase -> esperar 2 min y reiniciar TODO
          if (moved)
          {
               wait_two_minutes_blinking_orange();
               // seguimos el while(1) exterior -> vuelve a empezar (celeste y todo de 0)
               continue;
          }

          // 2) offsets iniciales (similar espíritu a tu calibration())
          int32_t ax_off = -mean_ax;
          int32_t ay_off = -mean_ay;
          int32_t az_off = (ACC_1G_LSB - mean_az);

          int32_t gx_off = -mean_gx;
          int32_t gy_off = -mean_gy;
          int32_t gz_off = -mean_gz;

          while (1)
          {
               int ready = 0;
               moved = false;

               // recalculamos medias con los offsets actuales (simulados en SW)
               if (!imu_meansensors(&mean_ax, &mean_ay, &mean_az,
                                    &mean_gx, &mean_gy, &mean_gz,
                                    &moved))
               {
                    // Error I2C -> parpadeo naranja hasta que vuelva a funcionar
                    while (!imu_meansensors(&mean_ax, &mean_ay, &mean_az,
                                            &mean_gx, &mean_gy, &mean_gz,
                                            &moved))
                    {
                         blink_orange_once(pdMS_TO_TICKS(200));
                    }
               }

               // Si se movió mientras estábamos ajustando -> esperar 2 min y reiniciar todo
               if (moved)
               {
                    wait_two_minutes_blinking_orange();
                    // Rompemos este while interno y dejamos que el while(1) externo reinicie
                    goto restart_all;
               }

               // aplicamos offsets a las medias (simulado)
               int32_t mean_ax_corr = mean_ax + ax_off;
               int32_t mean_ay_corr = mean_ay + ay_off;
               int32_t mean_az_corr = mean_az + az_off;

               int32_t mean_gx_corr = mean_gx + gx_off;
               int32_t mean_gy_corr = mean_gy + gy_off;
               int32_t mean_gz_corr = mean_gz + gz_off;

               // acelerómetro X
               if (abs(mean_ax_corr) <= acel_deadzone)
                    ready++;
               else
                    ax_off -= mean_ax_corr / acel_deadzone;

               // acelerómetro Y
               if (abs(mean_ay_corr) <= acel_deadzone)
                    ready++;
               else
                    ay_off -= mean_ay_corr / acel_deadzone;

               // acelerómetro Z -> queremos que esté cerca de +1 g
               int32_t err_az = (mean_az_corr - ACC_1G_LSB);
               if (abs(err_az) <= acel_deadzone)
                    ready++;
               else
                    az_off -= err_az / acel_deadzone;

               // gyro X
               if (abs(mean_gx_corr) <= giro_deadzone)
                    ready++;
               else
                    gx_off -= mean_gx_corr / (giro_deadzone + 1);

               // gyro Y
               if (abs(mean_gy_corr) <= giro_deadzone)
                    ready++;
               else
                    gy_off -= mean_gy_corr / (giro_deadzone + 1);

               // gyro Z
               if (abs(mean_gz_corr) <= giro_deadzone)
                    ready++;
               else
                    gz_off -= mean_gz_corr / (giro_deadzone + 1);

               if (ready == 6)
                    break;
          }

          // 3) aplicar offsets finales en la IMU (software)
          imu_set_offsets_lsb((int16_t)ax_off,
                              (int16_t)ay_off,
                              (int16_t)az_off,
                              (int16_t)gx_off,
                              (int16_t)gy_off,
                              (int16_t)gz_off);

          // Aquí podrías poner otro color (verde) para "calibración OK"
          // rgb_led_set(0, 255, 0);

          return true;

     restart_all:
          // volvemos al while(1) externo, que arranca de cero (celeste)
          continue;
     }
}
