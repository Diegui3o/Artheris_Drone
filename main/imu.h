#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int64_t t_us;

    // Acelerómetro (g)
    float acc_x_g, acc_y_g, acc_z_g;

    // Giroscopio (°/s) - igual a tu Arduino
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;

    // Giroscopio (rad/s) - opcional por si luego lo necesitas
    float gyro_x_rads, gyro_y_rads, gyro_z_rads;

    // Ángulos por acelerómetro (deg) - como en tu código
    float roll_acc_deg, pitch_acc_deg;
} ImuSample;

// === Setup / control ===
bool imu_init(int i2c_port, int sda_gpio, int scl_gpio, uint32_t clk_hz);
bool imu_start_1khz(int core, int priority);

// === Offsets (mismos que tu Arduino, aplicados a LSB crudos) ===
void imu_set_acc_offsets(int16_t ax_lsboff, int16_t ay_lsboff, int16_t az_lsboff);
void imu_set_gyro_offsets(int16_t gx_lsboff, int16_t gy_lsboff, int16_t gz_lsboff);

// === Lectura lock-free para ambos núcleos ===
bool imu_get_latest(ImuSample *out);