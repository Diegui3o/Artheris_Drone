#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int64_t t_us;

    float acc_x_g;
    float acc_y_g;
    float acc_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;

    float roll_acc_deg;
    float pitch_acc_deg;

    float yaw_deg;
} ImuSample;

// API principal
bool imu_init(int i2c_port, int sda_gpio, int scl_gpio, uint32_t clk_hz);
bool imu_start_1khz(int core, int priority);
bool imu_get_latest(ImuSample *out);

// 👉 NUEVAS FUNCIONES PARA CALIBRACIÓN (las que usa imu_calib.c)
bool imu_read_raw_lsb(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz);

void imu_set_offsets_lsb(int16_t ax_off, int16_t ay_off, int16_t az_off,
                         int16_t gx_off, int16_t gy_off, int16_t gz_off);