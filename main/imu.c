#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include <math.h>
#include <string.h>
#include "led.h"

// ===== Dispositivo =====
static uint8_t s_addr = 0x68;
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

// ACCEL_CONFIG = 0x10 -> ±8 g  → 4096 LSB/g
// GYRO_CONFIG  = 0x08 -> ±500°/s → 65.5 LSB/(°/s)
#define ACC_LSB_PER_G 4096.0f
#define GYRO_LSB_PER_DPS 65.5f
#define DEG2RAD 0.017453292519943295f

// ===== I2C =====
static int s_i2c_port = I2C_NUM_0;

static volatile int16_t s_ax_off_lsb = 0;
static volatile int16_t s_ay_off_lsb = 0;
static volatile int16_t s_az_off_lsb = 0;
static volatile int16_t s_gx_off_lsb = 0;
static volatile int16_t s_gy_off_lsb = 0;
static volatile int16_t s_gz_off_lsb = 0;

typedef struct
{
    volatile uint32_t seq;
    ImuSample sample;
} SharedImu;

static SharedImu s_shared = {0};

// ===== Timer y tarea productora =====
static TaskHandle_t s_imu_task = NULL;
static esp_timer_handle_t s_tmr = NULL;

static inline void seqlock_write_begin(volatile uint32_t *seq) { (*seq)++; }
static inline void seqlock_write_end(volatile uint32_t *seq) { (*seq)++; }

static inline bool seqlock_read(ImuSample *out)
{
    uint32_t s1 = s_shared.seq;
    if (s1 & 1)
        return false;
    ImuSample tmp = s_shared.sample;
    uint32_t s2 = s_shared.seq;
    if (s1 != s2 || (s2 & 1))
        return false;
    *out = tmp;
    return true;
}

bool imu_get_latest(ImuSample *out)
{
    for (int i = 0; i < 5; i++)
        if (seqlock_read(out))
            return true;
    return false;
}

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    // timeout 50 ms para inicio
    return i2c_master_write_to_device(s_i2c_port, s_addr, data, 2, pdMS_TO_TICKS(50));
}

static esp_err_t i2c_read_multi(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(s_i2c_port, s_addr,
                                        &reg, 1, buf, len,
                                        pdMS_TO_TICKS(50));
}

static esp_err_t i2c_read_u8(uint8_t reg, uint8_t *val)
{
    return i2c_read_multi(reg, val, 1);
}
void imu_set_offsets_lsb(int16_t ax_off, int16_t ay_off, int16_t az_off,
                         int16_t gx_off, int16_t gy_off, int16_t gz_off)
{
    s_ax_off_lsb = ax_off;
    s_ay_off_lsb = ay_off;
    s_az_off_lsb = az_off;
    s_gx_off_lsb = gx_off;
    s_gy_off_lsb = gy_off;
    s_gz_off_lsb = gz_off;
}

// Lectura cruda en LSB, sin aplicar offsets ni escalas
bool imu_read_raw_lsb(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t acc_raw[6];
    uint8_t gyr_raw[6];

    if (i2c_read_multi(REG_ACCEL_XOUT_H, acc_raw, sizeof(acc_raw)) != ESP_OK)
        return false;

    if (i2c_read_multi(REG_GYRO_XOUT_H, gyr_raw, sizeof(gyr_raw)) != ESP_OK)
        return false;

    *ax = (int16_t)((acc_raw[0] << 8) | acc_raw[1]);
    *ay = (int16_t)((acc_raw[2] << 8) | acc_raw[3]);
    *az = (int16_t)((acc_raw[4] << 8) | acc_raw[5]);

    *gx = (int16_t)((gyr_raw[0] << 8) | gyr_raw[1]);
    *gy = (int16_t)((gyr_raw[2] << 8) | gyr_raw[3]);
    *gz = (int16_t)((gyr_raw[4] << 8) | gyr_raw[5]);

    return true;
}

// --- init con la MISMA configuración que tu Arduino ---
bool imu_init(int i2c_port, int sda_gpio, int scl_gpio, uint32_t clk_hz)
{
    s_i2c_port = i2c_port;

    // 1) Arranca a 100 kHz para mayor tolerancia
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // mejor si además tienes pull-ups externos 4.7k–10k
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0};
    ESP_ERROR_CHECK(i2c_param_config(s_i2c_port, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(s_i2c_port, I2C_MODE_MASTER, 0, 0, 0));
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2) Probar 0x68 y 0x69 leyendo WHO_AM_I
    uint8_t who = 0;
    bool found = false;
    for (int k = 0; k < 2 && !found; ++k)
    {
        s_addr = (k == 0) ? 0x68 : 0x69;
        if (i2c_read_u8(REG_WHO_AM_I, &who) == ESP_OK)
        {
            found = true;
        }
    }
    if (!found)
    {
        // ESP_LOGE("IMU", "No responde 0x68 ni 0x69");
        rgb_led_set(255, 0, 0);
        return false;
    }

    // Reset y wake
    (void)i2c_write_reg(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t err = ESP_FAIL;
    for (int tries = 0; tries < 3; ++tries)
    {
        err = i2c_write_reg(REG_PWR_MGMT_1, 0x00);
        if (err == ESP_OK)
            break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (err != ESP_OK)
    {
        // ESP_LOGE("IMU", "No pude escribir PWR_MGMT_1 (wake). err=0x%x", err);
        rgb_led_set(255, 0, 0);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // 4) Config igual que tu sketch
    ESP_ERROR_CHECK(i2c_write_reg(REG_CONFIG, 0x05));       // DLPF = 5
    ESP_ERROR_CHECK(i2c_write_reg(REG_ACCEL_CONFIG, 0x10)); // ±8 g
    ESP_ERROR_CHECK(i2c_write_reg(REG_GYRO_CONFIG, 0x08));  // ±500 °/s
    ESP_ERROR_CHECK(i2c_write_reg(REG_SMPLRT_DIV, 0x00));   // 1 kHz

    // 5) (Opcional) Subir clock I2C al solicitado (p.ej., 400 kHz) después de que ya responde
    if (clk_hz != 100000)
    {
        cfg.master.clk_speed = clk_hz;
        ESP_ERROR_CHECK(i2c_param_config(s_i2c_port, &cfg));
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return true;
}

// --- timer 1 kHz -> notifica a la tarea ---
static void IRAM_ATTR imu_timer_cb(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &hp);
    if (hp)
        portYIELD_FROM_ISR();
}

// --- tarea productora: MISMA lectura y MISMAS conversiones que tu Arduino ---
static void imu_task(void *arg)
{
    s_imu_task = xTaskGetCurrentTaskHandle();

    const esp_timer_create_args_t tcfg = {
        .callback = imu_timer_cb,
        .arg = s_imu_task,
        .dispatch_method = ESP_TIMER_TASK, // si activas ISR en menuconfig, cámbialo a ESP_TIMER_ISR
        .name = "imu1k"};
    esp_timer_create(&tcfg, &s_tmr);
    esp_timer_start_periodic(s_tmr, 1000); // 1000 us = 1 kHz

    uint8_t acc_raw[6];
    uint8_t gyr_raw[6];

    for (;;)
    {
        // Espera tick 1 ms
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // === ACC (X,Y,Z)
        if (i2c_read_multi(REG_ACCEL_XOUT_H, acc_raw, sizeof(acc_raw)) != ESP_OK)
        {
            rgb_led_set(255, 0, 0);
            continue;
        }
        int16_t AccXLSB = (int16_t)((acc_raw[0] << 8) | acc_raw[1]);
        int16_t AccYLSB = (int16_t)((acc_raw[2] << 8) | acc_raw[3]);
        int16_t AccZLSB = (int16_t)((acc_raw[4] << 8) | acc_raw[5]);

        // === GYRO (X,Y,Z)
        if (i2c_read_multi(REG_GYRO_XOUT_H, gyr_raw, sizeof(gyr_raw)) != ESP_OK)
        {
            rgb_led_set(255, 0, 0);
            continue;
        }
        int16_t GyroX = (int16_t)((gyr_raw[0] << 8) | gyr_raw[1]);
        int16_t GyroY = (int16_t)((gyr_raw[2] << 8) | gyr_raw[3]);
        int16_t GyroZ = (int16_t)((gyr_raw[4] << 8) | gyr_raw[5]);

        AccXLSB += s_ax_off_lsb;
        AccYLSB += s_ay_off_lsb;
        AccZLSB += s_az_off_lsb;

        GyroX += s_gx_off_lsb;
        GyroY += s_gy_off_lsb;
        GyroZ += s_gz_off_lsb;

        // === conversiones idénticas ===
        ImuSample s;
        s.t_us = esp_timer_get_time();

        // Acelerómetro (g) ±8g, 4096 LSB/g
        s.acc_x_g = (float)AccXLSB / ACC_LSB_PER_G;
        s.acc_y_g = (float)AccYLSB / ACC_LSB_PER_G;
        s.acc_z_g = (float)AccZLSB / ACC_LSB_PER_G;

        // Gyro (°/s) ±500 dps, 65.5 LSB/(°/s)
        s.gyro_x_dps = (float)GyroX / GYRO_LSB_PER_DPS;
        s.gyro_y_dps = (float)GyroY / GYRO_LSB_PER_DPS;
        s.gyro_z_dps = (float)GyroZ / GYRO_LSB_PER_DPS;

        // Gyro (rad/s) por conveniencia futura
        s.gyro_x_rads = s.gyro_x_dps * DEG2RAD;
        s.gyro_y_rads = s.gyro_y_dps * DEG2RAD;
        s.gyro_z_rads = s.gyro_z_dps * DEG2RAD;

        const float kRAD2DEG = 57.29577951308232f; // 180/PI
        float ax = s.acc_x_g, ay = s.acc_y_g, az = s.acc_z_g;

        // Mismo cálculo que Arduino:
        s.roll_acc_deg = atan2f(ay, sqrtf(ax * ax + az * az)) * kRAD2DEG;
        s.pitch_acc_deg = -atan2f(ax, sqrtf(ay * ay + az * az)) * kRAD2DEG;

        // Publicar (lock-free)
        seqlock_write_begin(&s_shared.seq);
        s_shared.sample = s;
        seqlock_write_end(&s_shared.seq);
    }
}

bool imu_start_1khz(int core, int priority)
{
    if (s_imu_task)
        return true;

    BaseType_t ok = xTaskCreatePinnedToCore(
        imu_task, "imu_task", 4096, NULL, priority, &s_imu_task, core);

    if (ok != pdPASS)
    {
        rgb_led_set(255, 0, 0);
        return false;
    }
    return true;
}
