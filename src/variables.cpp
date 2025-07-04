#include <MPU6050.h> // Asegura que el compilador conoce la clase MPU6050
#include "variables.h"

// Define global variables
float vel_z = 0.0f;
float error_z = 0.0f;

MPU6050 accelgyro;

// Solo RatePitch, RateRoll y RateYaw deben ser volatile si se modifican en ISR
volatile float RatePitch = 0.0f;
volatile float RateRoll = 0.0f;
volatile float RateYaw = 0.0f;

float RateCalibrationRoll = 0.27f;
float RateCalibrationPitch = -0.85f;
float RateCalibrationYaw = -2.09f;
float AccXCalibration = 0.03f;
float AccYCalibration = 0.01f;
float AccZCalibration = -0.07f;

int pinLed = 2;
int ESCfreq = 500;

// Solo marcar como volatile si se usan en ISR
volatile float AngleRoll_est = 0.0f;
volatile float AnglePitch_est = 0.0f;
float tau_x = 0.0f, tau_y = 0.0f, tau_z = 0.0f;
float error_phi = 0.0f, error_theta = 0.0f, error_psi = 0.0f;

int buffersize = 1000;
int acel_deadzone = 8;
int giro_deadzone = 1;

int mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0;
int ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;

uint32_t LoopTimer = 0;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 15;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

// Timers y canales de radio: deben ser volatile
volatile uint32_t current_time = 0;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1 = 0;
volatile uint32_t timer_2 = 0;
volatile uint32_t timer_3 = 0;
volatile uint32_t timer_4 = 0;
volatile uint32_t timer_5 = 0;
volatile uint32_t timer_6 = 0;
volatile int ReceiverValue[6] = {0};
const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

// Solo AccX, AccY, AccZ y los ángulos estimados deberían ser volatile si los modifica una ISR
volatile float AccX = 0.0f, AccY = 0.0f, AccZ = 0.0f;
volatile float AngleRoll = 0.0f, AnglePitch = 0.0f, AngleYaw = 0.0f;

float GyroXdps = 0.0f, GyroYdps = 0.0f, GyroZdps = 0.0f;
int DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;
int InputRoll = 0, InputThrottle = 0, InputPitch = 0, InputYaw = 0;
int DesiredAngleRoll = 0, DesiredAnglePitch = 0, DesiredAngleYaw = 0;
float ErrorAngleRoll = 0.0f, ErrorAnglePitch = 0.0f;
float PrevErrorAngleRoll = 0.0f, PrevErrorAnglePitch = 0.0f;
float PrevItermAngleRoll = 0.0f, PrevItermAnglePitch = 0.0f;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

float MotorInput1 = 0.0f, MotorInput2 = 0.0f, MotorInput3 = 0.0f, MotorInput4 = 0.0f;

// Variables de estado (referencias e integrales): volatile si son compartidas con ISR
volatile float phi_ref = 0.0f;
volatile float theta_ref = 0.0f;
volatile float psi_ref = 0.0f;
volatile float integral_phi = 0.0f;
volatile float integral_theta = 0.0f;
volatile float integral_psi = 0.0f;

// === Variables para control avanzado ===
// Modo deslizante
float S_phi = 0, S_theta = 0; // Superficies deslizantes
float lambda_sliding = 0.5;   // Parámetro de deslizamiento

// Feedforward
float ff_phi = 0, ff_theta = 0, ff_psi = 0; // Términos feedforward
float prev_phi_ref = 0, prev_theta_ref = 0; // Referencias anteriores para derivada

float accAngleRoll;  // Ángulo de roll (grados)
float accAnglePitch; // Ángulo de pitch (grados)
float gyroRateRoll;
float gyroRatePitch;
float accAngleY;
float accAngleX;

float residual_history_roll[window_size] = {0};
float residual_history_pitch[window_size] = {0};
int residual_index_roll, residual_index_pitch;
float R_angle_roll, R_angle_pitch;
float lambda_roll, lambda_pitch;

// === Configuración del sistema ===
const uint16_t LOOP_FREQ = 100;               // Frecuencia del loop en Hz
const float DT = 1.0f / LOOP_FREQ;            // Paso de tiempo
const uint32_t LOOP_US = 1000000 / LOOP_FREQ; // Microsegundos por ciclo
const int IDLE_PWM = 1000;
float lambda = 0.96;
float residual_history[window_size] = {0};
int residual_index = 0;
float c_threshold = 0.01;

float dt = 0.004;       // Paso de tiempo (ajustar según la frecuencia de muestreo)
float Q_angle = 0.001f; // Covarianza del ruido del proceso (ángulo)
float Q_gyro = 0.003;   // Covarianza del ruido del proceso (giroscopio)
float R_angle = 0.03;   // Covarianza del ruido de medición (acelerómetro)

// --- CALIBRATION OFFSETS ---
double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
double gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// --- FILTER VARIABLES ---
double pitch = 0, roll = 0;
double Q_bias = 0.003f;
double R_measure = 0.03f;
double angle = 0.0f, bias = 0.0f, rate = 0;
double P[2][2] = {{0.0, 0.0}, {0.0, 0.0}};

Kalman kalmanRoll = {0, 0, {1, 0, 0, 1}};
Kalman kalmanPitch = {0, 0, {1, 0, 0, 1}};
Kalman kalmanYaw = {0, 0, {1, 0, 0, 1}};

unsigned long lastTime;
float T = 0.0f;

float magbias[3] = {0, 0, 0};  // Reemplaza estos valores tras calibrar
float magscale[3] = {1, 1, 1}; // Reemplaza estos valores tras calibrar

// Quaternion variables for orientation (initialize to identity quaternion)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Euler angles in radians (converted from quaternions)
float yaw = 0.0f;

// === UTILITY FUNCTIONS ===
float sat(float x, float epsilon)
{
  if (x > epsilon)
    return 1.0;
  if (x < -epsilon)
    return -1.0;
  return x / epsilon;
}

float k1;
float g1;
float k2;
float g2;

// Definición real (solo aquí)
float Kc_at[3][6] = {
    {2.1, 0, 0, 0.58, 0, 0},
    {0, 1.92, 0, 0, 0.38, 0},
    {0, 0, 5.3, 0, 0, 1.6}};

const float Ki_at[3][3] = {
    {0.08, 0, 0},
    {0, 0.08, 0},
    {0, 0, 0.01}};