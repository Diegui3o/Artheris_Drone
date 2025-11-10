set(srcs 
    "main.c"
    "control_loop.c"
    "imu.c"
    "kalman.c"
    "filter.c"
    "telemetry.c"
    "wifi_sta.c"
    "led.c"
    "cmd.c"
    "mode_control.c"
    "motor_ctrl.c"
    "cmd_motor.c"
    "manual_mode.c"
    "log_wrap_shim.c"
    "esc_arduino.cpp"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    PRIV_REQUIRES freertos esp_timer driver nvs_flash esp_netif esp_wifi esp_event lwip json
    REQUIRES espressif__arduino-esp32 ESP32Servo
)

target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-u __wrap_esp_log_write"
    "-u __wrap_esp_log_writev"
)