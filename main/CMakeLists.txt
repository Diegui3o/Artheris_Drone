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
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    PRIV_REQUIRES freertos esp_timer driver nvs_flash esp_netif esp_wifi esp_event lwip json
)