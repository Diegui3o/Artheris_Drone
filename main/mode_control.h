#pragma once

typedef enum {
    MODE_PILOT  = 0,
    MODE_WAIT   = 1,
    MODE_MANUAL = 2
} drone_mode_t;

void mode_control_start_core1(int priority);
void changeMode(drone_mode_t newMode);
drone_mode_t getMode(void);