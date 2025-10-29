#pragma once
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif


// Initializes GPIO/ISR, LEDC, etc. and launches the 1 kHz loop task.
void manual_mode_start(void);


// Stops timer + task, disarms motors.
void manual_mode_stop(void);


#ifdef __cplusplus
}
#endif