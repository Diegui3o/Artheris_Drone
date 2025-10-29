#ifndef CMD_MOTOR_H
#define CMD_MOTOR_H

#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

void handle_motor_cmd(cJSON *root);

#ifdef __cplusplus
}
#endif

#endif // CMD_MOTOR_H