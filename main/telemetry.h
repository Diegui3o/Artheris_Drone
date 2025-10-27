#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

bool telemetry_start_core0(const char *remote_ip, uint16_t remote_port, int priority);

#endif // TELEMETRY_H