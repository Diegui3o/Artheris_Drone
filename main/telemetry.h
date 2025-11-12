
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    bool telemetry_start_core0(const char *remote_ip, uint16_t remote_port, int priority);
    bool telemetry_start_core1(const char *remote_ip, uint16_t remote_port, int priority);

#ifdef __cplusplus
}
#endif