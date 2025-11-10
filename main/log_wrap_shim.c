#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_idf_version.h"

// Provee el símbolo que esp_wifi espera: __wrap_esp_log_writev
void __wrap_esp_log_writev(esp_log_level_t level, const char *tag, const char *fmt, va_list args)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // IDF 5.x: existe esp_log_writev y es void
    esp_log_writev(level, tag, fmt, args);
#else
    // IDF 4.x: no hay writev; formateamos y usamos esp_log_write
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    esp_log_write(level, tag, "%s", buf); // también es void
#endif
}

// Provee __wrap_esp_log_write en términos de la anterior
void __wrap_esp_log_write(esp_log_level_t level, const char *tag, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    __wrap_esp_log_writev(level, tag, fmt, args);
    va_end(args);
}
