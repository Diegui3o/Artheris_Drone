#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_netif_ip_addr.h"
#include "esp_err.h"
#include "esp_netif.h"

typedef struct {
    const char *ssid;
    const char *pass;
    uint32_t ip;       // inet_addr() o 0 para DHCP
    uint32_t gw;
    uint32_t netmask;
    int connect_timeout_ms;
    int max_retries;
} wifi_sta_cfg_t;

esp_err_t wifi_sta_start(const wifi_sta_cfg_t *cfg);
bool      wifi_sta_is_up(void);
bool      wifi_sta_get_ip(esp_netif_ip_info_t *out);

esp_err_t wifi_sta_start(const wifi_sta_cfg_t *cfg);
/** Hace disconnect/stop y desregistra handlers (idempotente) */
void wifi_sta_stop(void);
/** ¿Está “operativo” (conectado y con IP)? */
bool wifi_sta_is_up(void);
/** Copia la IP actual (si hay) */
bool wifi_sta_get_ip(esp_netif_ip_info_t *out);
