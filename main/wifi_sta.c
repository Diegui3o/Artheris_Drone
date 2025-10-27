#include "wifi_sta.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "WIFI";

static EventGroupHandle_t s_ev; // bits de estado
#define BIT_CONNECTED BIT0
#define BIT_GOT_IP BIT1

static bool s_started = false;
static bool s_handlers_registered = false;
static int s_retry_count = 0;
static int s_retry_max = 0;
static esp_netif_t *s_netif = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "STA start -> connect");
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "STA connected");
        xEventGroupSetBits(s_ev, BIT_CONNECTED);
        s_retry_count = 0;
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "STA disconnected");
        xEventGroupClearBits(s_ev, BIT_CONNECTED | BIT_GOT_IP);

        if (s_retry_count < s_retry_max)
        {
            int backoff = 500 * (1 << (s_retry_count > 5 ? 5 : s_retry_count)); // máx ~16s
            s_retry_count++;
            ESP_LOGW(TAG, "Reintento en %d ms (try %d/%d)", backoff, s_retry_count, s_retry_max);
            vTaskDelay(pdMS_TO_TICKS(backoff));
            esp_wifi_connect();
        }
        else
        {
            ESP_LOGE(TAG, "Máximo de reintentos alcanzado");
        }
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "GOT_IP: " IPSTR " gw=" IPSTR " mask=" IPSTR,
                 IP2STR(&evt->ip_info.ip), IP2STR(&evt->ip_info.gw), IP2STR(&evt->ip_info.netmask));
        xEventGroupSetBits(s_ev, BIT_GOT_IP);
    }
}

esp_err_t wifi_sta_start(const wifi_sta_cfg_t *cfg)
{
    // NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Netif + loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Crear netif STA (si no existía)
    s_netif = esp_netif_create_default_wifi_sta();
    assert(s_netif != NULL);

    // WiFi init
    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&icfg));

    // Handlers (solo una vez)
    if (!s_handlers_registered)
    {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
        s_handlers_registered = true;
    }

    // Config STA
    wifi_config_t w = {0};
    strncpy((char *)w.sta.ssid, cfg->ssid, sizeof(w.sta.ssid) - 1);
    if (cfg->pass)
        strncpy((char *)w.sta.password, cfg->pass, sizeof(w.sta.password) - 1);
    w.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    w.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH; // compatibilidad
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &w));

    // Event group
    if (!s_ev)
    {
        s_ev = xEventGroupCreate();
        if (!s_ev)
        {
            ESP_LOGE(TAG, "No se pudo crear event group");
            return ESP_ERR_NO_MEM;
        }
    }

    // DHCP o IP estática
    if (cfg->ip == 0)
    {
        // DHCP cliente habilitado
        esp_netif_dhcp_status_t st;
        if (esp_netif_dhcpc_get_status(s_netif, &st) == ESP_OK && st != ESP_NETIF_DHCP_STARTED)
        {
            ESP_ERROR_CHECK(esp_netif_dhcpc_start(s_netif));
        }
        ESP_LOGI(TAG, "DHCP cliente -> ON");
    }
    else
    {
        // ---- IP fija (solución robusta para v5.5.x) ----
        esp_netif_dhcp_status_t st;

        // 1️⃣ Detener DHCP si está activo
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_netif));

        // 2️⃣ Verificar que DHCP está realmente detenido
        ESP_ERROR_CHECK(esp_netif_dhcpc_get_status(s_netif, &st));
        if (st != ESP_NETIF_DHCP_STOPPED)
        {
            ESP_LOGE(TAG, "No se pudo detener DHCP");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "DHCP detenido correctamente");

        // 3️⃣ Aplicar IP estática (seguro ahora)
        esp_netif_ip_info_t ipi = {
            .ip.addr = cfg->ip,
            .gw.addr = cfg->gw,
            .netmask.addr = cfg->netmask,
        };
        ESP_ERROR_CHECK(esp_netif_set_ip_info(s_netif, &ipi));
        ESP_LOGI(TAG, "IP estática aplicada correctamente");
    }

    // Conexión
    s_retry_max = (cfg->max_retries < 0) ? 0 : cfg->max_retries;
    s_retry_count = 0;

    ESP_ERROR_CHECK(esp_wifi_start()); // -> WIFI_EVENT_STA_START -> connect
    ESP_LOGI(TAG, "STA start -> connect");

    // Esperar conexión
    uint32_t tmo_ms = cfg->connect_timeout_ms ? cfg->connect_timeout_ms : 15000;
    EventBits_t bits = xEventGroupWaitBits(s_ev, BIT_CONNECTED, pdFALSE, pdFALSE, pdMS_TO_TICKS(tmo_ms));
    if (!(bits & BIT_CONNECTED))
    {
        ESP_LOGW(TAG, "Timeout esperando asociación (%ums)", tmo_ms);
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Conectado al AP");

    // Esperar IP si DHCP
    if (cfg->ip == 0)
    {
        bits = xEventGroupWaitBits(s_ev, BIT_GOT_IP, pdFALSE, pdFALSE, pdMS_TO_TICKS(3000));
        if (!(bits & BIT_GOT_IP))
        {
            ESP_LOGW(TAG, "Conectado pero sin IP (timeout 3s DHCP)");
        }
        else
        {
            ESP_LOGI(TAG, "GOT_IP listo");
        }
    }
    else
    {
        ESP_LOGI(TAG, "Usando IP estática configurada");
    }

    s_started = true;
    ESP_LOGI(TAG, "WiFi STA listo.");
    return ESP_OK;
}

void wifi_sta_stop(void)
{
    if (!s_started)
        return;
    ESP_LOGI(TAG, "stop");

    if (s_handlers_registered)
    {
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
        s_handlers_registered = false;
    }

    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_ev)
    {
        vEventGroupDelete(s_ev);
        s_ev = NULL;
    }
    s_started = false;
}

bool wifi_sta_is_up(void)
{
    if (!s_ev)
        return false;
    EventBits_t b = xEventGroupGetBits(s_ev);
    return (b & (BIT_CONNECTED | BIT_GOT_IP)) == (BIT_CONNECTED | BIT_GOT_IP);
}

bool wifi_sta_get_ip(esp_netif_ip_info_t *out)
{
    if (!s_netif || !out)
        return false;
    if (!wifi_sta_is_up())
        return false;
    return esp_netif_get_ip_info(s_netif, out) == ESP_OK;
}
