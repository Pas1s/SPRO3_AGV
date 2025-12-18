#include "wifi_interface/wifi_lib.h"
#include "wifi_interface/Interface_updates.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

EventGroupHandle_t wifi_event_group = NULL;
int s_retry_num = 0;
const char *TAG_WIFI = "WIFI";
static esp_netif_t *s_sta = NULL;



static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG_WIFI, "Connecting to AP...");
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t*)data;
        ESP_LOGW(TAG_WIFI, "Disconnected: reason=%d", e ? e->reason : -1);
        if (s_retry_num < MAX_FAILURES) {
            s_retry_num++;
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
        }
    }
}

static void ip_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG_WIFI, "STA IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_num = 0;
        save_device_IP(e);
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);

    }
}

esp_err_t wifi_lib_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (!s_sta) s_sta = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    return esp_wifi_init(&cfg);
}

esp_err_t wifi_lib_connect(const char *ssid, const char *pass, const wifi_static_ip_t *sip)
{
    if (!wifi_event_group) wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t h1, h2;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &h1));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, &h2));

    wifi_config_t wc = {0};
    strncpy((char*)wc.sta.ssid, ssid, sizeof(wc.sta.ssid)-1);
    if (pass) strncpy((char*)wc.sta.password, pass, sizeof(wc.sta.password)-1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // override via Kconfig if needed
    wc.sta.pmf_cfg = (wifi_pmf_config_t){ .capable = true, .required = false };
#if ESP_IDF_VERSION_MAJOR >= 5
    wc.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
#endif

    // Optional static IP
    if (sip && sip->enable) {
        esp_netif_dhcpc_stop(s_sta);
        esp_netif_ip_info_t info = {.ip = sip->ip, .gw = sip->gw, .netmask = sip->netmask};
        ESP_ERROR_CHECK(esp_netif_set_ip_info(s_sta, &info));
        if (sip->dns1.addr) { esp_netif_dns_info_t d1 = {.ip = {.u_addr.ip4 = sip->dns1, .type = ESP_IPADDR_TYPE_V4}}; esp_netif_set_dns_info(s_sta, ESP_NETIF_DNS_MAIN, &d1);} 
        if (sip->dns2.addr) { esp_netif_dns_info_t d2 = {.ip = {.u_addr.ip4 = sip->dns2, .type = ESP_IPADDR_TYPE_V4}}; esp_netif_set_dns_info(s_sta, ESP_NETIF_DNS_BACKUP, &d2);} 
    } else {
        esp_netif_dhcpc_start(s_sta);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N));
    wifi_country_t eu = {.cc="EU", .schan=1, .nchan=13, .policy=WIFI_COUNTRY_POLICY_AUTO};
    esp_wifi_set_country(&eu);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_SUCCESS|WIFI_FAILURE, pdFALSE, pdFALSE, portMAX_DELAY);

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, h2));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, h1));

    if (bits & WIFI_SUCCESS) return ESP_OK;
    return ESP_FAIL;
}

esp_err_t wifi_lib_stop(void)
{
    return esp_wifi_stop();
}






