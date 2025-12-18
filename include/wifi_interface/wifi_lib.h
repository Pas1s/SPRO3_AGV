#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

// Event bits
#define WIFI_SUCCESS   BIT0
#define WIFI_FAILURE   BIT1
#ifndef MAX_FAILURES
#define MAX_FAILURES   10
#endif

extern EventGroupHandle_t wifi_event_group;
extern int s_retry_num;                 // reconnect tracker
extern const char *TAG_WIFI;

// Static IP (optional)
typedef struct {
    bool enable;
    esp_ip4_addr_t ip, gw, netmask, dns1, dns2;
} wifi_static_ip_t;

// Initialize netif + default loop and create Wi‑Fi (STA). Safe to call once at boot.
esp_err_t wifi_lib_init(void);

// Connect to AP using DHCP or optional static IP. Blocks until WIFI_SUCCESS/FAILURE bit.
esp_err_t wifi_lib_connect(const char *ssid, const char *pass,
                           const wifi_static_ip_t *sip_opt);

// Stop Wi‑Fi (keeps netif)
esp_err_t wifi_lib_stop(void);

// SNTP helpers
void sntp_sync_now(const char *server0);  // e.g. "pool.ntp.org"
void set_timezone(const char *tz);        // e.g. "CET-1CEST,M3.5.0/2,M10.5.0/3"

#ifdef __cplusplus
}
#endif