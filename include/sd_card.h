#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

// SPI pin mapping for SD card (adjust if your wiring is different)
#define PIN_NUM_MISO 13 // SD → ESP32
#define PIN_NUM_MOSI 11 // ESP32 → SD
#define PIN_NUM_CLK  12 // SPI Clock
#define PIN_NUM_CS   10 // Chip Select

// Legacy constant from earlier CSV code (kept for compatibility)
#define ARRAY_COLS 8

// --------- Configuration structures loaded from JSON on SD ----------

typedef struct {
    char ssid[32];
    char password[64];
} app_wifi_config_t;

typedef struct {
    char broker[64];     // Hostname or IP of MQTT broker
    int  port;           // Usually 1883
    char username[32];
    char password[64];
    char client_id[32];
    char base_topic[64]; // e.g. "agv/01/"
    int location_count;
} mqtt_config_t;

typedef struct {
    char name[32]; // device name, e.g. "AGV01"
    int  start_area;     // some integer area id
} device_config_t;

typedef struct {
    app_wifi_config_t   wifi;
    mqtt_config_t   mqtt;
    device_config_t device;
} app_config_t;

// ---------------------- SD-card helper functions ---------------------

// Disable stdio buffering so printf/ESP_LOG* appear immediately on UART
void init_stdio_buffering(void);

// Mount the SD card using SPI host/slot configuration
esp_err_t mount_sdcard(const char *mount_point,
                       sdmmc_host_t *host,
                       sdspi_device_config_t *slot_cfg,
                       esp_vfs_fat_sdmmc_mount_config_t *mount_cfg,
                       sdmmc_card_t **card);

// Unmount and free resources
void cleanup_sdcard(const char *mount_point,
                    sdmmc_card_t *card,
                    spi_host_device_t host_slot);

// Convenience init/uninit that wraps everything for you
void init_sdcard(void);
void unmount_sdcard(void);

// Build a CSV file path based on struct/file numbers.
// NOTE: Returns pointer to an internal static buffer (not thread-safe).
char *sd_path(int struct_num, int file_num);


// ----------------------- JSON config readers -------------------------

// Load only Wi-Fi credentials from /sdcard/config.json
bool sd_read_wifi_config_json(app_wifi_config_t *cfg);

// Load full app configuration (Wi-Fi, MQTT, device info) from JSON
bool sd_read_app_config_json(app_config_t *cfg);


void list_sd_root(void);