#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "cJSON.h"

#include "sd_card.h"

static const char *TAG = "sd_card";

// Global SD card context (simple singleton for this project)
static const char *SD_MOUNT_POINT = "/sdcard";
static sdmmc_card_t *g_card = NULL;

static sdmmc_host_t g_host = SDSPI_HOST_DEFAULT();
static sdspi_device_config_t g_slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
static esp_vfs_fat_sdmmc_mount_config_t g_mount_config;

// ---------------------- internal helpers ------------------------

void init_stdio_buffering(void)
{
    // Disable buffering so logs appear immediately on UART
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

esp_err_t mount_sdcard(const char *mount_point,
                       sdmmc_host_t *host,
                       sdspi_device_config_t *slot_cfg,
                       esp_vfs_fat_sdmmc_mount_config_t *mount_cfg,
                       sdmmc_card_t **card)
{
    esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_point,
                                            host,
                                            slot_cfg,
                                            mount_cfg,
                                            card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem on %s (%s)",
                 mount_point, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Filesystem mounted on %s", mount_point);
    return ESP_OK;
}

void cleanup_sdcard(const char *mount_point,
                    sdmmc_card_t *card,
                    spi_host_device_t host_slot)
{
    if (card) {
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        ESP_LOGI(TAG, "Card unmounted from %s", mount_point);
    }
    spi_bus_free(host_slot);
    ESP_LOGI(TAG, "SPI bus freed");
}

void init_sdcard(void)
{
    esp_err_t ret;

    init_stdio_buffering();

    // Initialize NVS (needed by some ESP-IDF components, harmless otherwise)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configure SD SPI host
    

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(g_host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus (%s)", esp_err_to_name(ret));
        return;
    }

    g_slot_config.gpio_cs = PIN_NUM_CS;
    g_slot_config.host_id = g_host.slot;

    g_mount_config.format_if_mount_failed = false;
    g_mount_config.max_files = 5;
    g_mount_config.allocation_unit_size = 16 * 1024;

    ret = mount_sdcard(SD_MOUNT_POINT, &g_host, &g_slot_config,
                       &g_mount_config, &g_card);
    if (ret != ESP_OK) {
        spi_bus_free(g_host.slot);
        return;
    }

    // Print card info to stdout
    sdmmc_card_print_info(stdout, g_card);
        
    list_sd_root();
}

void unmount_sdcard(void)
{
    if (!g_card) {
        ESP_LOGW(TAG, "unmount_sdcard() called but card is not mounted");
        return;
    }

    cleanup_sdcard(SD_MOUNT_POINT, g_card, g_host.slot);
    g_card = NULL;
}

// Build a simple CSV path like "/sdcard/struct_01_file_02.csv"
char *sd_path(int struct_num, int file_num)
{
    static char path[64];
    snprintf(path, sizeof(path),
             "%s/struct_%02d_file_%02d.csv",
             SD_MOUNT_POINT, struct_num, file_num);
    return path;
}


// ------------------------ JSON config helpers -----------------------

static char *read_file_to_buffer(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s, errno=%d", path, errno);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0 || size > 4096) {
        ESP_LOGE(TAG, "Bad file size %ld for %s", size, path);
        fclose(f);
        return NULL;
    }

    char *buf = malloc(size + 1);
    if (!buf) {
        ESP_LOGE(TAG, "Out of memory while reading %s", path);
        fclose(f);
        return NULL;
    }

    size_t read = fread(buf, 1, size, f);
    fclose(f);

    buf[read] = '\0';
    return buf;
}

// Extract a string field from an object into a fixed-size C buffer.
static void copy_json_string_field(cJSON *obj,
                                   const char *field_name,
                                   char *dest,
                                   size_t dest_len)
{
    if (!obj || !field_name || !dest || dest_len == 0) {
        return;
    }

    cJSON *item = cJSON_GetObjectItem(obj, field_name);
    if (cJSON_IsString(item)) {
        strncpy(dest, item->valuestring, dest_len - 1);
        dest[dest_len - 1] = '\0';
    } else {
        dest[0] = '\0';
    }
}

bool sd_read_app_config_json(app_config_t *cfg)
{
    if (!cfg) {
        return false;
    }

    const char *config_path = "/sdcard/CONFIG.CFG";

    char *json_str = read_file_to_buffer(config_path);
    if (!json_str) {
        return false;
    }

    cJSON *root = cJSON_Parse(json_str);
    free(json_str);

    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON in %s", config_path);
        return false;
    }

    // ---- wifi ----
    cJSON *wifi = cJSON_GetObjectItem(root, "wifi");
    if (cJSON_IsObject(wifi)) {
        copy_json_string_field(wifi, "ssid",
                               cfg->wifi.ssid, sizeof(cfg->wifi.ssid));
        copy_json_string_field(wifi, "password",
                               cfg->wifi.password, sizeof(cfg->wifi.password));
    }

    // ---- mqtt ----
    cJSON *mqtt = cJSON_GetObjectItem(root, "mqtt");
    if (cJSON_IsObject(mqtt)) {
        copy_json_string_field(mqtt, "broker",
                               cfg->mqtt.broker, sizeof(cfg->mqtt.broker));

        cJSON *port = cJSON_GetObjectItem(mqtt, "port");
        if (cJSON_IsNumber(port)) {
            cfg->mqtt.port = port->valueint;
        } else {
            cfg->mqtt.port = 1883; // default
        }

        copy_json_string_field(mqtt, "username",
                               cfg->mqtt.username, sizeof(cfg->mqtt.username));
        copy_json_string_field(mqtt, "password",
                               cfg->mqtt.password, sizeof(cfg->mqtt.password));
        copy_json_string_field(mqtt, "client_id",
                               cfg->mqtt.client_id, sizeof(cfg->mqtt.client_id));
        copy_json_string_field(mqtt, "base_topic",
                               cfg->mqtt.base_topic, sizeof(cfg->mqtt.base_topic));
        



    } else {
        cfg->mqtt.port = 1883;
        cfg->mqtt.broker[0] = '\0';
        cfg->mqtt.username[0] = '\0';
        cfg->mqtt.password[0] = '\0';
        cfg->mqtt.client_id[0] = '\0';
        cfg->mqtt.base_topic[0] = '\0';
    }

    // ---- device ----
    cJSON *device = cJSON_GetObjectItem(root, "device");
    if (cJSON_IsObject(device)) {
        copy_json_string_field(device, "name",
                               cfg->device.name, sizeof(cfg->device.name));

        cJSON *area = cJSON_GetObjectItem(device, "area");
        if (cJSON_IsNumber(area)) {
            cfg->device.start_area = area->valueint;
        } else {
            cfg->device.start_area = 0;
        }
    } else {
        cfg->device.name[0] = '\0';
        cfg->device.start_area = 0;
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Loaded app config from %s (ssid='%s', broker='%s', area=%d)",
             config_path,
             cfg->wifi.ssid,
             cfg->mqtt.broker,
             cfg->device.start_area);
    return true;
}

bool sd_read_wifi_config_json(app_wifi_config_t *cfg)
{
    if (!cfg) {
        return false;
    }

    app_config_t app_cfg = {0};
    if (!sd_read_app_config_json(&app_cfg)) {
        return false;
    }

    memcpy(cfg, &app_cfg.wifi, sizeof(app_wifi_config_t));
    return true;
}

void list_sd_root(void)
{
    DIR *dir = opendir("/sdcard");
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open /sdcard for listing");
        return;
    }

    struct dirent *ent;
    ESP_LOGI(TAG, "Contents of /sdcard:");
    while ((ent = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "  %s", ent->d_name);
    }
    closedir(dir);
}
