#include "wifi_interface/mqtt_lib.h"
#include "wifi_interface/Interface_updates.h"
#include "wifi_interface/wifi_lib.h"
#include "QR/tiny_code_reader_service.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "motor_control/line_follower.h"
#include "motor_control/motor_control.h"
#include "ToF/tof_sensor.h"
#define PATH_STATUS "Status"

static void interface_mqtt_msg_handler(const char *topic, int topic_len,
                                       const char *data,  int data_len);
static const char *TAG = "interface updates";
static char device_name_buf[32];
static const char *device_name = device_name_buf;


static esp_ip4_addr_t s_last_ip;
static bool s_last_ip_valid = false;
char ip_str[16];  // enough for "255.255.255.255\0"



/*** READ-SIDE state for Setup + Route ***/

// topic strings we’ll compare against
static char topic_high_speed[64];
static char topic_med_speed[64];
static char topic_low_speed[64];
static char topic_target_station[64];
static char topic_route_mode[64];

// cached values
static float   g_high_speed         = 0.0f;
static float   g_med_speed          = 0.0f;
static float   g_low_speed         = 0.0f;
static uint8_t g_target_station  = 0;

static area_t locations[20] = {0};



void set_device_root(const char *name) {
    if (!name) {
        device_name_buf[0] = '\0';
        return;
    }
    strncpy(device_name_buf, name, sizeof(device_name_buf) - 1);
    device_name_buf[sizeof(device_name_buf) - 1] = '\0';
}



void save_device_IP(const ip_event_got_ip_t *e)
{
    s_last_ip = e->ip_info.ip;
    s_last_ip_valid = true;
    esp_ip4addr_ntoa(&e->ip_info.ip, ip_str, sizeof(ip_str));
}


void init_interface(const char *name){
    set_device_root(name);

    interface_update_online();
    interface_update_area(0);
    interface_update_bat_level(69);
    interface_update_high_speed(1.0);
    interface_update_med_speed(0.40);
    interface_update_low_speed(0.20);
    interface_update_IP(ip_str);
    interface_update_mode(0);
    interface_update_payload(000000);
    interface_update_TOF(ToF_on);
    
    interface_update_speed(0.0);
    interface_update_target_station(1);

        // Build full topics: "<dev>/Setup/..." and "<dev>/Route/..."
    snprintf(topic_high_speed, sizeof(topic_high_speed),
             "%s/Setup/High_speed", device_name);

    snprintf(topic_med_speed, sizeof(topic_med_speed),
             "%s/Setup/Med_speed", device_name);

    snprintf(topic_low_speed, sizeof(topic_low_speed),
             "%s/Setup/Low_speed", device_name);

    snprintf(topic_target_station, sizeof(topic_target_station),
             "%s/Route/target_station", device_name);

    snprintf(topic_route_mode, sizeof(topic_route_mode),
             "%s/Route/Mode", device_name);   

    // Subscribe to all of them
    mqtt_sub(topic_high_speed, 1);
    mqtt_sub(topic_med_speed, 1);
    mqtt_sub(topic_low_speed, 1);
    mqtt_sub(topic_target_station, 1);
    mqtt_sub("Location/+/Type", 1);// gets all locations


}

/*** PUBLISH HELPERS ***/

void interface_update_bat_level(uint8_t level){
    static char topic[64];
    static char payload[8];

    snprintf(topic, sizeof(topic),
             "%s/Status/battery_level", device_name);

    snprintf(payload, sizeof(payload), "%u", level);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_area(uint8_t area){
    static char topic[64];
    static char payload[8];

    snprintf(topic, sizeof(topic),
             "%s/Status/Area", device_name);

    snprintf(payload, sizeof(payload), "%u", area);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_speed(float speed){
    static char topic[64];
    static char payload[16];

    snprintf(topic, sizeof(topic),
             "%s/Status/Speed_mode_speed", device_name);

    snprintf(payload, sizeof(payload), "%.2f", speed);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_payload(uint32_t payload_val){
    static char topic[64];
    static char payload[16];

    snprintf(topic, sizeof(topic),
             "%s/Status/Payload", device_name);

    snprintf(payload, sizeof(payload), "%lu", (unsigned long)payload_val);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_online(void){
    static char topic[64];
    const char *payload = "Online";

    snprintf(topic, sizeof(topic),
             "%s/Status/", device_name);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_mode(uint8_t mode){
    static char topic[64];
    static char payload[8];

    snprintf(topic, sizeof(topic),
             "%s/Status/Mode", device_name);

    snprintf(payload, sizeof(payload), "%u", mode);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_TOF(bool tof_on){
    static char topic[64];
    static char payload[8];

    snprintf(topic, sizeof(topic),
             "%s/Status/TOF", device_name);

    if (tof_on){
        snprintf(payload, sizeof(payload), "%s", "on");
    } else {
        snprintf(payload, sizeof(payload), "%s", "off");
    }
    

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish TOF (mqtt_pub returned %d)", id);
    }
}

void interface_update_IP(const char *address){
    static char topic[64];

    snprintf(topic, sizeof(topic),
             "%s/Network/IP_adresse", device_name);

    int id = mqtt_pub(topic, address, strlen(address), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_high_speed(float speed){
    static char topic[64];
    static char payload[16];

    snprintf(topic, sizeof(topic),
             "%s/Setup/High_speed", device_name);

    snprintf(payload, sizeof(payload), "%.2f", speed);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}
void interface_update_med_speed(float speed){
    static char topic[64];
    static char payload[16];

    snprintf(topic, sizeof(topic),
             "%s/Setup/Med_speed", device_name);

    snprintf(payload, sizeof(payload), "%.2f", speed);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_low_speed(float speed){
    static char topic[64];
    static char payload[16];

    snprintf(topic, sizeof(topic),
             "%s/Setup/Low_speed", device_name);

    snprintf(payload, sizeof(payload), "%.2f", speed);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}

void interface_update_target_station(uint8_t target){
    static char topic[64];
    static char payload[8];

    snprintf(topic, sizeof(topic),
             "%s/Route/target_station", device_name);

    snprintf(payload, sizeof(payload), "%u", target);

    int id = mqtt_pub(topic, payload, strlen(payload), 1, true);
    if (id < 0) {
        ESP_LOGE(TAG, "Failed to publish Area (mqtt_pub returned %d)", id);
    }
}


void update_area_type(uint8_t area, uint8_t value){
    locations[area-1].Type=value;
}

static void interface_mqtt_msg_handler(const char *topic, int topic_len,
                                       const char *data,  int data_len)
{
    // Make null-terminated copies
    char tbuf[64];
    char dbuf[32];

    int tlen = topic_len < (int)sizeof(tbuf) - 1 ? topic_len : (int)sizeof(tbuf) - 1;
    int dlen = data_len  < (int)sizeof(dbuf) - 1 ? data_len  : (int)sizeof(dbuf) - 1;

    memcpy(tbuf, topic, tlen);
    tbuf[tlen] = '\0';

    memcpy(dbuf, data, dlen);
    dbuf[dlen] = '\0';

    // Compare and parse
    if (strcmp(tbuf, topic_high_speed) == 0) {
        g_high_speed = strtof(dbuf, NULL);
        ESP_LOGI(TAG, "Updated high_speed = %.2f", g_high_speed);

    }
    else if (strcmp(tbuf, topic_med_speed) == 0) {
        g_med_speed = strtof(dbuf, NULL);
        ESP_LOGI(TAG, "Updated med_speed = %.2f", g_med_speed);

    }
    else if (strcmp(tbuf, topic_low_speed) == 0) {
        g_low_speed = strtof(dbuf, NULL);
        ESP_LOGI(TAG, "Updated low_speed = %.2f", g_low_speed);

    }
    else if (strcmp(tbuf, topic_target_station) == 0) {
        long v = strtol(dbuf, NULL, 10);
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        g_target_station = (uint8_t)v;
        ESP_LOGI(TAG, "Updated target_station = %u", g_target_station);
    }  else {
        const char *prefix = "Location/Area_";
        const char *suffix = "/Type";

        if (strncmp(tbuf, prefix, strlen(prefix)) == 0) {
            const char *after_prefix = tbuf + strlen(prefix);  // points at "<N>/Type"
            char *slash = strchr(after_prefix, '/');
            if (slash && strcmp(slash, suffix) == 0) {
                // Extract "<N>"
                char area_str[8] = {0};
                size_t area_len = (size_t)(slash - after_prefix);

                if (area_len > 0 && area_len < sizeof(area_str)) {
                    memcpy(area_str, after_prefix, area_len);
                    area_str[area_len] = '\0';

                    uint8_t area_index = atoi(area_str);  // Area number
                    // Interpret payload; here assuming Type is an integer
                    uint8_t type_value = atoi(dbuf);

                    ESP_LOGI(TAG, "Area_%d Type = %d (topic=%s, payload=%s)",
                             area_index, type_value, tbuf, dbuf);
                    update_area_type(area_index, type_value);
                }
            }
        }
    }
}


void interface_register_mqtt_handler(void)
{
    mqtt_set_msg_handler(interface_mqtt_msg_handler);
}

float interface_get_high_speed(void){

    return g_high_speed;
}

float interface_get_med_speed(void){

    return g_med_speed;
}

float interface_get_low_speed(void){

    return g_low_speed;
}

uint8_t interface_get_target_station(void){

    return g_target_station;
}


