#include <stdint.h>
#include "esp_wifi.h"


typedef struct{
    uint8_t Type;
    uint8_t Vehcials;

} area_t;

void set_device_root(const char *name);
void init_interface(const char *name);
void save_device_IP(const ip_event_got_ip_t *e);

/*** PUBLISH HELPERS ***/

void interface_update_bat_level(uint8_t level);
void interface_update_area(uint8_t area);
void interface_update_speed(float speed);
void interface_update_payload(uint32_t payload_val);
void interface_update_online(void);
void interface_update_mode(uint8_t mode);
void interface_update_IP(const char *address);
void interface_update_high_speed(float speed);
void interface_update_med_speed(float speed);
void interface_update_low_speed(float speed);
void interface_update_target_station(uint8_t target);

float interface_get_high_speed(void);
float interface_get_med_speed(void);
float interface_get_low_speed(void);
uint8_t interface_get_target_station(void);
void interface_update_TOF(bool tof_on);
void interface_register_mqtt_handler(void);