#pragma once
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "mqtt_client.h"
#include "sd_card.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const char *TAG_MQTT;
extern esp_mqtt_client_handle_t client; // global like your legacy code

// TLS config knobs (optional)
typedef struct {
    const char *uri;                      // mqtt:// or mqtts://
    const char *client_id;                // optional
    const char *username;                 // optional
    const char *password;                 // optional
    const char *server_cert_pem;          // root CA or self-signed
    const char *client_cert_pem;          // mTLS cert (e.g., AWS)
    const char *client_key_pem;           // mTLS key
    bool skip_common_name_check;          // set true for some brokers
    int keepalive_s;                      // default 60
    const char *lwt_topic;                // optional
    const char *lwt_msg;                  // optional
    int lwt_qos;                          // 0/1/2
    bool lwt_retain;                      // retain flag
} mqtt_lib_cfg_t;

void mqtt_lib_start(const mqtt_lib_cfg_t *cfg); // init+start, registers handler
void mqtt_lib_stop(void);

int mqtt_sub(const char *topic, int qos);
int mqtt_unsub(const char *topic);
int mqtt_pub(const char *topic, const void *data, int len, int qos, bool retain);
void mqtt_start_from_cfg(const app_config_t *cfg);

// Callback type for incoming MQTT messages
typedef void (*mqtt_msg_handler_t)(const char *topic, int topic_len,
                                   const char *data,  int data_len);

// Register a global message handler (called on MQTT_EVENT_DATA)
void mqtt_set_msg_handler(mqtt_msg_handler_t cb);

#ifdef __cplusplus
}
#endif

