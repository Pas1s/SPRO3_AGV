
#include "wifi_interface/mqtt_lib.h"
#include "wifi_interface/Interface_updates.h"
#include "esp_log.h"
#include "sd_card.h"

const char *TAG_MQTT = "MQTT_CLIENT";
esp_mqtt_client_handle_t client = NULL;
const char *AGV_NAME = NULL;
static mqtt_msg_handler_t s_msg_handler = NULL;


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "CONNECTED");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "DATA on %.*s: %.*s",
                    e->topic_len, e->topic, e->data_len, e->data);

            if (s_msg_handler) {
                s_msg_handler(e->topic, e->topic_len, e->data, e->data_len);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "DISCONNECTED → reconnect");
            esp_mqtt_client_reconnect(client);
            break;
        default:
            ESP_LOGI(TAG_MQTT, "event id=%d", e->event_id);
            break;
    }
}

void mqtt_lib_start(const mqtt_lib_cfg_t *cfg)
{
    esp_mqtt_client_config_t mc = {
        .broker = {
            .address.uri = cfg->uri,
            .verification = {
                .skip_cert_common_name_check = cfg->skip_common_name_check,
                .certificate = cfg->server_cert_pem,
            },
        },
        .credentials = {
            .client_id = cfg->client_id,
            .username = cfg->username,
            .authentication = {
                .password = cfg->password,
                .certificate = cfg->client_cert_pem,
                .key = cfg->client_key_pem,
            },
        },
        .session = {
            .keepalive = cfg->keepalive_s > 0 ? cfg->keepalive_s : 60,
            .last_will = {
                .topic = cfg->lwt_topic,
                .msg = cfg->lwt_msg,
                .qos = cfg->lwt_qos,
                .retain = cfg->lwt_retain,
            },
        },
    };
    client = esp_mqtt_client_init(&mc);
    if (!client) { ESP_LOGE(TAG_MQTT, "init failed"); return; }
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK) ESP_LOGE(TAG_MQTT, "start failed 0x%x", err);
}

void mqtt_start_from_cfg(const app_config_t *cfg)
{
    
    AGV_NAME = cfg->device.name;
    // 1) build URI string
    static char mqtt_uri[128];  // static so it stays valid after this function returns
    snprintf(mqtt_uri, sizeof(mqtt_uri),
             "mqtt://%s:%d",            // or "mqtts://" if you use TLS
             cfg->mqtt.broker,
             cfg->mqtt.port);

    static char mqtt_lwt_topic[50];
    snprintf(mqtt_lwt_topic, sizeof(mqtt_lwt_topic),
             "%s/Status/",            
             cfg->device.name);

    // 2) fill config struct
    mqtt_lib_cfg_t mcfg = {
        .uri = mqtt_uri,
        .client_id = cfg->mqtt.client_id,
        .username = cfg->mqtt.username,
        .password = cfg->mqtt.password,

        .server_cert_pem = NULL,   // set these if you use TLS
        .client_cert_pem = NULL,
        .client_key_pem = NULL,
        .skip_common_name_check = false,
        .keepalive_s = 30,

        .lwt_topic = "AGV/Status/",  // or build from cfg->mqtt.base_topic if you want
        .lwt_msg = "offline",
        .lwt_qos = 1,
        .lwt_retain = true,
    };
    interface_register_mqtt_handler(); // uses the handler from interface_updates.c
    mqtt_lib_start(&mcfg);  
}


void mqtt_lib_stop(void)
{
    if (!client) return;
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    client = NULL;
}

int mqtt_sub(const char *topic, int qos)
{
    if (!client) return -1;
    return esp_mqtt_client_subscribe(client, topic, qos);
}

int mqtt_unsub(const char *topic)
{
    if (!client) return -1;
    return esp_mqtt_client_unsubscribe(client, topic);
}

int mqtt_pub(const char *topic, const void *data, int len, int qos, bool retain)
{
    if (!client) return -1;
    return esp_mqtt_client_publish(client, topic, (const char*)data, len, qos, retain);
}

void mqtt_set_msg_handler(mqtt_msg_handler_t cb)
{
    s_msg_handler = cb;
}
