// tiny_code_reader_service.c

#include "QR/tiny_code_reader_service.h"
#include "QR/tiny_code_reader.h"
#include "wifi_interface/Interface_updates.h"
#include "ToF/tof_sensor.h"
#include "i2c_bus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "esp_timer.h"
#include <stdint.h>
#include "motor_control/line_follower.h"
#include "motor_control/motor_control.h"

static const char *TAG = "tcr_service";

static i2c_master_dev_handle_t s_tcr_dev = NULL;
static TaskHandle_t s_tcr_task_handle = NULL;
static motors_t *motors;
// QR numeric command codes (0–40 reserved for commands)
#define QR_CMD_STOP               1
#define QR_CMD_HIGH_SPEED         2
#define QR_CMD_MED_SPEED          3
#define QR_CMD_LOW_SPEED          4
#define QR_CMD_PICKUP             5
#define QR_CMD_DROP_OFF           6
#define QR_CMD_PAUSE              7
#define QR_CMD_DISABLE_TOF        8
#define QR_CMD_ENABLE_TOF         9
#define QR_CMD_LINE_CHANGE_RIGHT  10
#define QR_CMD_LINE_CHANGE_LEFT   11
#define QR_CMD_CHARGING           20
#define QR_CMD_CHANGE_OF_SURFACE  21


static void tcr_handle_code(int code, const char *raw)
{
    // Areas: 41..100 => area 1..60
    if (code >= 41 && code <= 100) {
        uint8_t area = (uint8_t)(code - 40);   // 44 => 4
        interface_update_area(area);
        ESP_LOGI(TAG, "QR AREA: code=%d -> area=%u (raw=%s)", code, (unsigned)area, raw);
        return;
    }

    // Commands: 0..40 (you define what each means)
    switch (code) {
        case QR_CMD_STOP:
            //ESP_LOGI(TAG, "QR: STOP (%s)", raw);
            ESP_LOGI(TAG, "QR: ENABLE TOF (%s)", raw);
            ToF_on=true;
            interface_update_TOF(ToF_on);
            break;

        case QR_CMD_HIGH_SPEED: {
            ESP_LOGI(TAG, "QR: HIGH SPEED (%s)", raw);
            float high_speed = interface_get_high_speed();
            interface_update_speed(high_speed);
            set_speed(motors, high_speed);
            break;
        }

        case QR_CMD_MED_SPEED: {
            ESP_LOGI(TAG, "QR: MED SPEED (%s)", raw);
            float med_speed = interface_get_med_speed();
            interface_update_speed(med_speed);
            set_speed(motors, med_speed);
            break;
        }

        case QR_CMD_LOW_SPEED: {
            ESP_LOGI(TAG, "QR: LOW SPEED (%s)", raw);
            float low_speed = interface_get_low_speed();
            interface_update_speed(low_speed);
            set_speed(motors, low_speed);
            break;
        }

        case QR_CMD_PICKUP:
            ESP_LOGI(TAG, "QR: PICKUP (%s)", raw);
            break;

        case QR_CMD_DROP_OFF:
            ESP_LOGI(TAG, "QR: DROP OFF (%s)", raw);
            break;

        case QR_CMD_PAUSE:
            ESP_LOGI(TAG, "QR: PAUSE (%s)", raw);
            break;

        case QR_CMD_DISABLE_TOF:
            ESP_LOGI(TAG, "QR: DISABLE TOF (%s)", raw);
            ToF_on=false;
            interface_update_TOF(ToF_on);
            break;

        case QR_CMD_ENABLE_TOF:
            ESP_LOGI(TAG, "QR: ENABLE TOF (%s)", raw);
            ToF_on=true;
            break;

        case QR_CMD_LINE_CHANGE_RIGHT:
            ESP_LOGI(TAG, "QR: LINE CHANGE RIGHT (%s)", raw);
            break;

        case QR_CMD_LINE_CHANGE_LEFT:
            ESP_LOGI(TAG, "QR: LINE CHANGE LEFT (%s)", raw);
            break;

        case QR_CMD_CHARGING:
            ESP_LOGI(TAG, "QR: CHARGING (%s)", raw);
            break;

        case QR_CMD_CHANGE_OF_SURFACE:
            ESP_LOGI(TAG, "QR: CHANGE OF SURFACE (%s)", raw);
            break;

        default:
            ESP_LOGW(TAG, "QR: UNKNOWN COMMAND code=%d (raw=%s)", code, raw);
            break;
    }
}


    
// Returns -1 if invalid, else 0..100
static int tcr_decode_numeric(const char *msg)
{
    if (!msg) return -1;

    while (isspace((unsigned char)*msg)) msg++;
    if (*msg == '\0') return -1;

    char *end = NULL;
    long v = strtol(msg, &end, 10);

    if (end == msg) return -1; // no digits parsed
    while (isspace((unsigned char)*end)) end++;
    if (*end != '\0') return -1; // non-numeric junk in string

    if (v < 0 || v > 100) return -1;
    return (int)v;
}


static uint16_t tcr_decode_message(const char *msg)
{
    if (!msg || !msg[0]) {
        return 0;
    }

    // You can change these strings to whatever is encoded in your QR codes
    if (strcmp(msg, "1") == 0) {
        return 1;
    } else if (strcmp(msg, "44") == 0) {
        return 44;
    } else if (strcmp(msg, "45") == 0) {
        return 45;
    } 
    ESP_LOGI("TAG","no match %s", msg);

    return 0;
}


static void tiny_code_reader_task(void *arg)
{
    tiny_code_reader_results_t res;

    ESP_LOGI(TAG, "Tiny Code Reader task started");
    motors = get_motors();
    while (1) {
        if (!tiny_code_reader_read(s_tcr_dev, &res)) {
            ESP_LOGW(TAG, "Failed to read from Tiny Code Reader");
        } else if (res.content_length > 0 && 1) {

            const char *msg = (const char *)res.content_bytes;

            // For debugging: see exactly what we parsed as a string
            ESP_LOGI(TAG, "TCR raw msg (len=%u): '%s'",
                     (unsigned)res.content_length, msg);

            int code = tcr_decode_numeric(msg);  // 0..100 or -1 if invalid

            if (code >= 0) {
                tcr_handle_code(code, msg);      // existing handler with switch
            } else {
                ESP_LOGW(TAG, "Non-numeric or out-of-range QR: '%s'", msg);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief Public init called from your app init
 */
esp_err_t tiny_code_reader_service_init(void)
{
    esp_err_t err;

    // 1) Init I2C bus (your existing i2c_bus.c)
    err = i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (!bus) {
        ESP_LOGE(TAG, "i2c_bus_get_handle returned NULL");
        return ESP_FAIL;
    }

    // 2) Add Tiny Code Reader as a device
    err = tiny_code_reader_init(bus, &s_tcr_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init Tiny Code Reader device: %s", esp_err_to_name(err));
        return err;
    }

    // Optional: turn sensor LED on so you can see it working
    tiny_code_reader_set_led(s_tcr_dev, true);

    // 3) Create background task
    if (s_tcr_task_handle == NULL) {
        BaseType_t ok = xTaskCreate(
            tiny_code_reader_task,
            "tiny_code_reader_task",
            4096,          // stack size
            NULL,
            5,             // priority, adjust for your system
            &s_tcr_task_handle
        );

        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create Tiny Code Reader task");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Tiny Code Reader service initialized");
    return ESP_OK;
}
