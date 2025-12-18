#include "QR/tiny_code_reader.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <string.h>

static const char *TAG = "tiny_code_reader";

#define TCR_I2C_FREQ_HZ     100000
#define TCR_I2C_TIMEOUT_MS  100

esp_err_t tiny_code_reader_init(i2c_master_bus_handle_t bus,
                                i2c_master_dev_handle_t *out_dev)
{
    if (!bus || !out_dev) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Config may be named i2c_device_config_t or similar; adjust if your IDF differs.
    i2c_device_config_t dev_cfg = {
        .device_address = TINY_CODE_READER_I2C_ADDR,
        .scl_speed_hz   = TCR_I2C_FREQ_HZ,
        .flags = {
            .disable_ack_check = false,
        },
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Tiny Code Reader device: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Tiny Code Reader device added on I2C");
    return ESP_OK;
}


bool tiny_code_reader_read(i2c_master_dev_handle_t dev,
                           tiny_code_reader_results_t *out_results)
{
    if (!dev || !out_results) {
        return false;
    }

    uint8_t reg = 0x00;
    uint8_t buf[TINY_CODE_READER_RESULT_BYTES];

    esp_err_t err = i2c_master_transmit_receive(
        dev,
        &reg, 1,
        buf, sizeof(buf),
        TCR_I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_transmit_receive failed: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t len = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if (len > TINY_CODE_READER_MAX_CONTENT_LEN) {
        len = TINY_CODE_READER_MAX_CONTENT_LEN;
    }

    out_results->content_length = len;
    memcpy(out_results->content_bytes, &buf[2], TINY_CODE_READER_MAX_CONTENT_LEN);

    // make it safe to print
    if (len < TINY_CODE_READER_MAX_CONTENT_LEN) {
        out_results->content_bytes[len] = 0;
    } else {
        out_results->content_bytes[TINY_CODE_READER_MAX_CONTENT_LEN - 1] = 0;
    }

    return true;
}

bool tiny_code_reader_set_led(i2c_master_dev_handle_t dev, bool enable)
{
    if (!dev) return false;

    uint8_t data[2];
    data[0] = 0x01;                // LED control register
    data[1] = enable ? 0x01 : 0x00;

    esp_err_t err = i2c_master_transmit(
        dev,
        data, sizeof(data),
        TCR_I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LED: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}