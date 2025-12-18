// i2c_bus.c
#include "i2c_bus.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "i2c_bus";
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_bus_config_t s_bus_cfg;

esp_err_t i2c_bus_init(void)
{
    if (s_i2c_bus) {
        return ESP_OK; // already init
    }

    i2c_master_bus_config_t bus_cfg =  {
        .i2c_port         = I2C_PORT,
        .sda_io_num       = I2C_SDA_PIN,
        .scl_io_num       = I2C_SCL_PIN,
        .clk_source       = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = false, // external 2.2k pull-ups
        },
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C bus: %s", esp_err_to_name(err));
        return err;
    }
    s_bus_cfg=bus_cfg;
    ESP_LOGI(TAG, "I2C bus initialised");
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_get_handle(void)
{
    return s_i2c_bus;
}
i2c_master_bus_config_t i2c_bus_get_cfg(void)
{
    return s_bus_cfg;
}
