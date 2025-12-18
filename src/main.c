#include <stdio.h>
#include "esp_log.h" //ESP_LOG
#include "nvs_flash.h" //flash
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Always after "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"

#include "motor_control/line_follower.h"
#include "motor_control/motor_control.h"
#include "battery.h"
#include "sd_card.h"
#include "ToF/tof_sensor.h"
#include "QR/tiny_code_reader_service.h"
#include "agv.h"
#include "package_sensor.h"
#include "wifi_interface/Interface_updates.h"
#include "wifi_interface/mqtt_lib.h"
#include "wifi_interface/wifi_lib.h"
#include "i2c_bus.h"


static const char *TAG = "MAIN";

static agv_t agv; // contains battery and motors


//initializing the flash so you can write to the microcontroller
void init_flash(void)
{
  esp_err_t ret = nvs_flash_init();
  //handles error 
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void app_main(void) {
    init_flash();

    init_sdcard();

    app_config_t cfg = {0};
 
    if (sd_read_app_config_json(&cfg)) {
        ESP_LOGI(TAG, "SSID=%s, broker=%s, password=%s",
                cfg.wifi.ssid, cfg.mqtt.broker, cfg.wifi.password);

    } else {
        ESP_LOGE(TAG, "Failed to load config from SD");
    }
 
    ESP_ERROR_CHECK(wifi_lib_init());
    ESP_LOGI(TAG, "Wi-Fi good");
    vTaskDelay(pdMS_TO_TICKS(1000));

    wifi_static_ip_t sip = {.enable=false};
    if (wifi_lib_connect(cfg.wifi.ssid, cfg.wifi.password, &sip) != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi failed");
        return;
    } else ESP_LOGI(TAG, "Wi-Fi good");
    
    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // install global isr service
    
    motors_init(&agv.motors);

    mqtt_start_from_cfg(&cfg);
    init_interface(cfg.device.name);

    package_sensor_init(&agv);
  
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(init_ToF_sensor(&agv.motors));
    ESP_ERROR_CHECK(tiny_code_reader_service_init());
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_ERROR_CHECK(battery_init(i2c_bus_get_handle(), &agv.battery_data)); 
    
    line_follower_init(&agv.motors);
    set_speed(&agv.motors, interface_get_low_speed());
    interface_update_speed(interface_get_low_speed());
 
    while (1) { 
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}