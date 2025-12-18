#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "agv.h"
#include "pins.h"
#include "package_sensor.h"



static const char *TAG = "PACKAGE";

TaskHandle_t package_monitor_task_handle = NULL;  // Task handle for the package monitor task

bool ready_for_task_g = false;


static void package_sensor_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PACKAGE_SENSOR_PIN,
        .mode = GPIO_MODE_INPUT,
#ifdef PIN_LOW_WITH_PACKAGE
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
#else
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
#endif
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
}


static void package_monitor_task(void *arg) {
    agv_t *agv = (agv_t *)arg;

        while (!ready_for_task_g) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    bool print_flag = true;
    while (1) {
        #ifdef PIN_LOW_WITH_PACKAGE 
        if (gpio_get_level(PACKAGE_SENSOR_PIN)) 
        #else
        if (!gpio_get_level(PACKAGE_SENSOR_PIN))
        #endif
        {
            motors_stop(&agv->motors); // brake
            if(print_flag) {
                ESP_LOGI(TAG, "Package fell off");
                print_flag = false;
            }
           
        } else
            motors_resume(&agv->motors); // no brakey
            vTaskDelay(pdMS_TO_TICKS(PACKAGE_TASK_DELAY_MS));  
    }
}


// Initialize the package sensor task
void package_sensor_init(agv_t *agv) {
    package_sensor_gpio_init();
    xTaskCreate(package_monitor_task, "package_monitor_task", 2048*2, agv, 3, &package_monitor_task_handle);
    vTaskSuspend(package_monitor_task_handle);  // Initially suspend the task
    ready_for_task_g = true;
}
