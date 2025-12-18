// VL53L5CX.c


#include "ToF/tof_sensor.h"
#include "i2c_bus.h"
#include "motor_control/motor_control.h"
#include "esp_log.h"
#include <string.h>
#include "stdbool.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_detection_thresholds.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "esp_heap_caps.h"

#define TOF_REQUIRED_PULSES   2      // X pulses required
#define TOF_WINDOW_US         20000   // within 25 ms
#define TOF_SENSOR_COUNT 3
static const char *TAG = "tof_sensor";

static QueueHandle_t s_tof_int_queue = NULL;
typedef enum {
    TOF_SENSOR_CENTER = 0,
    TOF_SENSOR_LEFT   = 1,
    TOF_SENSOR_RIGHT  = 2,
} tof_sensor_t;

 
static uint16_t left_field_zones[16] = {
        700, 1000, 1000, 1000,
        670, 980, 980, 600,
        500, 715, 750, 400,
        400, 450, 450, 400
};
 
 
static uint16_t center_field_zones[16] = {
        450, 920, 920, 450,
        450, 900, 900, 450,
        400, 530, 530, 400,
        0, 0, 0, 0
};
 
 
static uint16_t right_field_zones[16] = {
        1000, 1000, 1000, 600,
        850, 960, 960, 500,
        500, 680, 610, 480,
        400, 430, 430, 400
};
 

static uint16_t zero_low_zone[16] = {
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0
};

volatile bool ToF_flag_center = false;
volatile bool ToF_flag_right = false;
volatile bool ToF_flag_left = false;
volatile bool ToF_on = false;

// Center: VL53L5CX
VL53L5CX_ResultsData    g_center_res;
static VL53L5CX_Configuration  g_default_dev;
static VL53L5CX_Configuration  g_center_dev;
static VL53L5CX_Configuration  g_left_dev;
static VL53L5CX_Configuration  g_right_dev;

static uint8_t isAlive;
static motors_t *g_motors;


// ------ ISR & GPIO handling ------------------------------------------------
static void tof_int_filter_task(void *arg)
{
    // Combined pulse count / window
    uint32_t pending_pulses_total   = 0;
    int64_t  window_start_us        = 0;   // 0 = no active window

    // Track which sensors have fired at least once in the current window
    bool sensor_seen[TOF_SENSOR_COUNT] = {0};

    // Track last time we saw ANY pulse
    int64_t last_activity_us = 0;
    const int64_t CLEAR_TIMEOUT_US = 2000000;  // 2 s

    // How often we wake up to check for timeout (must be < CLEAR_TIMEOUT_US)
    const TickType_t QUEUE_WAIT_TICKS = pdMS_TO_TICKS(200); // 200 ms
    bool last_loop_on=false;
    for (;;) {
        tof_sensor_t sensor;
        int64_t now_us = esp_timer_get_time();

        // Wait for ANY ISR event, but with timeout
        if (xQueueReceive(s_tof_int_queue, &sensor, QUEUE_WAIT_TICKS) != pdTRUE) {
            // No new pulse within QUEUE_WAIT_TICKS
            now_us = esp_timer_get_time();

            if (last_activity_us != 0 &&
                (now_us - last_activity_us) >= CLEAR_TIMEOUT_US)
            {
                // 2 seconds without activity: clear flags once
                ToF_flag_center = false;
                ToF_flag_left   = false;
                ToF_flag_right  = false;

                // Reset window and activity marker
                window_start_us      = 0;
                pending_pulses_total = 0;
                for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
                    sensor_seen[i] = false;
                }
                last_activity_us = 0;
                motors_resume(g_motors);
            }
            continue;
        }

        // We got a pulse
        now_us = esp_timer_get_time();
        last_activity_us = now_us;

        int idx = (int)sensor;

        // Start new window if none active or window expired
        if (window_start_us == 0 ||
            (now_us - window_start_us) > TOF_WINDOW_US)
        {
            window_start_us      = now_us;
            pending_pulses_total = 1;

            // reset which sensors we’ve seen in this new window
            for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
                sensor_seen[i] = false;
            }
            sensor_seen[idx] = true;
        }
        else
        {
            // Still inside the window
            pending_pulses_total++;
            sensor_seen[idx] = true;
        }

        // Check combined pulses condition
        if (pending_pulses_total >= TOF_REQUIRED_PULSES &&
            (now_us - window_start_us) <= TOF_WINDOW_US)
        {
            // Trigger flags for all sensors that participated
            if (sensor_seen[TOF_SENSOR_CENTER]) {
                ToF_flag_center = true;
                //ESP_LOGI("TOF_INT", "Center triggered (part of %u pulses in %lld us)", pending_pulses_total, (long long)(now_us - window_start_us));
            }
            if (sensor_seen[TOF_SENSOR_LEFT]) {
                ToF_flag_left = true;
                //ESP_LOGI("TOF_INT", "Left triggered (part of %u pulses in %lld us)", pending_pulses_total, (long long)(now_us - window_start_us));
            }
            if (sensor_seen[TOF_SENSOR_RIGHT]) {
                ToF_flag_right = true;
                //ESP_LOGI("TOF_INT", "Right triggered (part of %u pulses in %lld us)", pending_pulses_total, (long long)(now_us - window_start_us));
            }
            if ((ToF_flag_center || ToF_flag_left || ToF_flag_right) && ToF_on){
                last_loop_on=true;
                motors_stop(g_motors);
            } else if (ToF_on){
                motors_resume(g_motors);
                last_loop_on=false;
                motors_resume(g_motors);
            }

            // Reset window for next detection
            window_start_us      = 0;
            pending_pulses_total = 0;
            for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
                sensor_seen[i] = false;
            }
        }
    }
}

void activate_ToF(bool active){

}

static void IRAM_ATTR vl53l5cx_center_isr(void *arg)
{
    tof_sensor_t id = TOF_SENSOR_CENTER;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_tof_int_queue) {
        xQueueSendFromISR(s_tof_int_queue, &id, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR vl53l5cx_left_isr(void *arg)
{
    tof_sensor_t id = TOF_SENSOR_LEFT;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_tof_int_queue) {
        xQueueSendFromISR(s_tof_int_queue, &id, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR vl53l5cx_right_isr(void *arg)
{
    tof_sensor_t id = TOF_SENSOR_RIGHT;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_tof_int_queue) {
        xQueueSendFromISR(s_tof_int_queue, &id, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t vl53l5cx_int_gpio_init(void)
{
    // 1) Create the queue
    if (s_tof_int_queue == NULL) {
        s_tof_int_queue = xQueueCreate(32, sizeof(tof_sensor_t));
        if (s_tof_int_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create ToF INT queue");
            return ESP_ERR_NO_MEM;
        }
    }

    // 2) Configure INT GPIOs
    gpio_config_t io_conf = {
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,   // external pull-ups if present
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,     // INT is active-low
    };

    esp_err_t err;

    // Center INT
    io_conf.pin_bit_mask = 1ULL << TOF_INT_CENTER_GPIO;
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config center failed: %s", esp_err_to_name(err));
        return err;
    }

    // Left INT
    io_conf.pin_bit_mask = 1ULL << TOF_INT_LEFT_GPIO;
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config left failed: %s", esp_err_to_name(err));
        return err;
    }

    // Right INT
    io_conf.pin_bit_mask = 1ULL << TOF_INT_RIGHT_GPIO;
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config right failed: %s", esp_err_to_name(err));
        return err;
    }

    // 3) Optional: sanity-check heap before touching interrupt allocator
    heap_caps_check_integrity_all(true);

    // 4) Install ISR service if not already installed
    //    Using IRAM flag is generally recommended for GPIO ISRs.
    /*err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE just means it was already installed
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return err;
    }*/

    // 5) Register handlers
    err = gpio_isr_handler_add(TOF_INT_CENTER_GPIO, vl53l5cx_center_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add center failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(TOF_INT_LEFT_GPIO, vl53l5cx_left_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add left failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(TOF_INT_RIGHT_GPIO, vl53l5cx_right_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add right failed: %s", esp_err_to_name(err));
        return err;
    }

    // 6) Only after interrupts are set up, start the filter task
    static TaskHandle_t tof_task_handle = NULL;
    if (tof_task_handle == NULL) {
        BaseType_t ok = xTaskCreate(
            tof_int_filter_task,
            "tof_int_filter",
            4096,
            NULL,
            5,
            &tof_task_handle
        );
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create tof_int_filter_task");
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

void init_lpn_pins(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LEFT_LPn_GPIO) | (1ULL << RIGHT_LPn_GPIO) | (1ULL << CENTER_LPn_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    // start with both sensors held in reset
    gpio_set_level(LEFT_LPn_GPIO, 0);
    gpio_set_level(RIGHT_LPn_GPIO, 0);
    gpio_set_level(CENTER_LPn_GPIO, 0);
}

bool is_valid_target_status(uint8_t st)
{
    // ST: 5 = valid, 9 = valid under some conditions.
    // You can widen this later if needed.
    return (st == 5 || st == 9);
}

esp_err_t init_ToF_sensor(motors_t *motors)
{
    g_motors=motors;
    ESP_LOGI(TAG, "Initialising sensors...");
    init_lpn_pins();

    vTaskDelay(pdMS_TO_TICKS(10));   // or 10 ms to be safe

    ESP_LOGI(TAG, "Initialising I2C bus");
    ESP_ERROR_CHECK(i2c_bus_init());
    vTaskDelay(pdMS_TO_TICKS(10));   // or 10 ms to be safe
    i2c_master_bus_handle_t bus = i2c_bus_get_handle();

    if (!bus) {
        ESP_LOGE(TAG, "i2c_bus_get_handle() returned NULL");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Initialising center sensor...");
    
    memset(&g_default_dev, 0, sizeof(g_default_dev));
    memset(&g_center_dev, 0, sizeof(g_center_dev));
    memset(&g_right_dev, 0, sizeof(g_right_dev));
    memset(&g_left_dev, 0, sizeof(g_left_dev));

    g_default_dev.platform.address   = VL53L5CX_DEFAULT_I2C_ADDRESS;
    g_center_dev.platform.address   = TOF_CENTER_ADDR_8BIT;
    g_right_dev.platform.address   = TOF_RIGHT_ADDR_8BIT;
    g_left_dev.platform.address   = TOF_LEFT_ADDR_8BIT;
    

    i2c_device_config_t cfg_default = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = (VL53L5CX_DEFAULT_I2C_ADDRESS >> 1), // convert 8-bit to 7-bit
        .scl_speed_hz    = VL53L5CX_MAX_CLK_SPEED,
    };

    i2c_device_config_t cfg_center = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = (TOF_CENTER_ADDR_8BIT >> 1), // convert 8-bit to 7-bit
        .scl_speed_hz    = VL53L5CX_MAX_CLK_SPEED,
    };

    i2c_device_config_t cfg_right = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = (TOF_RIGHT_ADDR_8BIT >> 1), // convert 8-bit to 7-bit
        .scl_speed_hz    = VL53L5CX_MAX_CLK_SPEED,
    };

    i2c_device_config_t cfg_left = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = (TOF_LEFT_ADDR_8BIT >> 1), // convert 8-bit to 7-bit
        .scl_speed_hz    = VL53L5CX_MAX_CLK_SPEED,
    };
    
    i2c_master_bus_config_t bus_cfg = i2c_bus_get_cfg();
    g_center_dev.platform.bus_config = bus_cfg;
    g_right_dev.platform.bus_config = bus_cfg;
    g_left_dev.platform.bus_config = bus_cfg;
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg_default, &g_default_dev.platform.handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg_center, &g_center_dev.platform.handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg_right, &g_right_dev.platform.handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg_left, &g_left_dev.platform.handle));
    
    ESP_LOGI(TAG, "Bus device added center sensor");
    // give it time to boot

    gpio_set_level(CENTER_LPn_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));   // or 10 ms to be safe
    uint8_t status_cen = 0;
    
    status_cen = vl53l5cx_is_alive(&g_default_dev, &isAlive);
	if(!isAlive || status_cen)
	{
		ESP_LOGI(TAG,"VL53L5CX not detected at default requested address\n");
		status_cen = vl53l5cx_is_alive(&g_center_dev, &isAlive);
        if(!isAlive || status_cen)
        {
            ESP_LOGE(TAG,"VL53L5CX not detected at center address\n");
            return ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "vl53l5cx_is_alive status_cen = %u (0x%02X)", status_cen, status_cen);
        }

	} else {
        ESP_LOGI(TAG, "vl53l5cx_is_alive status_cen = %u (0x%02X)", status_cen, status_cen);
        
        status_cen = vl53l5cx_init(&g_default_dev);
        ESP_LOGI(TAG, "vl53l5cx_init status_cen = %u (0x%02X)", status_cen, status_cen);
        if (status_cen) {
            ESP_LOGE(TAG, "VL53L5CX init failed: %u", status_cen);
            return ESP_FAIL;
        }  

        status_cen = vl53l5cx_set_i2c_address(&g_default_dev, TOF_CENTER_ADDR_8BIT);
        ESP_LOGI(TAG, "vl53l5cx_set_i2c_address status_cen = %u (0x%02X)", status_cen, status_cen);
        if (status_cen) {
            ESP_LOGE(TAG, "VL53L5CX set i2c address failed: %u", status_cen);
            return ESP_FAIL;
        }
    }

    status_cen = vl53l5cx_init(&g_center_dev);
    ESP_LOGI(TAG, "vl53l5cx_init status_cen = %u (0x%02X)", status_cen, status_cen);
    if (status_cen) {
        ESP_LOGE(TAG, "VL53L5CX init failed: %u", status_cen);
        return ESP_FAIL;
    }  

    // Put center in 4x4 mode (easier to reason about)
    status_cen = vl53l5cx_set_resolution(&g_center_dev, VL53L5CX_RESOLUTION_4X4);
    if (status_cen) {
        ESP_LOGE(TAG, "VL53L5CX set_resolution failed: %u", status_cen);
        return ESP_FAIL;
    }

    status_cen = vl53l5cx_set_ranging_frequency_hz(&g_center_dev, 60);   
    if (status_cen) {
        ESP_LOGE(TAG, "set_frequency failed: %u", status_cen);
        return ESP_FAIL;
    }
    status_cen = vl53l5cx_set_zone_dual_thresholds(&g_center_dev, zero_low_zone, center_field_zones, 12);
    if (status_cen) {
        ESP_LOGE(TAG, "set_thresholds failed: %u", status_cen);
        return ESP_FAIL;
    }

    status_cen = vl53l5cx_start_ranging(&g_center_dev);
    if (status_cen) {
        ESP_LOGE(TAG, "VL53L5CX start_ranging failed: %u", status_cen);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "VL53L5CX center ready");

    //right sensor
    gpio_set_level(RIGHT_LPn_GPIO, 1);  
    
    ESP_LOGI(TAG, "Initialising right sensor...");

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t status_right = 0;
    isAlive = 0;
    status_right = vl53l5cx_is_alive(&g_default_dev, &isAlive);
	if(!isAlive || status_right)
	{
		ESP_LOGE(TAG,"VL53L5CX not detected at default requested address\n");
		status_right = vl53l5cx_is_alive(&g_right_dev, &isAlive);
        if(!isAlive || status_right)
        {
            ESP_LOGE(TAG,"VL53L5CX not detected at right address\n");
            return ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "vl53l5cx_is_alive status_cen = %u (0x%02X)", status_right, status_right);
        }
	} else {
        ESP_LOGI(TAG, "vl53l5cx_is_alive status_cen = %u (0x%02X)", status_right, status_right);
        
        status_right = vl53l5cx_init(&g_default_dev);
        if (status_right) {
            ESP_LOGE(TAG, "VL53L5CX init failed: %u", status_right);
            return ESP_FAIL;
        }
        
        status_right = vl53l5cx_set_i2c_address(&g_default_dev, TOF_RIGHT_ADDR_8BIT);
        if (status_right) {
            ESP_LOGE(TAG, "VL53L5CX set i2c address failed: %u", status_right);
            return ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "vl53l5cx_set_i2c_address status_right = %u (0x%02X)", status_right, status_right);
        }
    }

    status_right = vl53l5cx_init(&g_right_dev);
    if (status_right) {
        ESP_LOGE(TAG, "VL53L5CX init failed: %u", status_right);
        return ESP_FAIL;
    
    }
    // Put center in 4x4 mode (easier to reason about)
    status_right = vl53l5cx_set_resolution(&g_right_dev, VL53L5CX_RESOLUTION_4X4);
    if (status_right) {
        ESP_LOGE(TAG, "VL53L5CX set_resolution failed: %u", status_right);
        return ESP_FAIL;
    }

    status_right = vl53l5cx_set_ranging_frequency_hz(&g_right_dev, 60);   // 10 Hz
    if (status_right) {
        ESP_LOGE(TAG, "set_frequency failed: %u", status_right);
        return ESP_FAIL;
    }
    status_right = vl53l5cx_set_zone_dual_thresholds(&g_right_dev, zero_low_zone, right_field_zones, 15);
    if (status_right) {
        ESP_LOGE(TAG, "set_thresholds failed: %u", status_right);
        return ESP_FAIL;
    }
    
    status_right = vl53l5cx_start_ranging(&g_right_dev);
    if (status_right) {
        ESP_LOGE(TAG, "VL53L5CX start_ranging failed: %u", status_right);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ToF right ready");
    
    //Left
    gpio_set_level(LEFT_LPn_GPIO, 1);
    
    ESP_LOGI(TAG, "Initialising left sensor...");
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t status_left = 0;
    status_left = vl53l5cx_is_alive(&g_default_dev, &isAlive);
	if(!isAlive || status_left)
	{
		ESP_LOGI(TAG,"VL53L5CX not detected at default requested address\n");
		status_left = vl53l5cx_is_alive(&g_left_dev, &isAlive);
        if(!isAlive || status_left)
        {
            ESP_LOGE(TAG,"VL53L5CX not detected at left address\n");
            return ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "vl53l5cx_is_alive status_cen = %u (0x%02X)", status_left, status_left);
        }
	} else {
        ESP_LOGI(TAG, "vl53l5cx_is_alive status_left = %u (0x%02X)", status_left, status_left); 
        status_left = vl53l5cx_init(&g_default_dev);
        if (status_left) {
            ESP_LOGE(TAG, "VL53L5CX init failed: %u", status_left);
            return ESP_FAIL;
        }
        
        status_left = vl53l5cx_set_i2c_address(&g_default_dev, TOF_LEFT_ADDR_8BIT);
        if (status_left) {
            ESP_LOGE(TAG, "VL53L5CX set i2c address failed: %u", status_left);
            return ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "vl53l5cx_set_i2c_address status_cen = %u (0x%02X)", status_left, status_left);
        }

    }
    
   
    status_left = vl53l5cx_init(&g_left_dev);
    if (status_left) {
        ESP_LOGE(TAG, "VL53L5CX init failed: %u", status_left);
        return ESP_FAIL;
    }
    
    // Put center in 4x4 mode (easier to reason about)
    status_left = vl53l5cx_set_resolution(&g_left_dev, VL53L5CX_RESOLUTION_4X4);
    if (status_left) {
        ESP_LOGE(TAG, "VL53L5CX set_resolution failed: %u", status_left);
        return ESP_FAIL;
    }

    status_left = vl53l5cx_set_ranging_frequency_hz(&g_left_dev, 60);   
    if (status_left) {
        ESP_LOGE(TAG, "set_frequency failed: %u", status_left);
        return ESP_FAIL;
    }

    status_left = vl53l5cx_set_zone_dual_thresholds(&g_left_dev, zero_low_zone, left_field_zones, 16);
    if (status_left) {
        ESP_LOGE(TAG, "set_thresholds failed: %u", status_left);
        return ESP_FAIL;
    }

    status_left = vl53l5cx_start_ranging(&g_left_dev);
    if (status_left) {
        ESP_LOGE(TAG, "VL53L5CX start_ranging failed: %u", status_left);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "VL53L5CX left ready");

    ESP_LOGI(TAG, "Initialising GPIO Pins sensor...");
    ESP_ERROR_CHECK(vl53l5cx_int_gpio_init());
    
    return ESP_OK;
}


esp_err_t vl53l5cx_read_all_zones(void)
{

    uint8_t data_ready = 0;
    uint8_t status = vl53l5cx_check_data_ready(&g_center_dev, &data_ready);
    if (status != 0 || !data_ready) {
        // no new data yet
        return ESP_ERR_INVALID_STATE;
    }

    status = vl53l5cx_get_ranging_data(&g_center_dev, &g_center_res);
    if (status != 0) {
        ESP_LOGW(TAG, "vl53l5cx_get_ranging_data failed: %u", status);
        return ESP_FAIL;
    }

     uint8_t resolution = 0;
    vl53l5cx_get_resolution(&g_center_dev, &resolution);
    if (resolution != VL53L5CX_RESOLUTION_4X4) {
        ESP_LOGW(TAG, "Center not in 4x4 mode (res=%u), forcing nb_zones=16 anyway", resolution);
    }
    const uint8_t nb_zones = 16;

    int16_t min_dist = INT16_MAX;
    int16_t min_zone = -1;

    for (uint8_t zone = 0; zone < nb_zones; ++zone) {
        int16_t d = -1;

#ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
        if (g_center_res.nb_target_detected[zone] > 0) {
            uint32_t idx = zone * VL53L5CX_NB_TARGET_PER_ZONE; // first target in zone

            uint8_t st = g_center_res.target_status[idx];
            if (is_valid_target_status(st)) {
                d = (int16_t)g_center_res.distance_mm[idx];
            }
        }
#endif

        ESP_LOGI("DETC", "zone %2u    distance %6d", zone, d);

        if (d > 0 && d < min_dist) {
            min_dist = d;
            min_zone = zone;
        }

        if (zone == (nb_zones - 1)) {
            if (min_dist == INT16_MAX) {
                ESP_LOGI("print", "min zone: none   min distance: -1 (no valid targets)");
            } else {
                ESP_LOGI("print", "min zone %2d   min distance %6d", min_zone, min_dist);
            }
        }
    }

    return ESP_OK;
}




esp_err_t vl53l5cx_set_zone_dual_thresholds(VL53L5CX_Configuration *dev, const uint16_t *low_mm, 
        const uint16_t *high_mm, uint8_t nb_zones)
{
    if (!dev || !high_mm) {
        return ESP_ERR_INVALID_ARG;
    }
    /*if (nb_zones != 12 && nb_zones != 64) {
        return ESP_ERR_INVALID_SIZE;
    }*/

    VL53L5CX_DetectionThresholds thresholds[64];  // max 8x8
    memset(thresholds, 0, sizeof(thresholds));

    for (uint8_t i = 0; i < nb_zones; ++i) {
        thresholds[i].zone_num             = i;
        thresholds[i].measurement          = VL53L5CX_DISTANCE_MM;
        thresholds[i].type                 = VL53L5CX_IN_WINDOW;      // fire if in [low, high]
        thresholds[i].mathematic_operation = VL53L5CX_OPERATION_NONE;

        thresholds[i].param_low_thresh  = low_mm[i];   // lower bound in mm
        thresholds[i].param_high_thresh = high_mm[i];  // upper bound in mm
    }

    // Mark last threshold entry
    thresholds[nb_zones - 1].zone_num |= VL53L5CX_LAST_THRESHOLD;

    uint8_t status = 0;

    status = vl53l5cx_set_detection_thresholds(dev, thresholds);
    if (status != 0) {
        ESP_LOGE(TAG, "set_detection_thresholds failed: %u", status);
        return ESP_FAIL;
    }

    status = vl53l5cx_set_detection_thresholds_enable(dev, 1);
    if (status != 0) {
        ESP_LOGE(TAG, "enable thresholds failed: %u", status);
        return ESP_FAIL;
    }


    ESP_LOGI(TAG, "zone dual thresholds configured (nb_zones=%u)", nb_zones);
    return ESP_OK;
}
