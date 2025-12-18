#include "driver/gpio.h"
#include "motor_control/motor_control.h"
#include "emergency.h"
#include "esp_log.h"

#include "pins.h"

static const char *TAG = "EMERGENCY";

static void IRAM_ATTR emergency_btn_isr_handler(void *arg) { 
    mosfet_set_state(RELAY_OFF);
    emergency_flag= true;
    g_relay_state = RELAY_OFF;
} 

static void IRAM_ATTR emergency_rst_isr_handler(void *arg) {
    emergency_flag = false;
}

void emergency_init(void) {

    ESP_LOGI(TAG, "Initializing emergency system...");

    // Configure MOSFET GPIO
    gpio_config_t mosfet = {
        .pin_bit_mask = (1ULL << EMERGENCY_MOSFET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&mosfet));

    // Configure emergency input and reset button GPIOs
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << EMERGENCY_SIGNAL_PIN) | (1ULL << EMERGENCY_RESET_BTN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    ESP_ERROR_CHECK(gpio_config(&io));
 
    // ISR service
    gpio_isr_handler_add(EMERGENCY_SIGNAL_PIN, emergency_btn_isr_handler, NULL);
    gpio_isr_handler_add(EMERGENCY_RESET_BTN_PIN, emergency_rst_isr_handler, NULL);
}
