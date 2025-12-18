
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>

#include "motor_control/motor_control.h"
#include "motor_control/encoder.h"
#include "pins.h"

#define TAG "ENCODER"

static gptimer_handle_t encoder_timer = NULL;

// Forward declaration
static void encoder_isr_handler(void *arg);
void encoder_task(void *arg);


// -------------------------
// ISR
// -------------------------
static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    motor_control_t *motor = (motor_control_t *)arg;

    uint64_t now;
    gptimer_get_raw_count(encoder_timer, &now);

    uint64_t diff = now - motor->encoder.last_tick;

    if (diff > NOISE_FILTER_TICKS) {
        portENTER_CRITICAL_ISR(&motor->encoder.lock);
        motor->encoder.pulse_count++;
        motor->encoder.dt_ticks_sum += diff;
        motor->encoder.last_tick = now;
        portEXIT_CRITICAL_ISR(&motor->encoder.lock);

        if (motor->encoder.task_handle) {
            BaseType_t hp = pdFALSE;
            vTaskNotifyGiveFromISR(motor->encoder.task_handle, &hp);
            portYIELD_FROM_ISR(hp);
        }
    }
}

// -------------------------
// Initialization
// -------------------------
void encoder_init(motor_control_t *motor_right, motor_control_t *motor_left)
{
    // Create tasks per motor
    xTaskCreate(encoder_task, "RightEncoderTask", 4096, motor_right, 10, &motor_right->encoder.task_handle);
    xTaskCreate(encoder_task, "LeftEncoderTask",  4096, motor_left,  10, &motor_left->encoder.task_handle);
    motor_right->encoder.lock = (portMUX_TYPE) portMUX_INITIALIZER_UNLOCKED;
    motor_left->encoder.lock  = (portMUX_TYPE) portMUX_INITIALIZER_UNLOCKED;
    // Configure GPTimer
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &encoder_timer));
    ESP_ERROR_CHECK(gptimer_enable(encoder_timer));
    ESP_ERROR_CHECK(gptimer_start(encoder_timer));

    // Assign IRQ pins
    motor_right->encoder.irq_pin = ENCODER_IRQ_PIN_RIGHT;
    motor_left->encoder.irq_pin  = ENCODER_IRQ_PIN_LEFT;

    motor_right->encoder.pulse_count = 0; // init to zero
    motor_left->encoder.pulse_count = 0;

    memset(&motor_right->encoder.speed_ma, 0, sizeof(speed_ma_t)); //init to zero
    memset(&motor_left->encoder.speed_ma, 0, sizeof(speed_ma_t));

    // Configure GPIOs
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << motor_right->encoder.irq_pin) | (1ULL << motor_left->encoder.irq_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // ISR
    gpio_isr_handler_add(motor_right->encoder.irq_pin, encoder_isr_handler, motor_right);
    gpio_isr_handler_add(motor_left->encoder.irq_pin,  encoder_isr_handler, motor_left);


    ESP_LOGI(TAG, "Encoders initialized using GPTimer @ %llu Hz", TIMER_RES_HZ);
}

float weighted_ma(speed_ma_t *ma, float new_value) {
    ma->history[ma->index] = new_value;
    ma->index = (ma->index + 1) % SPEED_MA_LEN;
    if (ma->index == 0) ma->filled = true;

    int count = ma->filled ? SPEED_MA_LEN : ma->index;
    float wsum = 0.0f;
    float wtotal = 0.0f;

    for (int i = 0; i < count; i++) {
        float w = (float)(i + 1);   // older -> smaller weight
        wsum += ma->history[(ma->index + SPEED_MA_LEN - 1 - i) % SPEED_MA_LEN] * w;
        wtotal += w;
    }
    return wsum / wtotal;
}

void encoder_task(void *arg)
{
    motor_control_t *motor = (motor_control_t *)arg;
    
    const float cutoff_freq_hz = ENCODER_FREQ_CUTOFF;
    const float omega_c = 2.0f * 3.1415926f * cutoff_freq_hz;
    
    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TIMEOUT_MS));
        
        int32_t pulses;
        uint64_t dt_ticks;
        
        portENTER_CRITICAL(&motor->encoder.lock);
        pulses = motor->encoder.pulse_count;
        dt_ticks = motor->encoder.dt_ticks_sum;
        motor->encoder.pulse_count = 0;
        motor->encoder.dt_ticks_sum = 0;
        portEXIT_CRITICAL(&motor->encoder.lock);
        
        if (pulses > 0) {

            float dt_sec_avg = ((float)dt_ticks / pulses) / TIMER_RES_HZ;

            // Raw instantaneous speed from encoder
            float raw_speed = DEGREES_PER_PULSE / dt_sec_avg;

            // Weighted moving average
            float smooth_raw = weighted_ma(&motor->encoder.speed_ma, raw_speed);

            // Low-pass filter
            float alpha = 1.0f - expf(-omega_c * dt_sec_avg);
            float filtered_speed = alpha * smooth_raw + (1.0f - alpha) * motor->current.speed;

            // Acceleration limit
            float dv_max = MAX_ACCEL_DPS2 * dt_sec_avg;
            float dv = filtered_speed - motor->current.speed;

            if (dv > dv_max) dv = dv_max;
            if (dv < -dv_max) dv = -dv_max;
            
            motor->current.speed += dv;

        } else {
            motor->current.speed = 0.0f;
        }

        motor->current.speed_mps = DPS_TO_MPS(motor->current.speed);
    }
}