#include <stdio.h>
#include "esp_log.h" //ESP_LOG
#include "nvs_flash.h" //flash
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Always after "freertos/FreeRTOS.h"
#include "math.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include <stdbool.h>
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "motor_control/motor_control.h"
#include "motor_control/line_follower.h"
#include "pins.h"

static const char *TAG = "LINE_FOLLOWER";


//Map our 7 sensors to ADC1 channels (GPIO1–GPIO7 on ESP32-S3).
static const adc_channel_t adc_channels[NUM_SENSORS] = {
    ADC_CHANNEL_2,  // sensor 0 -> GPIO3
    ADC_CHANNEL_3,  // sensor 1 -> GPIO4
    ADC_CHANNEL_4,  // sensor 2 -> GPIO5
    ADC_CHANNEL_5,  // sensor 3 -> GPIO6
    ADC_CHANNEL_6,  // sensor 4 -> GPIO7
    ADC_CHANNEL_7,  // sensor 5 -> GPIO8
    ADC_CHANNEL_8   // sensor 6 -> GPIO9
};

static int cal_min[NUM_SENSORS];
static int cal_max[NUM_SENSORS];
int normalized_reading[NUM_SENSORS];

//switching the line-ground contrast
bool black_line_on_white = true; 
// true  = black line on white background  (default)
// false = white line on dark background   (normalized inverted)

//Global array holding the readings.
int line_reading[NUM_SENSORS];

typedef struct {
    int cal_min[NUM_SENSORS];
    int cal_max[NUM_SENSORS];
} calib_data_t;

static void pid_init(line_follower_pid_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

// Call this once at startup
static void init_calibration()
{
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        cal_min[i] = 4095;   // high initial value
        cal_max[i] = 0;      // low initial value
    }

}

// This function performs calibration + normalization
// raw[]  = your measured ADC values (0–4095)
// norm[] = output normalized values (0–1000)
static void calibrate_and_normalize(int raw[], int norm[])
{
    // 1. Update min/max based on current readings
    for (int i = 0; i < NUM_SENSORS; i++) {

        int v = raw[i];

        if (v < cal_min[i]) cal_min[i] = v;
        if (v > cal_max[i]) cal_max[i] = v;
    }

    // 2. Normalize each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {

        int v = raw[i];
        int minv = cal_min[i];
        int maxv = cal_max[i];

        // safety: avoid divide-by-zero if min == max
        if (maxv <= minv) {
            norm[i] = 0;
            continue;
        }

        // map raw → 0..1000
        int value = (v - minv) * NORM_MAX / (maxv - minv);

        // clamp
        if (value < 0) value = 0;
        if (value > NORM_MAX) value = NORM_MAX;

        // mode switch:
        // black line on white:   value stays as is (0 = white, 1000 = black)
        // white line on dark:    invert it (0 = dark, 1000 = white line)
        if (!black_line_on_white) {
            value = NORM_MAX - value;
        }

        norm[i] = value;
    }
}
//Simple init: 12-bit width + 11dB attenuation (~0–3.3 V range)
adc_oneshot_unit_handle_t adc_handle;

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    for (int i = 0; i < NUM_SENSORS; i++) {
        adc_oneshot_config_channel(adc_handle, adc_channels[i], &chan_cfg);
    }
}


// Read all 7 sensors into line_reading[]
static void read_line_sensors(void)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(adc_handle, adc_channels[i], &raw);
        if (err == ESP_OK) {
            line_reading[i] = raw;      // real ADC value 0–4095
        } else {
            line_reading[i] = -1;       // indicate error (optional)
        }
    }
}

static float compute_line_position(const int normalized[])
{
    long weighted_sum = 0;
    long sum = 0;

    const int center = (NUM_SENSORS - 1) / 2;

    for (int i = 0; i < NUM_SENSORS; i++) {

        // normalized[i] is 0..1000  (0 = white, 1000 = black)
        int v = normalized[i];

        // Very small values contribute noise; ignore them
        if (v < 50) continue;
        
        int pos = (i - center);

        weighted_sum += (long)v * pos;
        sum += v;
    }

    if (sum == 0) {
        return LINE_NOT_FOUND;   // complete white = no line
    }

    return (float)weighted_sum / (float)sum;
}

static float pid_update(motors_t *motors, float error)
{
    line_follower_pid_t *pid = &motors->line_pid;

    // Proportional
    float P = pid->Kp * error;

    // Integral with simple anti-windup
    pid->integral += error;
    float I = pid->Ki * pid->integral;

    // Derivative (based on error difference)
    float derivative = error - pid->last_error;
    float D = pid->Kd * derivative;

    pid->last_error = error;

    float output = P + I + D;

    // Clamp output
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}


static inline motor_direction_t dir_from_speed(float s)
{
    if (s > 0) return MOTOR_FORWARD;
    if (s < 0) return MOTOR_BACKWARD;
    return MOTOR_STOP;
}


static uint16_t mps_to_pwm(float mps)
{
    // Linear mapping
    const float mps_min = 0.1f;
    const float mps_max = 1.5f;
    const float pwm_min = 550.0f;
    const float pwm_max = 950.0f; 

    // Clamp input speed
    if (mps < mps_min)  mps = mps_min;
    if (mps > mps_max)  mps = mps_max;

    float pwm = pwm_min +
                (mps - mps_min) *
                (pwm_max - pwm_min) /
                (mps_max - mps_min);

    return (uint16_t)(pwm + 0.5f); 
}


static void follow_line_task(void *arg)
{
    motors_t *motors = (motors_t *)arg;
    static motor_direction_t dir_right;
    static motor_direction_t dir_left;
    static int last_pos = 1;

    while (1) {
        float base_pwm =  mps_to_pwm(motors->moving_speed);

        read_line_sensors();
        calibrate_and_normalize(line_reading, normalized_reading);

        float pos = compute_line_position(normalized_reading);

        if (pos == LINE_NOT_FOUND) pos = last_pos/(abs(last_pos))*1;
        else last_pos = pos;

        float error = pos;
        float correction = pid_update(motors, error);

        //calc new pwm
        int left_speed  = base_pwm - correction; // not speed: PWM
        int right_speed = base_pwm + correction; // can become negative  = spinning?        

        dir_right = dir_from_speed(right_speed); // not speed: PWM
        dir_left  = dir_from_speed(left_speed); 
     
        // Send pwm to control loop
        motor_set_pwm_target(&motors->motor_right, right_speed, dir_right);
        motor_set_pwm_target(&motors->motor_left, left_speed, dir_left);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void line_follower_init(motors_t *motors)
{

    init_adc();
    init_calibration();
    motors->moving_speed = 0.0f; // start at zero speed

    pid_init(&motors->line_pid,
            55.0f,   // P
            0.0f,    // I
            10.0f,   // D
            -450.0f, // Maximum negative change in pwm
            +450.0f  // Maximum positive change in pwm
    ); 

    xTaskCreate(follow_line_task, "follow_line", 4096, motors, 5, NULL );        

 
    ESP_LOGI(TAG, "Line following initialized");
}

