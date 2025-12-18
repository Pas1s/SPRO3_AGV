#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "math.h"

#include "motor_control/motor_control.h"
#include "motor_control_internal.h"
#include "motor_control/encoder.h"
#include "emergency.h"
#include "pins.h"

static const char *TAG = "MOTORS";
//                                 Direction      Duty  Speed_dps  Speed_mps  Distance
const motor_values_t stop_values = {MOTOR_FORWARD,   0,    0.0f,      0.0f,      0.0f}; // reseting values
volatile relay_state_t g_relay_state = RELAY_OFF;
volatile bool emergency_flag = false;
static motors_t* global_motors; 

// Forward declaration
static inline void motor_set_pwm(motor_control_t *motor);
static void update_motor_direction(motor_control_t *motor);

// Tasks  
static void motor_update_task(void *arg);

// Global MCPWM timer handle
static mcpwm_timer_handle_t timer = NULL;


void motors_struct_init(motors_t *motors)
{
    motor_control_t *right = &motors->motor_right;
    motor_control_t *left  = &motors->motor_left;

    right->id = MOTOR_RIGHT;
    right->pwm_pin = MOTOR_RIGHT_PWM_PIN;
    right->forward_pin = MOTOR_RIGHT_FORWARD_PIN;
    right->reverse_pin = MOTOR_RIGHT_REVERSE_PIN;
    right->target = stop_values;
    right->current = stop_values;
    right->pid = default_pid_right;

    left->id = MOTOR_LEFT;
    left->pwm_pin = MOTOR_LEFT_PWM_PIN;
    left->forward_pin = MOTOR_LEFT_FORWARD_PIN;
    left->reverse_pin = MOTOR_LEFT_REVERSE_PIN;
    left->target = stop_values;
    left->current = stop_values;
    left->pid = default_pid_left;


    global_motors = motors;
}

motors_t* get_motors(void)
{
    return global_motors;
}

static void motor_init_single(motor_control_t *motor)
{
    // GPIO CONFIG
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << motor->forward_pin) | (1ULL << motor->reverse_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return;
    }

    // MCPWM OPERATOR
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };

    err = mcpwm_new_operator(&operator_config, &motor->operator_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_operator failed: %s", esp_err_to_name(err));
        return;
    }

    // TIMER CONNECT

    err = mcpwm_operator_connect_timer(motor->operator_handle, timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "operator_connect_timer failed: %s", esp_err_to_name(err));
        return;
    }

    // COMPARATO
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };


    err = mcpwm_new_comparator(
        motor->operator_handle,
        &comparator_config,
        &motor->comparator
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_comparator failed: %s", esp_err_to_name(err));
        return;
    }

    // GENERATOR 
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = motor->pwm_pin,
    };

    err = mcpwm_new_generator(
        motor->operator_handle,
        &generator_config,
        &motor->generator
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_generator failed: %s", esp_err_to_name(err));
        return;
    }


    err = mcpwm_generator_set_action_on_timer_event(
        motor->generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_action_on_timer_event failed: %s", esp_err_to_name(err));
        return;
    }

    err = mcpwm_generator_set_action_on_compare_event(
        motor->generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            motor->comparator,
            MCPWM_GEN_ACTION_LOW
        )
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_action_on_compare_event failed: %s", esp_err_to_name(err));
        return;
    }
    // set motor to intitial state
    update_motor_direction(motor);
    motor_set_pwm(motor);
}


/**
 * @brief Initialize the motor controller system for tank drive
 * @param motor_right Pointer to right motor structure
 * @param motor_left Pointer to left motor structure
 */
void motors_init(motors_t *motors)
{
    motor_control_t *motor_left = &motors->motor_left;
    motor_control_t *motor_right = &motors-> motor_right;

    motors_struct_init(motors);

    // init encoder/optocoupler for speed measurement
    encoder_init(motor_right, motor_left);

    // init pins for emergency button and reset button
    emergency_init();

    // Create shared MCPWM timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .period_ticks = MCPWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Initialize both motors with separate operators
    motor_init_single(motor_right);
    motor_init_single(motor_left);

    // Enable and start timer (shared by both motors)
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    xTaskCreate(motor_update_task, "RightMotorUpdate", 4096, motor_right, 5, &motor_right->task_handle);
    xTaskCreate(motor_update_task, "LeftMotorUpdate", 4096, motor_left, 5, &motor_left->task_handle);

    ESP_LOGI(TAG, "Both motors initialized succesfully");
}


static inline float saturate(float x, float minv, float maxv) {
    if (x < minv) return minv;
    if (x > maxv) return maxv;
    return x;
}

static void update_motor_direction(motor_control_t *motor) {

    if(motor->braking_flag) { // so they are not overwritten somewhere.
        gpio_set_level(motor->forward_pin, dir_map[MOTOR_BRAKE].forward);
        gpio_set_level(motor->reverse_pin, dir_map[MOTOR_BRAKE].reverse);
    } else {
        gpio_set_level(motor->forward_pin, dir_map[motor->target.dir].forward);
        gpio_set_level(motor->reverse_pin, dir_map[motor->target.dir].reverse);
        motor->current.dir = motor->target.dir;
    }
}

static void update_motor_speed(motor_control_t *motor)
{
    const float dt = ((float)PID_UPDATE_DELAY_MS) / 1000.0f;
    const float max_step = DUTY_STEP_MAX_PER_MILLE;
    float current_speed = motor->current.speed;

    // if                           target_speed == ~0.0f          or              changing direction
    bool stopping = ((fabsf(motor->target.speed) < STOP_SPEED_EPS) || (motor->current.dir != motor->target.dir));

    // Effective target for PID
    float effective_target = stopping ? 0.0f : motor->target.speed; // independent of external input, if changing dir, target = 0.0f 
    float error = effective_target - current_speed;

    float derivative_raw = (current_speed - motor->pid.last_speed) / dt;
    float derivative_filtered = weighted_ma(&motor->pid.derivative_ma, derivative_raw);

    // Raw, unclamped pwm change
    float pwm_change_raw = motor->pid.kp * error                // P
                         + motor->pid.ki * motor->pid.integral  // I
                         - motor->pid.kd * derivative_filtered; // D (on-measurement)

    // Clamp PWM change per step
    float pwm_change = saturate(pwm_change_raw, -max_step, max_step);

    // Determine minimum PWM based on stopping state
    float pwm_min = stopping ? 0.0f : MIN_PWM_PER_MILLE; // 0 pwm if stopping, othewise min pwm where motor spins

    // Calculate tentative PWM and clamp within allowed range
    float tentative_pwm = motor->current.duty_cycle + pwm_change;
    float new_pwm = saturate(tentative_pwm, pwm_min, (float)MAX_DUTY_PER_MILLE);

    if (tentative_pwm == pwm_min || tentative_pwm == MAX_DUTY_PER_MILLE) {
        // Actuator saturated, integrate only if error drives away from saturation
        if (!((error > 0 && tentative_pwm == MAX_DUTY_PER_MILLE) ||
            (error < 0 && tentative_pwm == pwm_min))) {
            motor->pid.integral += error * dt;
        }
    } else {
        motor->pid.integral += error * dt;
    }
        
    motor->pid.integral = saturate(motor->pid.integral,
                                    -motor->pid.integral_max,
                                    motor->pid.integral_max);

    // Update PID history
    motor->pid.last_speed = current_speed;
    motor->pid.last_speed_valid = true;
    motor->pid.last_integral_valid = true;

    // Apply PWM
    motor->current.duty_cycle = (uint16_t)new_pwm;
    motor_set_pwm(motor);    
}

inline static void update_motor_pwm(motor_control_t *motor) {
    if (motor->target.dir != motor->current.dir ) { 
        // motor->current.duty_cycle = 0; // for external comparisons
        mcpwm_comparator_set_compare_value(motor->comparator, 1000); // pwm = 0 - if duty_cycle is changed externally
    } else { 
        motor->current.duty_cycle = motor->target.duty_cycle;
        motor_set_pwm(motor); 
    }
}

motor_state_t get_motor_state(motor_control_t *motor)
{

    // Overwriting flags
    if (emergency_flag) return MOTOR_STATE_EMERGENCY;
    if (motor->braking_flag) return MOTOR_STATE_BRAKE;

    motor_state_t next_state = motor->state;

    const uint16_t target_duty  = motor->target.duty_cycle;
    const uint16_t current_duty = motor->current.duty_cycle;
    const bool is_moving        = (motor->current.speed < 6.0f);

    switch (motor->state)
    {
    case MOTOR_STATE_STOPPED:
        if (motor->target.dir != motor->current.dir)
            next_state = MOTOR_STATE_CHANGING_DIR;
        else if (target_duty > 0)
            next_state = MOTOR_STATE_ACCELERATING;
        break;

    case MOTOR_STATE_CHANGING_DIR: // only valid when speed == 0
        if (target_duty == 0)
            next_state = MOTOR_STATE_STOPPED;
        else
            next_state = MOTOR_STATE_ACCELERATING;
        break;

    case MOTOR_STATE_ACCELERATING:
    case MOTOR_STATE_RUNNING:
        if (target_duty == 0) {
            next_state = MOTOR_STATE_DECELERATING;
        } else if (motor->current.dir != motor->target.dir) {
            next_state = MOTOR_STATE_DECELERATING;
        } else if (target_duty < current_duty) {
            next_state = MOTOR_STATE_DECELERATING;
        } else if (target_duty > current_duty) {
            next_state = MOTOR_STATE_ACCELERATING;
        } else {
            next_state = MOTOR_STATE_RUNNING;
        }
        break;

    case MOTOR_STATE_DECELERATING:
        if (!is_moving) {
            next_state = MOTOR_STATE_CHANGING_DIR;
        } else if (target_duty > current_duty) {
            next_state = MOTOR_STATE_ACCELERATING;
        }
        // else continue decelerating
        break;

    case MOTOR_STATE_EMERGENCY:
        if (!emergency_flag) {
            if (target_duty == 0) {
                next_state = MOTOR_STATE_STOPPED;
            } else if (motor->target.dir != motor->current.dir) {
                next_state = MOTOR_STATE_CHANGING_DIR;
            } else if (target_duty > current_duty) {
                next_state = MOTOR_STATE_ACCELERATING;
            } else {
                next_state = MOTOR_STATE_DECELERATING;
            }
        }
        break;

    case MOTOR_STATE_BRAKE:
        if (!motor->braking_flag)
        
            next_state = MOTOR_STATE_CHANGING_DIR;
        break;

    default:
        break;
    }

    return next_state;
}


void motor_relay_toggle(relay_state_t state)
{
    if (state != g_relay_state) {
        mosfet_set_state(state);
        g_relay_state = state;
    }
}


static void motor_update_task(void *arg)
{
    motor_control_t *motor = (motor_control_t *)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t delay = pdMS_TO_TICKS(PID_UPDATE_DELAY_MS); 

    while (1) 
    {
        motor->state = get_motor_state(motor);
        switch (motor->state) 
        {
            case MOTOR_STATE_STOPPED:
                motor_relay_toggle(RELAY_OFF);
                break;
                

            case MOTOR_STATE_ACCELERATING:
                motor_relay_toggle(RELAY_ON);
                update_motor_pwm(motor);
                break;

            case MOTOR_STATE_RUNNING:
                motor_relay_toggle(RELAY_ON);
                update_motor_pwm(motor);
                
                break;

            case MOTOR_STATE_DECELERATING:
                motor_relay_toggle(RELAY_ON);
                update_motor_pwm(motor);
                break;

            case MOTOR_STATE_CHANGING_DIR:
                motor_relay_toggle(RELAY_ON);
                update_motor_direction(motor);
                break;

            case MOTOR_STATE_EMERGENCY:
                break;

            case MOTOR_STATE_BRAKE:
                motor_relay_toggle(RELAY_ON);
                motor->current.duty_cycle = 1000;
                update_motor_pwm(motor);
                motor->target.dir = MOTOR_BRAKE;
                update_motor_direction(motor);
                break;
            
        }

        xTaskDelayUntil(&last_wake, delay);  
    }
}

