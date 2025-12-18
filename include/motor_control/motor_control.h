#pragma once
//--------------------------------------------------
// Includes
//--------------------------------------------------
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.h"

#include "driver/pulse_cnt.h"
//--------------------------------------------------
// Constants & Macros
//--------------------------------------------------
#define PI 3.1415f
#define WHEEL_RADIUS_M 0.032f
#define MPS_TO_DPS(x) ( (x) * 180.0f / (PI * WHEEL_RADIUS_M) )
#define DPS_TO_MPS(x) ( (x) * (PI * WHEEL_RADIUS_M) / 180.0f )

#define SPEED_MA_LEN 15 // amount of values to use for rolling weighted average

//--------------------------------------------------
// Enumerations
//--------------------------------------------------
typedef enum {
    MOTOR_RIGHT = 0,
    MOTOR_LEFT,
    MOTOR_TOTAL
} motor_id_t;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD, 
    MOTOR_BRAKE
} motor_direction_t;

typedef enum {
    MOTOR_STATE_STOPPED,
    MOTOR_STATE_ACCELERATING,
    MOTOR_STATE_DECELERATING,
    MOTOR_STATE_BRAKE,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_CHANGING_DIR,
    MOTOR_STATE_EMERGENCY
} motor_state_t;

typedef enum {
    RELAY_OFF = 0,
    RELAY_ON = 1
} relay_state_t;

//--------------------------------------------------
// Motor Value Structs
//--------------------------------------------------
typedef struct {
    motor_direction_t dir;
    uint16_t duty_cycle;
    float speed; // deg/s
    float speed_mps;
    float distance;
} motor_values_t;


//--------------------------------------------------
// Rolling Weighted Average Struct
//--------------------------------------------------
typedef struct {
    float history[SPEED_MA_LEN];
    int index;
    bool filled;
} speed_ma_t;

//--------------------------------------------------
// Encoder
//--------------------------------------------------
typedef struct {
    gpio_num_t irq_pin;
    TaskHandle_t task_handle;
    portMUX_TYPE lock;

    volatile uint64_t last_tick;    
    volatile uint64_t dt_ticks_sum;     
    volatile uint16_t pulse_count;
    speed_ma_t speed_ma;
} encoder_t;

//--------------------------------------------------
// Motor PID Config
//--------------------------------------------------
typedef struct {
    float kp, ki, kd, kd_pwm, last_speed;
    float integral_max;
    float integral, last_error;
    bool last_speed_valid;
    bool last_integral_valid;

    speed_ma_t derivative_ma;
} pid_config_t;


//--------------------------------------------------
// Motor Control Structure
//--------------------------------------------------
typedef struct {
    motor_id_t id;
    volatile motor_state_t state;
    volatile bool braking_flag;

    gpio_num_t pwm_pin;
    gpio_num_t forward_pin;
    gpio_num_t reverse_pin;
    
    encoder_t encoder;

    volatile motor_values_t target;
    volatile motor_values_t current;

    pid_config_t pid;

    mcpwm_oper_handle_t operator_handle;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;

    TaskHandle_t task_handle;
} motor_control_t;

//--------------------------------------------------
// Line Follower PID
//--------------------------------------------------
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float out_min;
    float out_max;
} line_follower_pid_t; 

//--------------------------------------------------
// Motors Wrapper
//--------------------------------------------------
typedef struct {
    motor_control_t motor_right;
    motor_control_t motor_left;
    float moving_speed;
    line_follower_pid_t line_pid;
} motors_t;

//--------------------------------------------------
// Inline Motor Control Helpers
//--------------------------------------------------
/**
 * @brief Request the motor controller to achieve a target speed.
 *
 * @param motor Pointer to the motor control structure.
 * @param speed Target speed in m/s.
 */
static inline void motor_speed_request(motor_control_t *motor, float speed) {
    motor->target.speed = MPS_TO_DPS(speed);
    motor->target.speed_mps = speed;
}

/**
 * @brief Request the motor controller to adjust the current target speed by a delta.
 *
 * This adds the specified delta to the motor's existing speed target,
 * allowing incremental speed changes (e.g. for acceleration control).
 *
 * @param motor Pointer to the motor control structure.
 * @param delta_speed Change in speed in m/s (positive or negative).
 */
static inline void motor_speed_delta_request(motor_control_t *motor, float delta_speed) {
    motor->target.speed += MPS_TO_DPS(delta_speed);
    motor->target.speed_mps += delta_speed;
}

/**
 * @brief Request the motor controller to set a target direction.
 *
 * @param motor Pointer to the motor control structure.
 * @param direction Target direction (forward, backward, etc.).
 */
static inline void motor_direction_request(motor_control_t *motor, motor_direction_t direction) {
    motor->target.dir = direction;
}

/**
 * @brief Request the motor controller to achieve a target speed and direction.
 *
 * @param motor Pointer to the motor control structure.
 * @param speed Target speed in m/s.
 * @param direction Target direction (MOTOR_FORWARD, etc.).
 */
    static inline void motor_set_target(motor_control_t *motor, float speed, motor_direction_t direction) {
        motor->target.speed = MPS_TO_DPS(speed);
        motor->target.speed_mps = speed;
        motor->target.dir = direction;
    }

static inline void motor_set_pwm_target(motor_control_t *motor, uint16_t pwm, motor_direction_t dir) {
    motor->target.dir = dir;
    motor->target.duty_cycle = (pwm > UINT16_C(1000)) ? UINT16_C(1000) : pwm;
}

/**
 * @brief Request both motors to achieve a shared target speed and direction.
 *
 * This sets the same target speed and direction for the left and right motors,
 * useful for forward/backward straight-line movement.
 *
 * @param motors Pointer to the motors structure containing both motor controllers.
 * @param speed Target speed in m/s for both motors.
 * @param direction Target direction for both motors.
 */
static inline void motors_set_target(motors_t *motors, float speed, motor_direction_t direction) {
    motors->motor_left.target.speed = MPS_TO_DPS(speed);
    motors->motor_left.target.dir = direction;
    motors->motor_left.target.speed_mps = speed;

    motors->motor_right.target.speed = MPS_TO_DPS(speed);
    motors->motor_right.target.dir = direction;
    motors->motor_right.target.speed_mps = speed;
}

static inline void motor_stop(motor_control_t *motor) {
    motor->braking_flag = true;
}

static inline void motors_stop(motors_t *motors) {
    motor_stop(&motors->motor_right);
    motor_stop(&motors->motor_left);
}

static inline void motor_resume(motor_control_t *motor) {
    motor->braking_flag = false;
}

static inline void motors_resume(motors_t *motors) {
    motor_resume(&motors->motor_right);
    motor_resume(&motors->motor_left);
}

static inline void motor_set_pwm(motor_control_t *motor) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->comparator, motor->current.duty_cycle));
}

motors_t* get_motors(void);
//--------------------------------------------------
// External Function Declarations
//--------------------------------------------------
extern void motors_init(motors_t *motors);
extern void motor_direction_request(motor_control_t *motor, motor_direction_t direction);

