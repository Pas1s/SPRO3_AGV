#pragma once

#include "freertos/FreeRTOS.h"
#include "motor_control/motor_control.h"
#include "pins.h"

// PWM Configuration (shared between motors)
#define MCPWM_TIMER_RESOLUTION_HZ 20000000  // 20MHz
#define MCPWM_PERIOD_TICKS        1000  // 20 khz     
#define MAX_DUTY_PER_MILLE        1000.0f
#define MIN_PWM_PER_MILLE         500.0f 

// Motor Control Parameters
#define PID_UPDATE_DELAY_MS         10     // Delay between motor updates
#define DUTY_STEP_MAX_PER_MILLE     1000.0f    // Max Duty cycle increment (1/1000)

#define STOP_SPEED_EPS              0.5f    // degrees per second


// Precompiler optimization
#if MCPWM_PERIOD_TICKS == 1000
    #define PER_MILLE_TO_TICKS(x)   x
#else
    #define PER_MILLE_TO_TICKS(x)   (((x) * MCPWM_PERIOD_TICKS + 500) / 1000)
#endif

//M0
static const pid_config_t default_pid_right = {
    .kp = 0.003f,
    .ki = 0.0035f,    
    .kd = 0.00045f, 
    .integral_max = 100.0f,
    .last_speed_valid = false,
    .last_integral_valid = false
};
//M1
static const pid_config_t default_pid_left = {
    .kp = 0.0019f,
    .ki = 0.00291f,    
    .kd = 0.00045f, 
    .integral_max = 100.0f,
    .last_speed_valid = false,
    .last_integral_valid = false
};

static const struct {
    uint8_t forward;
    uint8_t reverse;
} dir_map[] = {
    [MOTOR_FORWARD]        = {1, 0},
    [MOTOR_BACKWARD]       = {0, 1},
    [MOTOR_BRAKE]          = {1, 1}, // active stop 
    [MOTOR_STOP]           = {0, 0}  // passive stop
};
