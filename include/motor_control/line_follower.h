#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include <stdbool.h>

#include "motor_control/motor_control.h"

#define NUM_SENSORS 7

#define BASE_SPEED 0.8f

//for normalization
#define NORM_MAX 1000   // normalized scale 0–1000

//for the graphical ADC shown
#define ADC_MAX_VALUE 4095
#define BAR_MAX_WIDTH 50   // max number of '|' symbols for full scale

//for line calculation
#define POSITION_SCALE 1000
#define LINE_NOT_FOUND 151515   // some big "impossible" number

extern void line_follower_init(motors_t *motors);

/**
 * @brief Request the motor controller to achieve a target base speed.
 *
 * @param motors Pointer to the motors structure.
 * @param speed Target base speed in m/s.
 */
static inline void set_speed(motors_t *motors, float speed) {
    motors->moving_speed = speed;
}