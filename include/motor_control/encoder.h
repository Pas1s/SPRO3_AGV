#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "motor_control/motor_control.h"
#include "pins.h"

#define ENCODER_HOLES         20
#define DEGREES_PER_PULSE     (360.0f / (ENCODER_HOLES))

#define PRINT_INTERVAL_MS     10
#define TIMEOUT_MS            150   // 150 ms → speed = 0
#define NOISE_FILTER_US       100 // 5 ms

#define ENCODER_FREQ_CUTOFF 1.0f //hz - seems like a good number

#define MAX_ACCEL_DPS2 2000  // Max acceleration (deg/s^2)

// 1 MHz timer resolution (1 µs per tick)
#define TIMER_RES_HZ 1000000ULL

// Noise filtering in timer ticks
#define NOISE_FILTER_TICKS (NOISE_FILTER_US * (TIMER_RES_HZ / 1000000ULL))

extern void encoder_init(motor_control_t *motor_right, motor_control_t *motor_left);

extern float weighted_ma(speed_ma_t *ma, float new_value);