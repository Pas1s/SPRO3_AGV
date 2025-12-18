#pragma once

// Right Motor 
#define MOTOR_RIGHT_FORWARD_PIN     GPIO_NUM_16
#define MOTOR_RIGHT_REVERSE_PIN     GPIO_NUM_15
#define MOTOR_RIGHT_PWM_PIN         GPIO_NUM_48

// Left Motor
#define MOTOR_LEFT_FORWARD_PIN      GPIO_NUM_18
#define MOTOR_LEFT_REVERSE_PIN      GPIO_NUM_17
#define MOTOR_LEFT_PWM_PIN          GPIO_NUM_47

// Encoder
#define ENCODER_IRQ_PIN_RIGHT       GPIO_NUM_39 
#define ENCODER_IRQ_PIN_LEFT        GPIO_NUM_14

// Emergency
#define EMERGENCY_MOSFET_PIN        GPIO_NUM_40
#define EMERGENCY_SIGNAL_PIN        GPIO_NUM_21
#define EMERGENCY_RESET_BTN_PIN     GPIO_NUM_1

// Package sensor
#define PACKAGE_SENSOR_PIN      GPIO_NUM_1
