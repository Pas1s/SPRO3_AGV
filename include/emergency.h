#pragma once
#include "driver/gpio.h"
#include "motor_control/motor_control.h"
#include "pins.h"

extern volatile bool emergency_flag; // defined in motor_control.c
extern volatile relay_state_t g_relay_state;

void emergency_init(void);

inline void mosfet_set_state(relay_state_t state) { 
    gpio_set_level(EMERGENCY_MOSFET_PIN, state); 
}