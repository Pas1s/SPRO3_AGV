#pragma once

#include "motor_control/motor_control.h"
#include "battery.h"

typedef struct {
  motors_t motors;
  battery_data_t battery_data;
} agv_t;
