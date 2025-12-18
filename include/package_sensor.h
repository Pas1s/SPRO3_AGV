#pragma once

#include "agv.h"


#define PIN_LOW_WITH_PACKAGE // if defined, pin == 0 -> package is on

#define PACKAGE_TASK_DELAY_MS 10

extern void package_sensor_init(agv_t *agv);


extern TaskHandle_t package_monitor_task_handle;

// This function is called when the AGV picks up the package
static inline void picked_up_package(void) {
    if (package_monitor_task_handle != NULL) {
        vTaskResume(package_monitor_task_handle);  // Resume the task when the package is picked up
    }
}

// This function is called when the AGV puts down the package
static inline void put_down_package(void) {
    if (package_monitor_task_handle != NULL) {
        vTaskSuspend(package_monitor_task_handle);  // Suspend the task when the package is put down
    }
}

