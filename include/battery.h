#pragma once

#include <driver/i2c_types.h> // for i2c handle
#include "freertos/FreeRTOS.h" //for error

typedef enum {
    BATTERY_STATE_CHARGING,
    BATTERY_STATE_DISCHARGING,
} battery_state_t;

typedef enum {
    BATTERY_FULL_CHARGE,    // ~100% SOC
    BATTERY_HIGH_CHARGE,    // ~75-99%
    BATTERY_MEDIUM_CHARGE,  // ~50-74%
    BATTERY_LOW_CHARGE,     // ~25-49%
    BATTERY_CRITICAL_CHARGE // <25%
} battery_charge_t;

typedef struct {
    battery_state_t state;
    float current_ma;
    float voltage_mv;
    float power_mw;  
    float charge_mah;
    float soh_pct;             // percent left of original capacity
    float rated_capacity_mah;  // rated capacity when new
    uint16_t charge_cycles;     // persisten storage needed
} battery_data_t;

// Functions
extern esp_err_t battery_init(i2c_master_bus_handle_t i2c_bus, battery_data_t *battery_data);

static inline uint8_t get_battery_charge_pct(battery_data_t *batt) {
    return (uint8_t)((batt->charge_mah / (batt->rated_capacity_mah * (batt->soh_pct / 100.0f))) * 100.0f);
}

