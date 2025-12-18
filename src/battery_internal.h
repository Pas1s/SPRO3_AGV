#pragma once

#include "battery.h"
#include "pins.h"

#define INA260_I2C_FREQ_HZ          400000
#define INA260_ADDR                 0x40 ///< INA260 default i2c address

// Registers for the INA260
#define  INA260_REG_CONFIG       0x00      // Configuration register
#define  INA260_REG_CURRENT      0x01      // Current register (Signed output)
#define  INA260_REG_VOLTAGE      0x02
#define  INA260_REG_POWER        0x03
#define INA260_REG_MASK_ENABLE   0x06 // Interrupt/Alert setting and checking register
#define INA260_REG_ALERT_LIMIT   0x07 // Alert limit value register

// LSB
#define INA260_LSB_MV 1.25f // mV/bit
#define INA260_LSB_MA 1.25f // mA/bit
#define INA260_LSB_MW 10.0f // mW/bit

#define BATTERY_MAX_MAH         2300.0f // Assumed full capacity in Ah   
#define BATTERY_SOH_PCT         100.0f // assumed percent left of original capacity
#define MAX_CURRENT_MA          5000  // >=5A = break
#define TIME_BETWEEN_MEAS_MS    500 
#define INA260_READ_ATTEMPS     3
#define INA260_MAX_WAIT_MS      10 // max time to wait for a response over i2c in ms

typedef enum { REG_CURRENT, REG_VOLTAGE, REG_POWER, NUM_REGS } ina260_reg_idx_t;

static const uint8_t ina260_regs[NUM_REGS] = {
    INA260_REG_CURRENT,
    INA260_REG_VOLTAGE,
    INA260_REG_POWER
};
// For doing linear regression
static const struct {
    uint16_t mv;
    float soc;
} ocv_map[] = {
    {12800, 1.00f},   // rested full AGM
    {12600, 0.90f},
    {12400, 0.75f},
    {12200, 0.50f},
    {12000, 0.25f},
    {11900, 0.10f},
    {11800, 0.00f}
};

