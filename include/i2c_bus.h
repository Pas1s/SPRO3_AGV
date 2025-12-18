// i2c_bus.h
#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"


// Adjust these to your wiring
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     19
#define I2C_SCL_PIN     20
#define I2C_TIMEOUT_TICKS 400000


#define TOF_RIGHT_ADDR_8BIT  ((uint16_t)0x56)
#define TOF_LEFT_ADDR_8BIT   ((uint16_t)0x54)
#define TOF_CENTER_ADDR_8BIT   ((uint16_t)0x58)

#ifdef __cplusplus
extern "C" {
#endif

// Initialise the I2C master bus (idempotent)
esp_err_t i2c_bus_init(void);

// Get the global bus handle (for your other sensors)
i2c_master_bus_handle_t i2c_bus_get_handle(void);

i2c_master_bus_config_t i2c_bus_get_cfg(void);

#ifdef __cplusplus
}
#endif
