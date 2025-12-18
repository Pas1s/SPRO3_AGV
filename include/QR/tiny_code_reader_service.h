// tiny_code_reader_service.h
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIGH_SPEED_MODE 1
#define MED_SPEED_MODE 2
#define LOW_SPEED_MODE 3


/**
 * @brief Initialize Tiny Code Reader service:
 *  - init I2C bus (using i2c_bus.c)
 *  - add Tiny Code Reader I2C device
 *  - create background task that checks sensor every 150 ms
 */
esp_err_t tiny_code_reader_service_init(void);


#ifdef __cplusplus
}
#endif
