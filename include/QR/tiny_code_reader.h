#pragma once

#include "driver/i2c_master.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 7-bit I2C address of the Tiny Code Reader
#define TINY_CODE_READER_I2C_ADDR        0x0C
#define TINY_CODE_READER_RESULT_BYTES    256
#define TINY_CODE_READER_MAX_CONTENT_LEN 254

// Packed struct that matches the module's output format
typedef struct __attribute__((packed)) {
    uint16_t content_length;                     // Little-endian
    uint8_t  content_bytes[TINY_CODE_READER_MAX_CONTENT_LEN];
} tiny_code_reader_results_t;

/**
 * @brief Read a result frame from the Tiny Code Reader.
 *
 * This polls the sensor once. If no QR code has been seen recently,
 * content_length will be 0.
 *
 * @param i2c_port I2C port (I2C_NUM_0 or I2C_NUM_1)
 * @param out_results Pointer to struct to fill
 * @return true on success, false on I2C error
 */
bool tiny_code_reader_read(i2c_master_dev_handle_t dev,
                           tiny_code_reader_results_t *out_results);

/**
 * @brief Enable or disable the on-board status LED.
 *
 * According to the docs, writing 0x00 to register 0x01 disables the LED. :contentReference[oaicite:4]{index=4}
 *
 * @param i2c_port I2C port
 * @param enable true = LED on (default), false = LED off
 * @return true on success
 */
bool tiny_code_reader_set_led(i2c_master_dev_handle_t dev, bool enable);

esp_err_t tiny_code_reader_init(i2c_master_bus_handle_t bus,
                                i2c_master_dev_handle_t *out_dev);
#ifdef __cplusplus
}
#endif
