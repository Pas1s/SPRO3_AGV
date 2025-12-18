// Official includes
#include "esp_log.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/i2c_types.h>
#include "driver/i2c_master.h"
#include <stdio.h>
#include "driver/gpio.h"

#include "battery_internal.h" //
#include "battery.h"


static bool emergency_flag = false; // should be removed to use the one in motor_control

static const char *TAG = "BATTERY";
static i2c_master_dev_handle_t ina260_dev = NULL;


//Functions
static float ocv_to_soc(uint16_t mv);
static void calculate_charge_estimate(battery_data_t* battery_data);
static esp_err_t ina260_read_reg(uint8_t reg, uint16_t *value);

static void battery_struct_init(battery_data_t *batt) {
    batt->rated_capacity_mah = BATTERY_MAX_MAH;
    batt->soh_pct = BATTERY_SOH_PCT; // assume 100% 
}

static float ocv_to_soc(uint16_t mv)
{
    size_t n = sizeof(ocv_map) / sizeof(ocv_map[0]);

    // clamp top/bottom
    if (mv >= ocv_map[0].mv) return 1.0f;
    if (mv <= ocv_map[n - 1].mv) return 0.0f;

    // find interval for linear interpolation
    for (size_t i = 0; i < n - 1; i++) {
        uint16_t v1 = ocv_map[i].mv;
        uint16_t v2 = ocv_map[i + 1].mv;

        if (mv <= v1 && mv >= v2) {
            float s1 = ocv_map[i].soc;
            float s2 = ocv_map[i + 1].soc;
            float t = (float)(mv - v2) / (float)(v1 - v2);
            return s2 + t * (s1 - s2);
        }
    }
    return 0.5f; // shuting up compiler
}

static void calculate_charge_estimate(battery_data_t* batt)
{
    uint16_t raw_voltage_val = 0;
    for (uint8_t i = 0; i < INA260_READ_ATTEMPS; i++) {
        if (ina260_read_reg(ina260_regs[REG_VOLTAGE], &raw_voltage_val) == ESP_OK) {
            break;
        }
    }

    // Fallback
    if (raw_voltage_val < 10) { // less than 12.5mV 
        raw_voltage_val = 50;
    }

    uint16_t voltage_mv = ((uint32_t)raw_voltage_val*5)/4; // 1.25 mV/bit 

    float soc = ocv_to_soc(voltage_mv);

    // clamp (again)
    if (soc > 1.0f) soc = 1.0f;
    if (soc < 0.0f) soc = 0.0f;

    batt->charge_mah = batt->rated_capacity_mah * (batt->soh_pct / 100.0f) * soc;
}


static esp_err_t ina260_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];

    // Tell the chip the register 
    esp_err_t ret = i2c_master_transmit(ina260_dev, &reg, 1, INA260_MAX_WAIT_MS); // only 1 byte
    if (ret != ESP_OK) return ret;

    // Read the 16-bit value
    ret = i2c_master_receive(ina260_dev, buf, 2, INA260_MAX_WAIT_MS); // 2 bytes (16 bits)
    if (ret != ESP_OK) return ret;

    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

static esp_err_t ina260_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    return i2c_master_transmit(ina260_dev, buf, 3, INA260_MAX_WAIT_MS);
}

esp_err_t ina260_configure(void)
{
    // Averaging = 64 samples   (code 3)
    const uint16_t AVG = (3 << 9);

    // Voltage conversion time = 1.1 ms (code 4)
    const uint16_t VT = (4 << 6);

    // Current conversion time = 1.1 ms (code 4)
    const uint16_t CT = (4 << 3);

    // Mode = continuous (code 7)
    const uint16_t MODE = 7;

    uint16_t config = AVG | VT | CT | MODE;

    return ina260_write_reg(INA260_REG_CONFIG, config);
}

static void battery_monitor_task(void *arg)
{
    battery_data_t *data = (battery_data_t *)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(TIME_BETWEEN_MEAS_MS); 
    const float delta_hours = (float)TIME_BETWEEN_MEAS_MS / 1000.0f / 3600.0f; // 500ms in hours

    for (;;)
    {
        uint16_t raw_values[NUM_REGS];
        bool all_ok = true;

        for (ina260_reg_idx_t r = 0; r < NUM_REGS; r++)
        {
            if (ina260_read_reg(ina260_regs[r], &raw_values[r]) != ESP_OK)
            {
                ESP_LOGW(TAG, "INA260 read failed for register %d", r);
                all_ok = false;
            }
        }

        if (all_ok)
        {
            // Convert raw readings to actual units
            data->current_ma = (int16_t)raw_values[REG_CURRENT] * INA260_LSB_MA;  // mA
            data->voltage_mv = raw_values[REG_VOLTAGE] * INA260_LSB_MV;           // mV
            data->power_mw   = raw_values[REG_POWER] * INA260_LSB_MW;             // mW

            if (data->current_ma > MAX_CURRENT_MA)  emergency_flag = true;// emergency
            // Coulomb counting (mAh)
            data->charge_mah -= data->current_ma * delta_hours;
        }

        vTaskDelay(delay_ticks);
    }
}


esp_err_t ina260_init(i2c_master_bus_handle_t i2c_bus)
{
    i2c_device_config_t ina_cfg = {
        .device_address = INA260_ADDR,      // default INA260 address
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = INA260_I2C_FREQ_HZ,
    };

    return i2c_master_bus_add_device(i2c_bus, &ina_cfg, &ina260_dev);
}

static void IRAM_ATTR ina_alert_isr(void *arg)
{
    emergency_flag = true;
}

esp_err_t battery_init(i2c_master_bus_handle_t i2c_bus, battery_data_t* battery_data) {
    esp_err_t ret = ina260_init(i2c_bus);
    if (ret != ESP_OK) return ret;
 
    battery_struct_init(battery_data);

    calculate_charge_estimate(battery_data); // ret ?
    ESP_LOGI(TAG, "Estimated Battery Charge: %.2fmAh: %d%%", battery_data->charge_mah, get_battery_charge_pct(battery_data));
    xTaskCreate(battery_monitor_task, "battery_task", 4096, battery_data, 3, NULL); // ret ?

    return ESP_OK;
}