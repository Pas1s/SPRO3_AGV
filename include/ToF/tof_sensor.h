#include "esp_err.h"
#include "stdbool.h"
#include "vl53l5cx_api.h"
#include "motor_control/motor_control.h"
#define LEFT_LPn_GPIO   GPIO_NUM_2   
#define RIGHT_LPn_GPIO  GPIO_NUM_41   
#define CENTER_LPn_GPIO GPIO_NUM_42
#define TOF_INT_LEFT_GPIO   GPIO_NUM_37   
#define TOF_INT_RIGHT_GPIO  GPIO_NUM_35   
#define TOF_INT_CENTER_GPIO GPIO_NUM_36

// set by ISR when INT pin goes low
extern volatile bool ToF_flag_center;
extern volatile bool ToF_flag_right;
extern volatile bool ToF_flag_left;
extern volatile bool ToF_on;


esp_err_t init_ToF_sensor(motors_t *motors);
esp_err_t vl53l5cx_read_all_zones(void);
esp_err_t vl53l5cx_set_zone_dual_thresholds(VL53L5CX_Configuration *dev, const uint16_t *low_mm, 
        const uint16_t *high_mm, uint8_t nb_zones);
void configure_thresholds(const int sensor, const uint16_t *h_threshold, const uint16_t * l_threshold);