#ifndef VL53L0X_H
#define VL53L0X_H

#include "esp_err.h"
#include <stdint.h>

// VL53L0X I2C Address (Default 0x29)
#define VL53L0X_ADDR 0x29

/**
 * @brief Initialize the VL53L0X sensor
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t vl53l0x_init(void);

/**
 * @brief Read distance in millimeters
 *
 * @return uint16_t Distance in mm (MAX 8190), or 8190 if out of range/error
 */
uint16_t vl53l0x_read_range_mm(void);

#endif // VL53L0X_H
