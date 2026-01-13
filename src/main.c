/**
 * VL53L0X & BMP280 Sensor Display - ESP32
 * Shows distance (mm) and pressure/altitude
 */
#include "../lib/baro/baro.h"
#include "../lib/vl53l0x/vl53l0x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define TAG "MAIN"

static vl53l0x_t vl53_sensor;

void app_main(void) {
  ESP_LOGI(TAG, "--- Sensor Display Demo ---");

  // 1. Initialize I2C (shared bus)
  // vl53l0x_i2c_init() installs the I2C driver on I2C_NUM_0
  if (vl53l0x_i2c_init() != ESP_OK) {
    ESP_LOGE(TAG, "I2C Init Failed");
    return;
  }

  // 2. Initialize VL53L0X Laser Sensor
  if (vl53l0x_init(&vl53_sensor) != ESP_OK) {
    ESP_LOGE(TAG, "VL53L0X Init Failed");
  } else {
    ESP_LOGI(TAG, "VL53L0X Init Success");
    vl53l0x_start_continuous(&vl53_sensor);
  }

  // 3. Initialize BMP280 Barometer
  // baro_init() assumes I2C driver is already installed (which we did above)
  if (baro_init() != ESP_OK) {
    ESP_LOGE(TAG, "Baro Init Failed");
  } else {
    ESP_LOGI(TAG, "Baro Init Success");
  }

  // 4. Calibrate Barometer (Zeroing)
  ESP_LOGI(TAG, "Calibrating Barometer... Keep Still!");
  float start_alt_sum = 0.0f;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    baro_read();
    start_alt_sum += baro_get_data()->altitude_m;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  float ground_alt = start_alt_sum / samples;
  ESP_LOGI(TAG, "Calibration Done. Ground Alt: %.2f m", ground_alt);

  ESP_LOGI(TAG, "Starting Measurement Loop...");

  while (1) {
    // Read VL53L0X
    uint16_t dist_mm = 0;
    bool laser_ok = (vl53l0x_read_continuous(&vl53_sensor, &dist_mm) == ESP_OK);

    // Read Barometer
    baro_read();
    const baro_data_t *baro = baro_get_data();
    float relative_alt = baro->altitude_m - ground_alt;

    // Display Output
    printf("\033[2J\033[H"); // Clear Screen
    printf("--- Sensor Readings ---\n");

    // Laser Output
    if (laser_ok) {
      if (dist_mm > 0 && dist_mm < 8190) {
        printf("LASER: %4u mm  (%4.1f cm)\n", dist_mm, dist_mm / 10.0f);
      } else {
        printf("LASER: Out of Range\n");
      }
    } else {
      printf("LASER: Error\n");
    }

    // Baro Output
    printf("BARO : %6.1f Pa  |  Rel Alt: %6.1f m  |  Temp: %4.1f C\n",
           baro->pressure_pa, relative_alt, baro->temperature_c);

    printf("-----------------------\n");
    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz update
  }
}
