/**
 * @file read_imu_calibration.c
 * @brief Standalone tool to read and display IMU calibration values from NVS
 *
 * PURPOSE:
 * This tool reads the stored IMU calibration values from NVS (non-volatile
 * storage) and displays them without interfering with the main flight control
 * loop. Run this as a separate firmware to verify calibration is correct.
 *
 * BUILD:
 * Place this in src/main.c temporarily, or configure platformio.ini to build
 * this as a separate environment target.
 *
 * USAGE:
 * Flash this firmware, open serial monitor, and view calibration values.
 * The tool also shows current sensor readings for comparison.
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdio.h>

#include "../lib/imu/imu.h"

#define LED_PIN 2
#define SAMPLES_TO_AVERAGE 100

void app_main(void) {
  // Initialize LED for status
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0);

  printf("\n");
  printf("========================================\n");
  printf("   IMU CALIBRATION READER TOOL\n");
  printf("========================================\n\n");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  printf("[OK] NVS Initialized\n");

  // Initialize IMU
  if (imu_init() != ESP_OK) {
    printf("[ERROR] IMU Init Failed!\n");
    while (1) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  printf("[OK] IMU Initialized\n\n");

  // Try to load calibration from NVS
  printf("Loading calibration from NVS...\n\n");
  bool cal_loaded = imu_calibration_load_from_nvs();

  if (cal_loaded) {
    gpio_set_level(LED_PIN, 1); // Solid LED = calibration loaded

    // Get and display calibration values
    imu_calibration_t cal;
    imu_get_calibration(&cal);

    printf("\n========================================\n");
    printf("   STORED CALIBRATION VALUES\n");
    printf("========================================\n");
    printf("\n");
    printf("Gyro Bias (deg/s) - Subtracted from readings:\n");
    printf("  X (Roll):  %+8.4f deg/s\n", cal.gyro_bias_x);
    printf("  Y (Pitch): %+8.4f deg/s\n", cal.gyro_bias_y);
    printf("  Z (Yaw):   %+8.4f deg/s\n", cal.gyro_bias_z);
    printf("\n");
    printf("Accel Offset (degrees) - Subtracted from angle:\n");
    printf("  Roll:  %+8.4f degrees\n", cal.accel_offset_roll);
    printf("  Pitch: %+8.4f degrees\n", cal.accel_offset_pitch);
    printf("\n");
    printf("========================================\n");

    // Interpretation
    printf("\nINTERPRETATION:\n");
    printf("----------------------------------------\n");
    if (fabsf(cal.accel_offset_roll) > 3.0f ||
        fabsf(cal.accel_offset_pitch) > 3.0f) {
      printf("[WARNING] Large accel offsets detected!\n");
      printf("  This may indicate calibration was done on\n");
      printf("  a non-level surface. Consider recalibrating.\n");
    } else if (fabsf(cal.accel_offset_roll) > 1.0f ||
               fabsf(cal.accel_offset_pitch) > 1.0f) {
      printf("[INFO] Moderate accel offsets.\n");
      printf("  Acceptable for most flight, but a level\n");
      printf("  recalibration could improve performance.\n");
    } else {
      printf("[GOOD] Calibration values look reasonable.\n");
    }
    printf("\n");

  } else {
    printf("\n[WARNING] No calibration data found in NVS!\n");
    printf("The drone will use fresh calibration on boot.\n\n");
  }

  // Now read and display CURRENT sensor values
  printf("========================================\n");
  printf("   CURRENT SENSOR READINGS\n");
  printf("========================================\n");
  printf("Averaging %d samples... ", SAMPLES_TO_AVERAGE);
  fflush(stdout);

  float avg_roll = 0, avg_pitch = 0;
  float avg_gx = 0, avg_gy = 0, avg_gz = 0;

  for (int i = 0; i < SAMPLES_TO_AVERAGE; i++) {
    imu_read(0.004f); // 4ms dt
    const imu_data_t *imu = imu_get_data();
    avg_roll += imu->roll_deg;
    avg_pitch += imu->pitch_deg;
    avg_gx += imu->gyro_x_dps;
    avg_gy += imu->gyro_y_dps;
    avg_gz += imu->gyro_z_dps;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  avg_roll /= SAMPLES_TO_AVERAGE;
  avg_pitch /= SAMPLES_TO_AVERAGE;
  avg_gx /= SAMPLES_TO_AVERAGE;
  avg_gy /= SAMPLES_TO_AVERAGE;
  avg_gz /= SAMPLES_TO_AVERAGE;

  printf("Done!\n\n");
  printf("After Calibration Applied:\n");
  printf("  Roll:  %+8.2f degrees (should be ~0 when level)\n", avg_roll);
  printf("  Pitch: %+8.2f degrees (should be ~0 when level)\n", avg_pitch);
  printf("  Gyro X: %+8.2f deg/s (should be ~0 when still)\n", avg_gx);
  printf("  Gyro Y: %+8.2f deg/s (should be ~0 when still)\n", avg_gy);
  printf("  Gyro Z: %+8.2f deg/s (should be ~0 when still)\n", avg_gz);
  printf("\n");

  // Check if current readings indicate problems
  printf("DIAGNOSIS:\n");
  printf("----------------------------------------\n");
  if (fabsf(avg_roll) > 3.0f || fabsf(avg_pitch) > 3.0f) {
    printf("[PROBLEM] Angles are NOT near zero!\n");
    printf("  The drone thinks it's tilted when it may be level.\n");
    printf("  SOLUTION: Recalibrate IMU on a flat, level surface.\n");
  } else if (fabsf(avg_roll) > 1.0f || fabsf(avg_pitch) > 1.0f) {
    printf("[MINOR] Small offset remaining.\n");
    printf("  Acceptable, but recalibration may improve stability.\n");
  } else {
    printf("[GOOD] Angle readings are near zero. Calibration looks OK.\n");
  }

  if (fabsf(avg_gx) > 1.0f || fabsf(avg_gy) > 1.0f || fabsf(avg_gz) > 1.0f) {
    printf("[WARNING] Gyro readings not near zero.\n");
    printf("  Make sure the drone is completely still.\n");
  }

  printf("\n========================================\n");
  printf("          TOOL COMPLETE\n");
  printf("========================================\n");

  // Continuous monitoring loop
  printf("\nStarting continuous monitoring (Ctrl+C to stop)...\n");
  printf("Roll    | Pitch   | Gyro X  | Gyro Y  | Gyro Z\n");
  printf("--------|---------|---------|---------|--------\n");

  while (1) {
    imu_read(0.02f);
    const imu_data_t *imu = imu_get_data();
    printf("\r%+7.2f | %+7.2f | %+7.2f | %+7.2f | %+7.2f", imu->roll_deg,
           imu->pitch_deg, imu->gyro_x_dps, imu->gyro_y_dps, imu->gyro_z_dps);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
