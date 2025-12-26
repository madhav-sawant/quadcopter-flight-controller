#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float roll_deg;
  float pitch_deg;
} imu_data_t;

esp_err_t imu_init(void);
void imu_read(float dt_sec);
const imu_data_t *imu_get_data(void);
void imu_calibrate_gyro(void);
void imu_calibrate_accel(void);

#endif // IMU_H
