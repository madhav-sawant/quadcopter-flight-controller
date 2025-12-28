#include "imu.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdio.h>

#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define BIT_CLKSEL_PLL_X 0x01
#define REG_CONFIG 0x1A
#define DLPF_CFG_20HZ 0x04 // 20Hz bandwidth - filters motor vibration noise
#define REG_GYRO_CONFIG 0x1B
#define FS_SEL_2000 0x18
#define REG_ACCEL_CONFIG 0x1C
#define AFS_SEL_8G 0x10
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43
#define REG_WHO_AM_I 0x75

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define COMPLEMENTARY_ALPHA 0.96f // Lower alpha = faster drift correction
#define GYRO_SCALE_FACTOR 16.4f
#define ACCEL_SCALE_FACTOR 4096.0f
#define RAD_TO_DEG 57.2957795f

static imu_data_t imu_state;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static float accel_offset_pitch = 0.0f;
static float accel_offset_roll = 0.0f;

static esp_err_t write_register(uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t read_registers(uint8_t start_reg, uint8_t *buffer,
                                size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, start_reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1)
    i2c_master_read(cmd, buffer, len - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, buffer + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t imu_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                     I2C_MASTER_TX_BUF_DISABLE, 0);

  // Check WHO_AM_I
  uint8_t who_am_i;
  if (read_registers(REG_WHO_AM_I, &who_am_i, 1) != ESP_OK) {
    printf("IMU: Failed to read WHO_AM_I register.\n");
    return ESP_FAIL;
  }
  if (who_am_i != MPU6050_ADDR) {
    printf("IMU: WHO_AM_I mismatch. Expected 0x%02x, got 0x%02x\n",
           MPU6050_ADDR, who_am_i);
    return ESP_FAIL;
  }
  printf("IMU: Found MPU6050 at 0x%02x\n", who_am_i);

  esp_err_t ret;
  if ((ret = write_register(REG_PWR_MGMT_1, BIT_CLKSEL_PLL_X)) != ESP_OK)
    return ret;

  // Verify PWR_MGMT_1
  uint8_t pwr_mgmt;
  read_registers(REG_PWR_MGMT_1, &pwr_mgmt, 1);
  printf("IMU: PWR_MGMT_1 written 0x%02x, read 0x%02x\n", BIT_CLKSEL_PLL_X,
         pwr_mgmt);

  if ((ret = write_register(REG_CONFIG, DLPF_CFG_20HZ)) != ESP_OK)
    return ret;
  printf("IMU: DLPF set to 20Hz for noise filtering\n");
  if ((ret = write_register(REG_GYRO_CONFIG, FS_SEL_2000)) != ESP_OK)
    return ret;
  if ((ret = write_register(REG_ACCEL_CONFIG, AFS_SEL_8G)) != ESP_OK)
    return ret;

  return ESP_OK;
}

void imu_calibrate_gyro(void) {
  const int samples = 1000;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  uint8_t buffer[6];

  for (int i = 0; i < samples; i++) {
    if (read_registers(REG_GYRO_XOUT_H, buffer, 6) == ESP_OK) {
      int16_t raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
      int16_t raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
      int16_t raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);
      sum_x += raw_x;
      sum_y += raw_y;
      sum_z += raw_z;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  gyro_bias_x = (sum_x / samples) / GYRO_SCALE_FACTOR;
  gyro_bias_y = (sum_y / samples) / GYRO_SCALE_FACTOR;
  gyro_bias_z = (sum_z / samples) / GYRO_SCALE_FACTOR;
}

void imu_calibrate_accel(void) {
  const int samples = 1000;
  float sum_pitch = 0;
  float sum_roll = 0;
  uint8_t buffer[6];

  // Temporary state for calibration
  float ax_g, ay_g, az_g;

  for (int i = 0; i < samples; i++) {
    if (read_registers(REG_ACCEL_XOUT_H, buffer, 6) == ESP_OK) {
      int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
      int16_t ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
      int16_t az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);

      ax_g = ax_raw / ACCEL_SCALE_FACTOR;
      ay_g = ay_raw / ACCEL_SCALE_FACTOR;
      az_g = az_raw / ACCEL_SCALE_FACTOR;

      float pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
      float roll = atan2f(ay_g, az_g) * RAD_TO_DEG;

      sum_pitch += pitch;
      sum_roll += roll;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  accel_offset_pitch = sum_pitch / samples;
  accel_offset_roll = sum_roll / samples;
}

void imu_read(float dt_sec) {
  static bool first_read = true; // Track first read for angle initialization

  uint8_t buffer[14];
  if (read_registers(REG_ACCEL_XOUT_H, buffer, 14) != ESP_OK) {
    printf("IMU: Read failed!\n");
    return;
  }

  int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
  int16_t gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
  int16_t gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

  imu_state.accel_x_g = ax_raw / ACCEL_SCALE_FACTOR;
  imu_state.accel_y_g = ay_raw / ACCEL_SCALE_FACTOR;
  imu_state.accel_z_g = az_raw / ACCEL_SCALE_FACTOR;
  imu_state.gyro_x_dps = (gx_raw / GYRO_SCALE_FACTOR) - gyro_bias_x;
  imu_state.gyro_y_dps = (gy_raw / GYRO_SCALE_FACTOR) - gyro_bias_y;
  imu_state.gyro_z_dps = (gz_raw / GYRO_SCALE_FACTOR) - gyro_bias_z;

  // ============================================================================
  // COMPLEMENTARY FILTER
  // Combines Gyro (fast, no drift short-term) + Accel (stable, noisy)
  // ============================================================================

  float accel_pitch = atan2f(imu_state.accel_x_g,
                             sqrtf(imu_state.accel_y_g * imu_state.accel_y_g +
                                   imu_state.accel_z_g * imu_state.accel_z_g)) *
                      RAD_TO_DEG;
  float accel_roll =
      atan2f(imu_state.accel_y_g, imu_state.accel_z_g) * RAD_TO_DEG;

  // Apply Calibration Offsets
  accel_pitch -= accel_offset_pitch;
  accel_roll -= accel_offset_roll;

  // CRITICAL: On first read, initialize angles directly from accelerometer
  // This prevents slow convergence issues if drone is not perfectly level at
  // boot
  if (first_read) {
    imu_state.pitch_deg = accel_pitch;
    imu_state.roll_deg = accel_roll;
    first_read = false;
    printf("IMU: Angles initialized from accel - Roll: %.2f, Pitch: %.2f\n",
           accel_roll, accel_pitch);
  } else {
    // Normal complementary filter operation
    imu_state.pitch_deg =
        COMPLEMENTARY_ALPHA *
            (imu_state.pitch_deg + imu_state.gyro_y_dps * dt_sec) +
        (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
    imu_state.roll_deg = COMPLEMENTARY_ALPHA * (imu_state.roll_deg +
                                                imu_state.gyro_x_dps * dt_sec) +
                         (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;
  }
}

const imu_data_t *imu_get_data(void) { return &imu_state; }

/* -------------------------------------------------------------------------- */
/*                   NVS Calibration Storage                                  */
/* -------------------------------------------------------------------------- */

// Use separate namespace from PID config to avoid conflicts
#define NVS_IMU_CAL_NAMESPACE "imu_cal"

// Magic number to verify calibration validity
#define IMU_CAL_MAGIC 0xCAFE1234

bool imu_calibration_load_from_nvs(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_IMU_CAL_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    printf("IMU CAL: No calibration data found in NVS.\n");
    return false;
  }

  // Verify magic number
  uint32_t magic = 0;
  size_t len = sizeof(uint32_t);
  err = nvs_get_blob(handle, "magic", &magic, &len);
  if (err != ESP_OK || magic != IMU_CAL_MAGIC) {
    printf("IMU CAL: Invalid calibration data in NVS.\n");
    nvs_close(handle);
    return false;
  }

  bool success = true;

  // Load gyro bias values
  len = sizeof(float);
  if (nvs_get_blob(handle, "gyro_bias_x", &gyro_bias_x, &len) != ESP_OK)
    success = false;
  len = sizeof(float);
  if (nvs_get_blob(handle, "gyro_bias_y", &gyro_bias_y, &len) != ESP_OK)
    success = false;
  len = sizeof(float);
  if (nvs_get_blob(handle, "gyro_bias_z", &gyro_bias_z, &len) != ESP_OK)
    success = false;

  // Load accel offset values
  len = sizeof(float);
  if (nvs_get_blob(handle, "accel_off_p", &accel_offset_pitch, &len) != ESP_OK)
    success = false;
  len = sizeof(float);
  if (nvs_get_blob(handle, "accel_off_r", &accel_offset_roll, &len) != ESP_OK)
    success = false;

  nvs_close(handle);

  if (success) {
    printf("IMU CAL: Loaded from NVS:\n");
    printf("  Gyro Bias: X=%.4f, Y=%.4f, Z=%.4f\n", gyro_bias_x, gyro_bias_y,
           gyro_bias_z);
    printf("  Accel Offset: Pitch=%.4f, Roll=%.4f\n", accel_offset_pitch,
           accel_offset_roll);
  } else {
    printf("IMU CAL: Failed to load some calibration values.\n");
  }

  return success;
}

void imu_calibration_save_to_nvs(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_IMU_CAL_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    printf("IMU CAL: Failed to open NVS for writing.\n");
    return;
  }

  // Save gyro bias values
  nvs_set_blob(handle, "gyro_bias_x", &gyro_bias_x, sizeof(float));
  nvs_set_blob(handle, "gyro_bias_y", &gyro_bias_y, sizeof(float));
  nvs_set_blob(handle, "gyro_bias_z", &gyro_bias_z, sizeof(float));

  // Save accel offset values
  nvs_set_blob(handle, "accel_off_p", &accel_offset_pitch, sizeof(float));
  nvs_set_blob(handle, "accel_off_r", &accel_offset_roll, sizeof(float));

  // Write magic number last to confirm all data was saved
  uint32_t magic = IMU_CAL_MAGIC;
  nvs_set_blob(handle, "magic", &magic, sizeof(uint32_t));

  nvs_commit(handle);
  nvs_close(handle);

  printf("IMU CAL: Saved to NVS:\n");
  printf("  Gyro Bias: X=%.4f, Y=%.4f, Z=%.4f\n", gyro_bias_x, gyro_bias_y,
         gyro_bias_z);
  printf("  Accel Offset: Pitch=%.4f, Roll=%.4f\n", accel_offset_pitch,
         accel_offset_roll);
}

/* -------------------------------------------------------------------------- */
/*                   Calibration Access Functions */
/* -------------------------------------------------------------------------- */

void imu_get_calibration(imu_calibration_t *cal) {
  if (cal == NULL)
    return;
  cal->gyro_bias_x = gyro_bias_x;
  cal->gyro_bias_y = gyro_bias_y;
  cal->gyro_bias_z = gyro_bias_z;
  cal->accel_offset_pitch = accel_offset_pitch;
  cal->accel_offset_roll = accel_offset_roll;
}

void imu_print_calibration(void) {
  printf("\n========================================\n");
  printf("       IMU CALIBRATION VALUES\n");
  printf("========================================\n");
  printf("Gyro Bias (deg/s):\n");
  printf("  X: %+.4f\n", gyro_bias_x);
  printf("  Y: %+.4f\n", gyro_bias_y);
  printf("  Z: %+.4f\n", gyro_bias_z);
  printf("----------------------------------------\n");
  printf("Accel Offset (degrees):\n");
  printf("  Pitch: %+.4f\n", accel_offset_pitch);
  printf("  Roll:  %+.4f\n", accel_offset_roll);
  printf("========================================\n\n");
}
