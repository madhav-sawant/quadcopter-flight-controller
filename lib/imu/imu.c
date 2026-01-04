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
#define DLPF_CFG_98HZ                                                          \
  0x02 // 98Hz bandwidth (~4ms delay) - FAST for responsive control!
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

#define COMPLEMENTARY_ALPHA                                                    \
  0.995f // 99.5% gyro - minimizes accel EMI from receiver
#define GYRO_SCALE_FACTOR 16.4f
#define ACCEL_SCALE_FACTOR 4096.0f
#define RAD_TO_DEG 57.2957795f

// Software low-pass filter coefficients
// UPDATED 2024-12-30: Strengthened filtering to reject F450 motor vibration
// Alpha = 0.1 means only 10% of new noisy sample is used, 90% is old smoothed
// value.
#define GYRO_LPF_ALPHA                                                         \
  0.40f // Stronger filtering for high-gain/vibration setup (was 0.70)
#define ACCEL_LPF_ALPHA 0.10f // Strong filtering for Accel (was 0.90)

static imu_data_t imu_state;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static float accel_offset_pitch = 0.0f;
static float accel_offset_roll = 0.0f;

// Accelerometer scale calibration (1.0 / measured_z_when_level)
// Default 1.0 = no correction. Calculated during calibration.
static float accel_scale_factor = 1.0f;

// Manual trim offsets for PHYSICAL sensor mounting angle
// These compensate for MPU6050 not being perfectly level on the frame
// Positive PITCH = compensate for forward tilt (nose down mounting)
// Positive ROLL = compensate for right wing low mounting
// Trims set to 0.0 - Calibrate via Dashboard!
#define PITCH_TRIM_DEG 0.0f
#define ROLL_TRIM_DEG 0.0f

// Filtered sensor values (software LPF on top of hardware DLPF)
static float gyro_filtered_x = 0.0f;
static float gyro_filtered_y = 0.0f;
static float gyro_filtered_z = 0.0f;
static float accel_filtered_x = 0.0f;
static float accel_filtered_y = 0.0f;
static float accel_filtered_z = 0.0f;
static bool sensor_filter_initialized = false;

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

  if ((ret = write_register(REG_CONFIG, DLPF_CFG_98HZ)) != ESP_OK)
    return ret;
  printf("IMU: DLPF set to 98Hz for ~4ms delay\n");
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
  float sum_z = 0; // For scale calibration
  uint8_t buffer[6];

  // Temporary state for calibration
  float ax_g, ay_g, az_g;

  printf("IMU: Calibrating accelerometer (1000 samples)...\n");

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
      sum_z += az_g; // Accumulate Z readings
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  accel_offset_pitch = sum_pitch / samples;
  accel_offset_roll = sum_roll / samples;

  // Calculate scale factor: when level, Z should read 1.0g
  // scale_factor = 1.0 / measured_average_z
  float avg_z = sum_z / samples;
  if (avg_z > 0.5f && avg_z < 1.5f) { // Sanity check
    accel_scale_factor = 1.0f / avg_z;
    printf("IMU: Accel scale factor = %.3f (measured Z = %.3fg)\n",
           accel_scale_factor, avg_z);
  } else {
    accel_scale_factor = 1.0f; // Default if values are weird
    printf(
        "IMU: Accel scale factor = 1.0 (default, Z = %.3fg was out of range)\n",
        avg_z);
  }
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

  // Raw sensor values
  // SENSOR ORIENTATION CORRECTION:
  // MPU6050 is mounted in STANDARD orientation: X+ axis pointing FORWARD.
  // No axis negation needed - sensor frame matches flight frame.
  // Standard: X=Forward, Y=Right, Z=Down (NED) or Z=Up (ENU depending on mount)
  float accel_raw_x = (ax_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float accel_raw_y = (ay_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float accel_raw_z = (az_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float gyro_raw_x = (gx_raw / GYRO_SCALE_FACTOR) - gyro_bias_x;
  float gyro_raw_y = (gy_raw / GYRO_SCALE_FACTOR) - gyro_bias_y;
  float gyro_raw_z = (gz_raw / GYRO_SCALE_FACTOR) - gyro_bias_z;

  // Apply software low-pass filters
  // Reduces motor vibration noise that passes through hardware DLPF
  if (!sensor_filter_initialized) {
    accel_filtered_x = accel_raw_x;
    accel_filtered_y = accel_raw_y;
    accel_filtered_z = accel_raw_z;
    gyro_filtered_x = gyro_raw_x;
    gyro_filtered_y = gyro_raw_y;
    gyro_filtered_z = gyro_raw_z;
    sensor_filter_initialized = true;
  } else {
    // IIR low-pass filter: filtered = alpha * new + (1-alpha) * old
    accel_filtered_x = ACCEL_LPF_ALPHA * accel_raw_x +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_x;
    accel_filtered_y = ACCEL_LPF_ALPHA * accel_raw_y +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_y;
    accel_filtered_z = ACCEL_LPF_ALPHA * accel_raw_z +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_z;
    gyro_filtered_x =
        GYRO_LPF_ALPHA * gyro_raw_x + (1.0f - GYRO_LPF_ALPHA) * gyro_filtered_x;
    gyro_filtered_y =
        GYRO_LPF_ALPHA * gyro_raw_y + (1.0f - GYRO_LPF_ALPHA) * gyro_filtered_y;
    gyro_filtered_z =
        GYRO_LPF_ALPHA * gyro_raw_z + (1.0f - GYRO_LPF_ALPHA) * gyro_filtered_z;
  }

  // Use filtered values
  imu_state.accel_x_g = accel_filtered_x;
  imu_state.accel_y_g = accel_filtered_y;
  imu_state.accel_z_g = accel_filtered_z;
  imu_state.gyro_x_dps = gyro_filtered_x;
  imu_state.gyro_y_dps = gyro_filtered_y;
  imu_state.gyro_z_dps = gyro_filtered_z;

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

  // Apply Calibration Offsets (from NVS calibration)
  accel_pitch -= accel_offset_pitch;
  accel_roll -= accel_offset_roll;

  // NOTE: Trim is now applied INSIDE the complementary filter
  // This ensures the full correction takes effect, not just 8%!
  // Old approach added trim to accel only, but 92% weight goes to gyro
  // which didn't know about the offset.

  // CRITICAL: On first read, initialize angles directly from accelerometer
  // This prevents slow convergence issues if drone is not perfectly level at
  // boot
  if (first_read) {
    // Apply trim to initial angle
    imu_state.pitch_deg = accel_pitch + PITCH_TRIM_DEG;
    imu_state.roll_deg = accel_roll + ROLL_TRIM_DEG;
    first_read = false;
    printf("IMU: Angles initialized - Roll: %.2f (trim: %.1f), Pitch: %.2f "
           "(trim: %.1f)\n",
           imu_state.roll_deg, ROLL_TRIM_DEG, imu_state.pitch_deg,
           PITCH_TRIM_DEG);
  } else {
    // Store previous angles for rate limiting
    float prev_pitch = imu_state.pitch_deg;
    float prev_roll = imu_state.roll_deg;

    // Complementary filter with TRIM applied to the accelerometer contribution
    // This way both gyro integration (which starts from trimmed initial angle)
    // and accel correction (which uses trimmed target) agree on what "level"
    // means
    float new_pitch =
        COMPLEMENTARY_ALPHA *
            (imu_state.pitch_deg + imu_state.gyro_y_dps * dt_sec) +
        (1.0f - COMPLEMENTARY_ALPHA) * (accel_pitch + PITCH_TRIM_DEG);
    float new_roll =
        COMPLEMENTARY_ALPHA *
            (imu_state.roll_deg + imu_state.gyro_x_dps * dt_sec) +
        (1.0f - COMPLEMENTARY_ALPHA) * (accel_roll + ROLL_TRIM_DEG);

    // ANGLE SPIKE FILTER: Limit rate of change to 300 deg/s
    // This prevents accelerometer vibration spikes from causing sudden angle
    // jumps At 250Hz loop, max change per cycle = 300 / 250 = 1.2 degrees
    const float MAX_ANGLE_RATE_DPS = 300.0f;
    float max_delta = MAX_ANGLE_RATE_DPS * dt_sec;

    float pitch_delta = new_pitch - prev_pitch;
    if (pitch_delta > max_delta)
      pitch_delta = max_delta;
    if (pitch_delta < -max_delta)
      pitch_delta = -max_delta;

    float roll_delta = new_roll - prev_roll;
    if (roll_delta > max_delta)
      roll_delta = max_delta;
    if (roll_delta < -max_delta)
      roll_delta = -max_delta;

    imu_state.pitch_deg = prev_pitch + pitch_delta;
    imu_state.roll_deg = prev_roll + roll_delta;
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

  // Load accel scale factor (optional - older calibrations may not have it)
  len = sizeof(float);
  if (nvs_get_blob(handle, "accel_scale", &accel_scale_factor, &len) !=
      ESP_OK) {
    accel_scale_factor = 1.0f; // Default if not found
    printf("IMU CAL: No accel scale factor found, using 1.0\n");
  }

  nvs_close(handle);

  if (success) {
    printf("IMU CAL: Loaded from NVS:\n");
    printf("  Gyro Bias: X=%.4f, Y=%.4f, Z=%.4f\n", gyro_bias_x, gyro_bias_y,
           gyro_bias_z);
    printf("  Accel Offset: Pitch=%.4f, Roll=%.4f\n", accel_offset_pitch,
           accel_offset_roll);
    printf("  Accel Scale Factor: %.4f\n", accel_scale_factor);
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

  // Save accel scale factor
  nvs_set_blob(handle, "accel_scale", &accel_scale_factor, sizeof(float));

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
  printf("  Accel Scale Factor: %.4f\n", accel_scale_factor);
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
