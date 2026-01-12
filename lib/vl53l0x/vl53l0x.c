#include "vl53l0x.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM 0
#define I2C_MASTER_TIMEOUT_MS 1000

// --- Low Level I2C Helpers ---
static esp_err_t write_byte(uint8_t reg, uint8_t value) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t read_byte(uint8_t reg, uint8_t *value) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t read_multi(uint8_t reg, uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1)
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

// --- The Magic Init Sequence (Taken from Pololu/ST API) ---
esp_err_t vl53l0x_init(void) {
  uint8_t val;

  // Check ID
  if (read_byte(0xC0, &val) != ESP_OK)
    return ESP_FAIL;
  if (val != 0xEE)
    return ESP_FAIL; // Wrong ID

  // 1. Data Init (Standard I2C to set 2.8V mode)
  write_byte(0x88, 0x00);
  write_byte(0x80, 0x01);
  write_byte(0xFF, 0x01);
  write_byte(0x00, 0x00);
  read_byte(0x91, &val); // Stop variable
  write_byte(0x00, 0x01);
  write_byte(0xFF, 0x00);
  write_byte(0x80, 0x00);

  // 2. Load Tuning Settings (Proprietary ST Blob)
  write_byte(0xFF, 0x01);
  write_byte(0x00, 0x00);
  write_byte(0xFF, 0x00);
  write_byte(0x09, 0x00);
  write_byte(0x10, 0x00);
  write_byte(0x11, 0x00);
  write_byte(0x24, 0x01);
  write_byte(0x25, 0xFF);
  write_byte(0x75, 0x00);
  write_byte(0xFF, 0x01);
  write_byte(0x4E, 0x2C);
  write_byte(0x48, 0x00);
  write_byte(0x30, 0x20);

  write_byte(0xFF, 0x00);
  write_byte(0x30, 0x09);
  write_byte(0x54, 0x00);
  write_byte(0x31, 0x04);
  write_byte(0x32, 0x03);
  write_byte(0x40, 0x83);
  write_byte(0x46, 0x25);
  write_byte(0x60, 0x00);
  write_byte(0x27, 0x00);
  write_byte(0x50, 0x06);
  write_byte(0x51, 0x00);
  write_byte(0x52, 0x96);
  write_byte(0x56, 0x08);
  write_byte(0x57, 0x30);
  write_byte(0x61, 0x00);
  write_byte(0x62, 0x00);
  write_byte(0x64, 0x00);
  write_byte(0x65, 0x00);
  write_byte(0x66, 0xA0);

  write_byte(0xFF, 0x01);
  write_byte(0x22, 0x32);
  write_byte(0x47, 0x14);
  write_byte(0x49, 0xFF);
  write_byte(0x4A, 0x00);

  write_byte(0xFF, 0x00);
  write_byte(0x7A, 0x0A);
  write_byte(0x7B, 0x00);
  write_byte(0x78, 0x21);

  write_byte(0xFF, 0x01);
  write_byte(0x23, 0x34);
  write_byte(0x42, 0x00);
  write_byte(0x44, 0xFF);
  write_byte(0x45, 0x26);
  write_byte(0x46, 0x05);
  write_byte(0x40, 0x40);
  write_byte(0x0E, 0x06);
  write_byte(0x20, 0x1A);
  write_byte(0x43, 0x40);

  write_byte(0xFF, 0x00);
  write_byte(0x34, 0x03);
  write_byte(0x35, 0x44);

  write_byte(0xFF, 0x01);
  write_byte(0x31, 0x04);
  write_byte(0x4B, 0x09);
  write_byte(0x4C, 0x05);
  write_byte(0x4D, 0x04);

  write_byte(0xFF, 0x00);
  write_byte(0x44, 0x00);
  write_byte(0x45, 0x20);
  write_byte(0x47, 0x08);
  write_byte(0x48, 0x28);
  write_byte(0x67, 0x00);
  write_byte(0x70, 0x04);
  write_byte(0x71, 0x01);
  write_byte(0x72, 0xFE);
  write_byte(0x76, 0x00);
  write_byte(0x77, 0x00);

  write_byte(0xFF, 0x01);
  write_byte(0x0D, 0x01);

  write_byte(0xFF, 0x00);
  write_byte(0x80, 0x01);
  write_byte(0x01, 0xF8);

  write_byte(0xFF, 0x01);
  write_byte(0x8E, 0x01);
  write_byte(0x00, 0x01);
  write_byte(0xFF, 0x00);
  write_byte(0x80, 0x00);

  // 3. Start Continuous Measurement
  write_byte(0x00,
             0x02); // REG_SYSRANGE_START (Bit 1 = Continuous / Back-to-back)

  return ESP_OK;
}

uint16_t vl53l0x_read_range_mm(void) {
  uint8_t buf[12];
  // Check if measurement is ready (Bit 0 of 0x13)
  // read_byte(0x13, &val); // Interrupt Status
  // if (!(val & 0x07)) return 8190; // Not ready

  // Read result
  // The result registers for range are 0x1E (High) and 0x1F (Low).
  // BUT we need to read a block to be safe.
  // 0x14 + 10 = 0x1E.

  // Read 2 bytes from 0x1E
  if (read_multi(0x1E, buf, 2) != ESP_OK)
    return 8191;

  uint16_t range = (buf[0] << 8) | buf[1];

  if (range < 20)
    range = 0; // Filter noise floor

  // Clear Interrupt
  write_byte(0x0B, 0x01);

  return range;
}
