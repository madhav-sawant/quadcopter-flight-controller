#include "adc.h"
#include "esp_log.h"

static adc_oneshot_unit_handle_t adc1_handle;

void adc_init(void) {
  // 1. Init ADC Unit
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  // 2. Config Channel
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_WIDTH,
      .atten = ADC_ATTEN,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));
}

uint16_t adc_read_raw(void) {
  int raw_val = 0;
  uint32_t sum = 0;

  // Average 64 samples
  for (int i = 0; i < 64; i++) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw_val));
    sum += raw_val;
  }

  return (uint16_t)(sum / 64);
}

uint16_t adc_read_voltage(uint16_t raw) {
  // Basic linear mapping for 12-bit ADC (0-4095) -> 0-3300mV (approx)
  // Note: This is a rough approximation. For precision, use esp_adc_cal
  // (calibration scheme). However, the user's existing code used this simple
  // formula, so we preserve it to keep calibration valid.
  uint16_t voltage = (raw * 3300) / 4095;
  return (uint16_t)voltage;
}

uint16_t adc_read_battery_voltg(void) {
  uint16_t raw = adc_read_raw();
  uint16_t adc_mv = adc_read_voltage(raw);

  // Apply voltage divider and offset correction
  uint32_t scaled_mv =
      ((uint16_t)adc_mv * VOLTAGE_SCALE_MV) / VOLTAGE_SCALE_DIV;

  if (scaled_mv < VOLTAGE_OFFSET_MV) {
    return 0;
  }

  uint32_t battery_mv = scaled_mv - VOLTAGE_OFFSET_MV;

  return (uint16_t)battery_mv;
}
