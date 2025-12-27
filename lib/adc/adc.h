#ifndef ADC_H
#define ADC_H

#include "esp_adc/adc_oneshot.h"
#include <stdint.h>

// ADC1 Channel 7 (GPIO 35)
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_7
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH ADC_BITWIDTH_12

// Calibration Constants (Empirically determined)
#define VOLTAGE_SCALE_MV 41624
#define VOLTAGE_SCALE_DIV 10000
#define VOLTAGE_OFFSET_MV 180

void adc_init(void);
uint16_t adc_read_battery_voltg(void);

#endif // ADC_H
