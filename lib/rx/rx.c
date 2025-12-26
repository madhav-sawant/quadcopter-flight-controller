#include "rx.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static volatile uint16_t rx_channels[RX_CHANNEL_COUNT];
static volatile uint8_t current_channel = 0;
static volatile int64_t last_time_us = 0;
static volatile bool connected = false;
static volatile int64_t last_frame_time_us = 0;

// Interrupt Service Routine for PPM Pin
static void IRAM_ATTR rx_isr_handler(void *arg) {
  int64_t now = esp_timer_get_time();
  int64_t dt = now - last_time_us;
  last_time_us = now;

  // Sync Pulse Detection (Long pause between frames)
  if (dt > RX_SYNC_MIN_US) {
    current_channel = 0;
    connected = true;
    last_frame_time_us = now;
  }
  // Channel Pulse Processing
  else if (dt >= RX_MIN_US && dt <= RX_MAX_US) {
    if (current_channel < RX_CHANNEL_COUNT) {
      rx_channels[current_channel] = (uint16_t)dt;
      current_channel++;
    }
  }
}

void rx_init(void) {
  // Initialize channels to center/safe values
  for (int i = 0; i < RX_CHANNEL_COUNT; i++) {
    rx_channels[i] = 1500;
  }
  // Throttle (usually ch 2 or 3 depending on map) to 1000 for safety
  rx_channels[2] = 1000;

  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << RX_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type =
          GPIO_INTR_ANYEDGE // PPM pulses are defined by time between edges
                            // (usually rising to rising or falling to falling)
  };

  // Note: Standard PPM is often measured Rising-to-Rising or
  // Falling-to-Falling. If we use ANYEDGE, we might measure pulse width AND gap
  // width, which is PWM. PPM encodes data in the time *between* the same edge
  // of consecutive pulses. Let's assume standard PPM where we measure time
  // between Rising edges.
  io_conf.intr_type = GPIO_INTR_POSEDGE;

  gpio_config(&io_conf);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(RX_PIN, rx_isr_handler, NULL);
}

bool rx_is_connected(void) {
  // Check if we received a frame recently (e.g., within 100ms)
  if (esp_timer_get_time() - last_frame_time_us > 100000) {
    connected = false;
  }
  return connected;
}

uint16_t rx_get_channel(uint8_t channel_index) {
  if (channel_index >= RX_CHANNEL_COUNT)
    return 0;
  return rx_channels[channel_index];
}

void rx_get_all(uint16_t *channels) {
  for (int i = 0; i < RX_CHANNEL_COUNT; i++) {
    channels[i] = rx_channels[i];
  }
}
