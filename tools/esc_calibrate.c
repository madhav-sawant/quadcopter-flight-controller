/**
 * @file main.c
 * @brief ESC Calibration Program
 *
 * ESC CALIBRATION PROCEDURE:
 * ==========================
 * 1. Upload this code to ESP32
 * 2. DISCONNECT the LiPo battery from the ESCs
 * 3. Open serial monitor
 * 4. Follow the on-screen countdown prompts
 *
 * MOTOR PINS:
 * - Motor 1: GPIO 12
 * - Motor 2: GPIO 13
 * - Motor 3: GPIO 14
 * - Motor 4: GPIO 27
 */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/*                               Configuration                                */
/* -------------------------------------------------------------------------- */

#define PWM_FREQ_HZ 500
#define PWM_RES_BIT 12
#define PWM_MOTOR_COUNT 4

#define PWM_MIN_US 1000
#define PWM_MAX_US 2000

#define LED_PIN 2

// Motor GPIO pins
static const int motor_gpios[PWM_MOTOR_COUNT] = {12, 13, 14, 27};
static const ledc_channel_t motor_channels[PWM_MOTOR_COUNT] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

/* -------------------------------------------------------------------------- */
/*                              PWM Functions                                 */
/* -------------------------------------------------------------------------- */

static void pwm_init(void) {
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = (ledc_timer_bit_t)PWM_RES_BIT,
      .freq_hz = PWM_FREQ_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&ledc_timer);

  for (int i = 0; i < PWM_MOTOR_COUNT; i++) {
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = motor_channels[i],
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor_gpios[i],
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
  }
}

static void set_all_motors(uint32_t pulse_us) {
  uint32_t max_duty = (1 << PWM_RES_BIT) - 1;
  uint32_t duty =
      (uint32_t)(((uint64_t)pulse_us * (uint64_t)max_duty * PWM_FREQ_HZ) /
                 1000000ULL);

  for (int i = 0; i < PWM_MOTOR_COUNT; i++) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor_channels[i], duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor_channels[i]);
  }
}

/* -------------------------------------------------------------------------- */
/*                              LED Functions                                 */
/* -------------------------------------------------------------------------- */

static void led_init(void) {
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

static void led_blink(int times, int delay_ms) {
  for (int i = 0; i < times; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  }
}

/* -------------------------------------------------------------------------- */
/*                              Helper Functions                              */
/* -------------------------------------------------------------------------- */

static void countdown(int seconds, const char *message) {
  printf("\n%s\n", message);
  for (int i = seconds; i > 0; i--) {
    printf("   %d seconds remaining...\n", i);
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  printf("   GO!\n\n");
}

/* -------------------------------------------------------------------------- */
/*                                 Main Entry                                 */
/* -------------------------------------------------------------------------- */

void app_main(void) {
  // Wait for serial connection
  vTaskDelay(pdMS_TO_TICKS(3000));

  printf("\n\n");
  printf("╔════════════════════════════════════════╗\n");
  printf("║      ESC CALIBRATION PROGRAM           ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");
  printf("Motor Pins:\n");
  printf("  Motor 1: GPIO 12\n");
  printf("  Motor 2: GPIO 13\n");
  printf("  Motor 3: GPIO 14\n");
  printf("  Motor 4: GPIO 27\n");
  printf("\n");

  // Initialize
  led_init();
  pwm_init();

  // =========================================================================
  // STEP 0: Safety Warning
  // =========================================================================
  printf("╔════════════════════════════════════════╗\n");
  printf("║  ⚠️  SAFETY CHECK - READ CAREFULLY!     ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");
  printf("   1. REMOVE ALL PROPELLERS!\n");
  printf("   2. DISCONNECT LIPO BATTERY NOW!\n");
  printf("   3. Have the LiPo ready to connect\n");
  printf("\n");

  countdown(10, "Preparing ESC calibration in:");

  // =========================================================================
  // STEP 1: Set MAX throttle (2000us) BEFORE powering ESC
  // =========================================================================
  printf("╔════════════════════════════════════════╗\n");
  printf("║  STEP 1: MAX THROTTLE SET (2000us)     ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");

  set_all_motors(PWM_MAX_US);
  printf("   ✓ PWM Output: 2000us (MAX THROTTLE)\n");
  printf("   ✓ All 4 motors outputting MAX signal\n");
  printf("\n");

  gpio_set_level(LED_PIN, 1); // LED ON = MAX throttle active

  // =========================================================================
  // STEP 2: Wait for user to connect battery
  // =========================================================================
  printf("╔════════════════════════════════════════╗\n");
  printf("║  STEP 2: CONNECT LIPO BATTERY NOW!     ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");
  printf("   >>> CONNECT THE LIPO TO ESCs NOW! <<<\n");
  printf("\n");
  printf("   You should hear: BEEP-BEEP\n");
  printf("   (This means ESC detected high throttle)\n");
  printf("\n");

  countdown(15, "You have 15 seconds to connect the battery:");

  // =========================================================================
  // STEP 3: Wait for calibration beeps
  // =========================================================================
  printf("╔════════════════════════════════════════╗\n");
  printf("║  STEP 3: WAITING FOR ESC...            ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");
  printf("   Waiting for ESC calibration beeps...\n");
  printf("\n");

  countdown(5, "Switching to MIN throttle in:");

  // =========================================================================
  // STEP 4: Set MIN throttle (1000us)
  // =========================================================================
  printf("╔════════════════════════════════════════╗\n");
  printf("║  STEP 4: MIN THROTTLE SET (1000us)     ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");

  set_all_motors(PWM_MIN_US);
  printf("   ✓ PWM Output: 1000us (MIN THROTTLE)\n");
  printf("\n");
  printf("   You should hear: Confirmation beeps\n");
  printf("   (This means calibration is stored!)\n");
  printf("\n");

  led_blink(5, 100); // 5 fast blinks = MIN throttle set
  gpio_set_level(LED_PIN, 0);

  // =========================================================================
  // STEP 5: Disarm - Keep MIN for 5 seconds
  // =========================================================================
  printf("╔════════════════════════════════════════╗\n");
  printf("║  STEP 5: HOLDING MIN THROTTLE          ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");

  countdown(5, "Holding MIN throttle:");

  // =========================================================================
  // COMPLETE
  // =========================================================================
  printf("\n");
  printf("╔════════════════════════════════════════╗\n");
  printf("║  ✓ CALIBRATION COMPLETE!               ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\n");
  printf("   ESC calibration saved to EEPROM.\n");
  printf("\n");
  printf("   Next steps:\n");
  printf("   1. Disconnect LiPo battery\n");
  printf("   2. Re-flash flight controller firmware\n");
  printf("   3. Test each motor individually\n");
  printf("\n");
  printf("   Holding MIN throttle. Safe to disconnect.\n");
  printf("\n");

  // Keep outputting MIN throttle and blink LED slowly
  while (1) {
    set_all_motors(PWM_MIN_US);
    led_blink(1, 2000);
    printf("   [Still holding MIN throttle - 1000us]\n");
  }
}
