/**
 * @file main.c
 * @brief Rate PID & Mixer Validation Firmware
 *
 * PURPOSE:
 * Validate the INNER (RATE) PID control loop and Motor Mixing.
 *
 * TEST PROCEDURE (NO PROPS!):
 * 1. Power on.
 * 2. System initializes (IMU, PWM, ADC).
 * 3. Gyro calibrates (Keep still).
 * 4. Safety Check: Battery voltage.
 * 5. ARMING SEQUENCE:
 *    - Wait 5 seconds.
 *    - Arm motors at IDLE speed.
 * 6. LOOP:
 *    - Read Gyro.
 *    - Run Rate PID (Target = 0 deg/s).
 *    - Mix outputs (Throttle = Idle).
 *    - Send to Motors.
 *    - Verify motors react to movement (correct direction).
 *
 * SAFETY:
 * - Low battery cutoff.
 * - Crash angle protection (disarm if > 60 deg tilt).
 * - Kill switch (Boot button GPIO 0).
 */

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../lib/config/config.h"
#include "../lib/rx/rx.h"
#include "../lib/webserver/webserver.h"
#include "adc.h"
#include "angle_control.h"
#include "imu.h"
#include "mixer.h"
#include "pwm.h"
#include "rate_control.h"

/* -------------------------------------------------------------------------- */
/*                               Configuration                                */
/* -------------------------------------------------------------------------- */

// Status LED
#define LED_PIN 2
#define BUTTON_PIN 0 // Boot button for emergency stop

// Control loop frequency (500 Hz)
#define CONTROL_LOOP_FREQ_HZ 500
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ)

// Debug print frequency (10 Hz)
#define DEBUG_PRINT_DIVIDER 50

// RC Mapping
#define RC_MAX_ANGLE_DEG 45.0f
#define RC_MAX_YAW_RATE_DPS 180.0f
#define RC_DEADBAND_US 20

/* -------------------------------------------------------------------------- */
/*                               Global State                                 */
/* -------------------------------------------------------------------------- */

static volatile bool control_loop_flag = false;
static int debug_counter = 0;

// System State
static bool system_armed = false;
static bool error_state = false;

// Debug data
static float debug_gyro[3];
static float debug_angle[2]; // Roll, Pitch
static float debug_pid[3];
static uint16_t debug_motors[4];
static uint16_t debug_rc[5]; // Roll, Pitch, Thr, Yaw, Aux1
static uint16_t debug_vbat = 0;
static int64_t debug_exec_time_us = 0;

/* -------------------------------------------------------------------------- */
/*                          Control Loop (500 Hz Timer)                       */
/* -------------------------------------------------------------------------- */

static void control_loop_callback(void *arg) {
  (void)arg;

  int64_t start_time = esp_timer_get_time();

  // 1. Read IMU
  imu_read(1.0f / CONTROL_LOOP_FREQ_HZ);
  const imu_data_t *imu = imu_get_data();

  // 2. Safety Checks (Crash detection)
  // TEST MODE: RESTORED CRASH CHECK
  // Fused angle is stable, so we can safely use this now.
  if (fabs(imu->roll_deg) > sys_cfg.crash_angle_deg ||
      fabs(imu->pitch_deg) > sys_cfg.crash_angle_deg) {
    mixer_arm(false);
    system_armed = false;
    error_state = true;
  } else {
    // Auto-reset error state if level and disarmed
    if (error_state && !system_armed && fabs(imu->roll_deg) < 20.0f &&
        fabs(imu->pitch_deg) < 20.0f) {
      error_state = false;
    }
  }

  // 3. Update Angle & Rate PID Controllers

  // Read RX Channels (AETR: 0=Roll, 1=Pitch, 2=Throttle, 3=Yaw)
  uint16_t rx_roll = rx_get_channel(0);
  uint16_t rx_pitch = rx_get_channel(1);
  uint16_t rx_thr = rx_get_channel(2);
  uint16_t rx_yaw = rx_get_channel(3);

  // Map Roll/Pitch (Angle Mode)
  float target_roll = 0.0f;
  if (abs(rx_roll - 1500) > RC_DEADBAND_US) {
    target_roll = (float)(rx_roll - 1500) / 500.0f * RC_MAX_ANGLE_DEG;
  }

  float target_pitch = 0.0f;
  if (abs(rx_pitch - 1500) > RC_DEADBAND_US) {
    // Invert pitch if necessary (Forward stick = Lower PWM? Check radio)
    // Assuming Forward = High PWM -> Pitch Down (Negative Angle)
    // Standard: Low=Pitch Up, High=Pitch Down? Or vice versa.
    // Usually: Stick Up (High PWM) -> Pitch Down (Nose Down).
    // Let's assume Standard: High PWM (2000) -> +45 deg.
    target_pitch = (float)(rx_pitch - 1500) / 500.0f * RC_MAX_ANGLE_DEG;
  }

  // Map Yaw (Rate Mode)
  float target_yaw_rate = 0.0f;
  if (abs(rx_yaw - 1500) > RC_DEADBAND_US) {
    target_yaw_rate = (float)(rx_yaw - 1500) / 500.0f * RC_MAX_YAW_RATE_DPS;
  }

  // A. Outer Loop (Angle)
  angle_control_update(imu->roll_deg, imu->pitch_deg, target_roll, target_pitch,
                       1.0f / CONTROL_LOOP_FREQ_HZ);
  const angle_output_t *angle_out = angle_control_get_output();

  // B. Inner Loop (Rate)
  // Use Angle Loop Output as Setpoints for Roll/Pitch
  // Yaw is Rate-Controlled from Stick
  rate_control_update(angle_out->roll_rate_setpoint,
                      angle_out->pitch_rate_setpoint, target_yaw_rate,
                      imu->gyro_x_dps, imu->gyro_y_dps, imu->gyro_z_dps);

  // 4. Update Mixer
  const rate_output_t *pid_out = rate_control_get_output();

  // Throttle:
  // DIRECT RX CONTROL
  // If armed, use the mapped RX throttle.
  // If disarmed, mixer handles it (stops).
  uint16_t throttle = system_armed ? rx_thr : 1000;

  // Safety: Ensure throttle doesn't drop below min or exceed max
  if (throttle < 1000)
    throttle = 1000;
  if (throttle > 2000)
    throttle = 2000;

  // TEST MODE: RESTORED PID MIXING
  // Full flight controller logic active!
  mixer_update(throttle, pid_out->roll, pid_out->pitch, pid_out->yaw);

  int64_t end_time = esp_timer_get_time();

  // 5. Store Debug Data
  if (++debug_counter >= DEBUG_PRINT_DIVIDER) {
    debug_counter = 0;
    control_loop_flag = true;

    debug_gyro[0] = imu->gyro_x_dps;
    debug_gyro[1] = imu->gyro_y_dps;
    debug_gyro[2] = imu->gyro_z_dps;

    debug_angle[0] = imu->roll_deg;
    debug_angle[1] = imu->pitch_deg;

    debug_pid[0] = pid_out->roll;
    debug_pid[1] = pid_out->pitch;
    debug_pid[2] = pid_out->yaw;

    mixer_get_outputs(&debug_motors[0], &debug_motors[1], &debug_motors[2],
                      &debug_motors[3]);

    // Capture RX for debug
    debug_rc[0] = rx_get_channel(0); // Roll
    debug_rc[1] = rx_get_channel(1); // Pitch
    debug_rc[2] = rx_get_channel(2); // Thr
    debug_rc[3] = rx_get_channel(3); // Yaw
    debug_rc[4] = rx_get_channel(4); // Aux1

    debug_exec_time_us = end_time - start_time;
  }
}

/* -------------------------------------------------------------------------- */
/*                                 Main Entry                                 */
/* -------------------------------------------------------------------------- */

void app_main(void) {
  // 0. IMMEDIATE PWM INIT (Fix for ESC boot issue)
  // Initialize motors FIRST to prevent ESCs from seeing floating pins
  pwm_init();

  // Send stable 1000us (IDLE) immediately
  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, 1000);
  }

  // Wait for serial monitor to connect
  // printf("BOOTING...\n");
  // vTaskDelay(pdMS_TO_TICKS(3000)); // REMOVED: Faster boot

  // 1. Init NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Load Configuration (defaults first, then try NVS)
  config_load_defaults();
  if (config_load_from_nvs()) {
    // printf("Loaded PID config from NVS.\n");
  } else {
    // printf("Using default PID config.\n");
  }

  // 2. Init Wi-Fi and Web Server (runs on Core 0)
  webserver_init();
  // printf("Web Server running at http://192.168.4.1\n");

  // 2. Init GPIO
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  gpio_reset_pin(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

  // 3. Init Subsystems
  adc_init();
  // pwm_init(); // Moved to top

  // ESC Init (Moved to top)

  rx_init();
  mixer_init();

  if (imu_init() != ESP_OK) {
    // printf("IMU Init Failed!\n");
    while (1)
      vTaskDelay(100);
  }

  // 4. IMU Calibration (Check NVS first, then calibrate if needed)
  if (imu_calibration_exists_in_nvs()) {
    // Load stored calibration from NVS (skip manual calibration!)
    // printf("Found IMU calibration in NVS, loading...\n");
    if (!imu_calibration_load_from_nvs()) {
      // Failed to load, do fresh calibration
      // printf("Failed to load calibration, performing fresh
      // calibration...\n");
      goto do_calibration;
    }
    // printf("IMU calibration loaded from NVS - Skipping calibration!\n");
  } else {
  do_calibration:
    // No stored calibration, perform fresh calibration
    // printf("No IMU calibration in NVS, performing fresh calibration...\n");
    // printf("Calibrating Gyro... Keep Still.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    imu_calibrate_gyro();
    // printf("Gyro Calibrated.\n");

    // printf("Calibrating Accel... Keep Level.\n");
    imu_calibrate_accel();
    // printf("Accel Calibrated.\n");

    // Save calibration to NVS for next boot
    imu_calibration_save_to_nvs();
    // printf("IMU calibration saved to NVS.\n");
  }

  // 5. Init Control Loops
  rate_control_init();
  angle_control_init();

  // 6. Start Control Loop
  const esp_timer_create_args_t timer_args = {
      .callback = &control_loop_callback,
      .name = "control_loop",
      .dispatch_method = ESP_TIMER_TASK,
  };
  esp_timer_handle_t control_timer;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &control_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(control_timer, CONTROL_LOOP_PERIOD_US));

  // printf("Control Loop Started.\n");

  // LED blink state for low battery warning
  static uint8_t led_blink_counter = 0;

  // 8. Main Loop (Monitoring & Arming)
  while (1) {
    // 7. Arming & Safety Logic (RX Channel 5) - runs every iteration
    bool rx_ok = rx_is_connected();
    uint16_t rx_aux1 = rx_get_channel(4);    // Channel 5 (0-indexed is 4)
    bool arm_switch_high = (rx_aux1 > 1600); // Threshold for arming

    // Arming State Machine
    if (rx_ok && arm_switch_high && !error_state) {
      if (!system_armed) {
        // Check throttle safety before arming
        uint16_t rx_thr_check = rx_get_channel(2);
        if (rx_thr_check < 1150) { // Throttle must be low to arm
          system_armed = true;
          mixer_arm(true);
          // printf("ARMED! (Switch High)\n");
        } else {
          // static int warn_counter = 0;
          // if (warn_counter++ % 50 == 0)
          //   printf("CANNOT ARM: Throttle not low!\n");
        }
      }
    } else {
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        // if (!rx_ok)
        //   printf("DISARMED! (RX Signal Lost)\n");
        // else
        //   printf("DISARMED! (Switch Low)\n");
      }
    }

    // Low Battery Check
    debug_vbat = adc_read_battery_voltg();
    bool low_battery = (debug_vbat < sys_cfg.low_bat_threshold);

    // LED Status Logic:
    // - Armed: LED ON (solid)
    // - Low Battery: LED BLINK (fast)
    // - Disarmed + Battery OK: LED OFF
    if (low_battery) {
      led_blink_counter++;
      gpio_set_level(LED_PIN, (led_blink_counter / 5) % 2); // Fast blink
    } else if (system_armed) {
      gpio_set_level(LED_PIN, 1); // Solid ON
    } else {
      gpio_set_level(LED_PIN, 0); // OFF
    }

    // Critical battery disarm
    if (system_armed && debug_vbat < 9500) { // Critical 9.5V
      system_armed = false;
      mixer_arm(false);
      // printf("CRITICAL BATTERY! DISARMED.\n");
    }

    // Check Emergency Stop Button
    if (gpio_get_level(BUTTON_PIN) == 0) {
      system_armed = false;
      mixer_arm(false);
      // printf("EMERGENCY STOP TRIGGERED!\n");
      error_state = true;
    }

    // TEST MODE: Print ALL Debug Data
    if (control_loop_flag) {
      control_loop_flag = false;
      // printf("GYRO: %6.2f %6.2f %6.2f | ANG: %6.2f %6.2f | RX: %4d %4d %4d "
      //        "%4d %4d | MOT: %4d %4d %4d %4d | BAT: %4d mV | STS: %s %s\n",
      //        debug_gyro[0], debug_gyro[1], debug_gyro[2], debug_angle[0],
      //        debug_angle[1], debug_rc[0], debug_rc[1], debug_rc[2],
      //        debug_rc[3], debug_rc[4], debug_motors[0], debug_motors[1],
      //        debug_motors[2], debug_motors[3], debug_vbat, system_armed ?
      //        "ARMED" : "DISARM", error_state ? "ERR" : "OK");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
