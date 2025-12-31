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
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../lib/blackbox/blackbox.h"
#include "../lib/config/config.h"
#include "../lib/pwm/pwm.h"
#include "../lib/rx/rx.h"
#include "../lib/webserver/webserver.h"
#include "adc.h"
#include "angle_control.h"
#include "imu.h"
#include "mixer.h"
#include "rate_control.h"

/* -------------------------------------------------------------------------- */
/*                               Configuration                                */
/* -------------------------------------------------------------------------- */

// Status LED
#define LED_PIN 2
#define BUTTON_PIN 0 // Boot button for emergency stop

// Throttle Limit for Tuning (prevents climbing too high)
// Set to 1400 during tuning, change to 2000 for normal flight
#define TUNING_THROTTLE_LIMIT 1400

// Control loop frequency (250 Hz)
#define CONTROL_LOOP_FREQ_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ)

// Debug print frequency (10 Hz) -> 250 / 25 = 10
#define DEBUG_PRINT_DIVIDER 25

// RC Mapping
#define RC_MAX_ANGLE_DEG 45.0f
#define RC_MAX_YAW_RATE_DPS 180.0f
#define RC_DEADBAND_US 20

/* -------------------------------------------------------------------------- */
/*                               Global State                                 */
/* -------------------------------------------------------------------------- */

static volatile bool control_loop_flag = false;
static int debug_counter = 0;
static int blackbox_counter = 0;  // Counter for 83Hz blackbox logging
static int led_blink_counter = 0; // Counter for low battery LED blinking

// System State
bool system_armed = false;
static bool error_state = false;
char system_status_msg[64] = "System Ready"; // Global status message

// Debug data
static float debug_gyro[3];
static float debug_angle[2];      // Roll, Pitch
static float debug_setpoint[2];   // Target Roll, Pitch
static int debug_integral_active; // 1=Active, 0=Frozen
static float debug_pid[3];
static uint16_t debug_motors[4];
static uint16_t debug_vbat = 0;
static int64_t debug_exec_time_us = 0;

/* -------------------------------------------------------------------------- */
/*                   Control Loop Task (Core 1 - 250 Hz)                      */
/* -------------------------------------------------------------------------- */

// Control loop runs as a dedicated FreeRTOS task pinned to Core 1
// Core 0: WiFi, Webserver, Logger (background I/O)
// Core 1: Control Loop, Sensor Fusion, PWM (real-time flight critical)

static void control_loop_task(void *arg) {
  (void)arg;

  // Use esp_timer for precise 4ms (250Hz) timing since FreeRTOS tick is 10ms
  const int64_t cycle_period_us =
      1000000 / CONTROL_LOOP_FREQ_HZ; // 4000us for 250Hz
  int64_t next_cycle_time = esp_timer_get_time();

  printf("Control Loop Task started on Core %d\n", xPortGetCoreID());

  // Register this task with the Task Watchdog Timer (TWDT)
  // This is necessary because Core 1 runs at high priority and never yields
  esp_task_wdt_add(NULL);

  while (1) {
    // Feed the watchdog at each iteration
    esp_task_wdt_reset();
    int64_t start_time = esp_timer_get_time();

    // 1. Read IMU
    imu_read(1.0f / CONTROL_LOOP_FREQ_HZ);
    const imu_data_t *imu = imu_get_data();

    // 2. Safety Checks (Crash detection)
    // Angle-based: if roll or pitch angle is too high, disarm
    if (fabs(imu->roll_deg) > sys_cfg.crash_angle_deg ||
        fabs(imu->pitch_deg) > sys_cfg.crash_angle_deg) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      snprintf(system_status_msg, sizeof(system_status_msg),
               "CRASH DETECTED: Angle > %.0f deg", sys_cfg.crash_angle_deg);
    }

    // Gyro rate-based: detect rapid rotation (impact/crash) even if filtered
    // angles lag 500 deg/s threshold catches most crash scenarios
    const float CRASH_GYRO_RATE_DPS = 500.0f;
    if (fabs(imu->gyro_x_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_y_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_z_dps) > CRASH_GYRO_RATE_DPS) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      snprintf(system_status_msg, sizeof(system_status_msg),
               "CRASH DETECTED: Gyro > %.0f dps", CRASH_GYRO_RATE_DPS);
    }

    // 3. Read RX Channels
    uint16_t rx_roll = rx_get_channel(0);
    uint16_t rx_pitch = rx_get_channel(1);
    uint16_t rx_thr = rx_get_channel(2);
    uint16_t rx_yaw = rx_get_channel(3);

    // Map Yaw (always rate controlled)
    float target_yaw_rate = 0.0f;
    if (abs(rx_yaw - 1500) > RC_DEADBAND_US) {
      target_yaw_rate = (float)(rx_yaw - 1500) / 500.0f * RC_MAX_YAW_RATE_DPS;
    }

    // Map Roll/Pitch Stick -> Angle Target
    float target_roll = 0.0f;
    if (abs(rx_roll - 1500) > RC_DEADBAND_US) {
      target_roll = (float)(rx_roll - 1500) / 500.0f * RC_MAX_ANGLE_DEG;
    }

    float target_pitch = 0.0f;
    if (abs(rx_pitch - 1500) > RC_DEADBAND_US) {
      target_pitch = (float)(rx_pitch - 1500) / 500.0f * RC_MAX_ANGLE_DEG;
    }

    // Update webserver live display with target angles
    webserver_set_targets(target_roll, target_pitch);

    // 4. Cascade Control: Angle -> Rate
    // Angle loop output = rate setpoint for Rate loop
    // When Angle P = 0, angle_control outputs 0, so Rate loop targets 0 deg/s
    angle_control_update(imu->roll_deg, imu->pitch_deg, target_roll,
                         target_pitch, 1.0f / CONTROL_LOOP_FREQ_HZ);
    const angle_output_t *angle_out = angle_control_get_output();

    // Rate loop: try to achieve the rate setpoint from angle loop (0 when P=0)
    rate_control_update(angle_out->roll_rate_setpoint,
                        angle_out->pitch_rate_setpoint, target_yaw_rate,
                        imu->gyro_x_dps, imu->gyro_y_dps, imu->gyro_z_dps);

    // 5. Update Mixer
    const rate_output_t *pid_out = rate_control_get_output();

    // Throttle:
    // DIRECT RX CONTROL
    // If armed, use the mapped RX throttle.
    // If disarmed, mixer handles it (stops).
    uint16_t throttle = system_armed ? rx_thr : 1000;

    // Safety: Ensure throttle doesn't drop below min or exceed max
    if (throttle < 1000)
      throttle = 1000;
    if (throttle > TUNING_THROTTLE_LIMIT) // Limited for tuning safety
      throttle = TUNING_THROTTLE_LIMIT;

    // Integral Anti-Windup: Freeze I-term when throttle is low (on ground)
    // Threshold lowered from 1200 to 1080 to allow I-term during low-throttle
    // tuning
    bool freeze_integral = (throttle < 1080);
    rate_control_freeze_integral(freeze_integral);
    angle_control_freeze_integral(freeze_integral);

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

      debug_setpoint[0] = target_roll;
      debug_setpoint[1] = target_pitch;
      debug_integral_active = (int)!freeze_integral;

      debug_pid[0] = pid_out->roll;
      debug_pid[1] = pid_out->pitch;
      debug_pid[2] = pid_out->yaw;

      mixer_get_outputs(&debug_motors[0], &debug_motors[1], &debug_motors[2],
                        &debug_motors[3]);

      debug_exec_time_us = end_time - start_time;
    }

    // 6. Blackbox Logging (50Hz - every 10th iteration) - ONLY WHEN ARMED
    if (system_armed && ++blackbox_counter >= BLACKBOX_LOG_DIVIDER) {
      blackbox_counter = 0;

      // Calculate expanded flags
      uint16_t bb_flags = 0;
      if (system_armed)
        bb_flags |= BLACKBOX_FLAG_ARMED;
      if (error_state)
        bb_flags |= BLACKBOX_FLAG_ERROR;
      if (angle_control_in_recovery())
        bb_flags |= BLACKBOX_FLAG_RECOVERY;
      if (freeze_integral)
        bb_flags |= BLACKBOX_FLAG_I_FROZEN;
      // Add low battery flag
      if (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold)
        bb_flags |= BLACKBOX_FLAG_LOW_BAT;
      // Gyro saturation check
      if (fabs(imu->gyro_x_dps) > 400 || fabs(imu->gyro_y_dps) > 400)
        bb_flags |= BLACKBOX_FLAG_GYRO_SAT;
      // PID saturation check
      if (fabs(pid_out->roll) > sys_cfg.rate_output_limit * 0.95f)
        bb_flags |= BLACKBOX_FLAG_PID_SAT;

      blackbox_entry_t entry = {
          // Time & Status
          .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
          .flags = bb_flags,

          // Raw IMU Data
          .gyro_x = imu->gyro_x_dps,
          .gyro_y = imu->gyro_y_dps,
          .gyro_z = imu->gyro_z_dps,
          .accel_x = imu->accel_x_g,
          .accel_y = imu->accel_y_g,
          .accel_z = imu->accel_z_g,

          // Fused Angles
          .angle_roll = imu->roll_deg,
          .angle_pitch = imu->pitch_deg,

          // Angle Loop (Outer) - setpoints and errors
          .angle_setpoint_roll = target_roll,
          .angle_setpoint_pitch = target_pitch,
          .angle_error_roll = target_roll - imu->roll_deg,
          .angle_error_pitch = target_pitch - imu->pitch_deg,
          .angle_i_term_roll = angle_control_get_i_roll(),
          .angle_i_term_pitch = angle_control_get_i_pitch(),

          // Rate Loop (Inner) - setpoints and errors
          .rate_setpoint_roll = angle_out->roll_rate_setpoint,
          .rate_setpoint_pitch = angle_out->pitch_rate_setpoint,
          .rate_error_roll = angle_out->roll_rate_setpoint - imu->gyro_x_dps,
          .rate_error_pitch = angle_out->pitch_rate_setpoint - imu->gyro_y_dps,
          .rate_i_term_roll = rate_control_get_i_roll(),
          .rate_i_term_pitch = rate_control_get_i_pitch(),
          .rate_i_term_yaw = rate_control_get_i_yaw(),

          // PID Outputs
          .pid_roll = pid_out->roll,
          .pid_pitch = pid_out->pitch,
          .pid_yaw = pid_out->yaw,

          // Motors
          .motor = {debug_motors[0], debug_motors[1], debug_motors[2],
                    debug_motors[3]},

          // RC Inputs
          .rc_throttle = throttle,
          .rc_roll = rx_roll,
          .rc_pitch = rx_pitch,

          // System Health
          .battery_mv = debug_vbat,
          .loop_time_us = (uint16_t)debug_exec_time_us,
          .cpu_temp = 0, // TODO: Add ESP32 temperature reading
          .pad = 0};
      blackbox_log(&entry);
    } else if (!system_armed) {
      blackbox_counter = 0; // Reset counter when disarmed
    }

    // Wait for next cycle - precise timing using esp_timer
    // Busy-wait until next cycle time (FreeRTOS tick is 10ms, too slow for
    // 250Hz)
    next_cycle_time += cycle_period_us;
    while (esp_timer_get_time() < next_cycle_time) {
      // Tight loop - yields CPU briefly to avoid complete starvation
      taskYIELD();
    }
  }
}

/* -------------------------------------------------------------------------- */
/*                                 Main Entry                                 */
/* -------------------------------------------------------------------------- */

void app_main(void) {
  // ========== CRITICAL: ESC BOOT FIX ==========
  // Configure motor GPIOs as OUTPUT LOW immediately to prevent
  // ESCs from seeing floating signals during ESP32 boot.
  // This happens BEFORE any other initialization!
  gpio_reset_pin(PWM_MOTOR_1_GPIO); // Motor 1
  gpio_reset_pin(PWM_MOTOR_2_GPIO); // Motor 2
  gpio_reset_pin(PWM_MOTOR_3_GPIO); // Motor 3
  gpio_reset_pin(PWM_MOTOR_4_GPIO); // Motor 4
  gpio_set_direction(PWM_MOTOR_1_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_2_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_3_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_4_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(PWM_MOTOR_1_GPIO, 0);
  gpio_set_level(PWM_MOTOR_2_GPIO, 0);
  gpio_set_level(PWM_MOTOR_3_GPIO, 0);
  gpio_set_level(PWM_MOTOR_4_GPIO, 0);

  // Now init PWM properly (will take over these pins)
  pwm_init();

  // Send stable IDLE signal to all motors for 3 seconds
  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, 1000);
  }
  vTaskDelay(pdMS_TO_TICKS(3000)); // ESC arm time
  // ========== END ESC BOOT FIX ==========

  // 1. Init NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Load Configuration
  config_load_defaults();
  config_load_from_nvs(); // Load saved values from flash (if any)

  // 2. Init GPIO
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  gpio_reset_pin(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

  // 3. Init Subsystems
  adc_init();

  rx_init();
  mixer_init();
  blackbox_init();  // RAM-based flight data logger
  webserver_init(); // WiFi AP + PID tuning web interface

  if (imu_init() != ESP_OK) {
    printf("IMU Init Failed!\n");
    while (1)
      vTaskDelay(100);
  }

  // 4. Gyro/Accel Calibration (load from NVS if available)
  if (imu_calibration_load_from_nvs()) {
    printf("IMU Calibration loaded from NVS.\n");
  } else {
    printf("Calibrating Gyro... Keep Still.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    imu_calibrate_gyro();
    printf("Gyro Calibrated.\n");

    printf("Calibrating Accel... Keep Level.\n");
    imu_calibrate_accel();
    printf("Accel Calibrated.\n");

    // Save calibration to NVS for next boot
    imu_calibration_save_to_nvs();
    printf("Calibration saved to NVS.\n");
  }

  // ========== LEVEL CHECK ==========
  // Take multiple IMU readings and check if quad is level
  // This helps verify trim values are correct before flying
  printf("\n");
  printf("========================================\n");
  printf("       LEVEL CHECK - Keep quad level!\n");
  printf("========================================\n");
  printf("Taking 100 samples...\n");

  float roll_sum = 0.0f;
  float pitch_sum = 0.0f;
  const int LEVEL_CHECK_SAMPLES = 100;

  // Wait for IMU to stabilize
  vTaskDelay(pdMS_TO_TICKS(500));

  // Collect samples
  for (int i = 0; i < LEVEL_CHECK_SAMPLES; i++) {
    imu_read(0.004f); // 4ms dt
    const imu_data_t *imu = imu_get_data();
    roll_sum += imu->roll_deg;
    pitch_sum += imu->pitch_deg;
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms between samples
  }

  float avg_roll = roll_sum / LEVEL_CHECK_SAMPLES;
  float avg_pitch = pitch_sum / LEVEL_CHECK_SAMPLES;

  printf("\n");
  printf("LEVEL CHECK RESULTS:\n");
  printf("----------------------------------------\n");
  printf("  Average Roll:  %+.2f degrees\n", avg_roll);
  printf("  Average Pitch: %+.2f degrees\n", avg_pitch);
  printf("----------------------------------------\n");

  // Check if level (within ±1 degree is acceptable)
  bool roll_ok = fabsf(avg_roll) < 1.0f;
  bool pitch_ok = fabsf(avg_pitch) < 1.0f;

  if (roll_ok && pitch_ok) {
    printf("  STATUS: ✓ LEVEL OK!\n");
    printf("  Quad is level, ready to fly.\n");
  } else {
    printf("  STATUS: ✗ NOT LEVEL - Adjust trim!\n");
    printf("\n");
    printf("  RECOMMENDED TRIM ADJUSTMENTS:\n");

    if (!pitch_ok) {
      if (avg_pitch > 0) {
        printf("  → PITCH_TRIM_DEG: Decrease by %.1f\n", avg_pitch);
        printf("    (Current pitch reads +%.1f, reduce trim)\n", avg_pitch);
      } else {
        printf("  → PITCH_TRIM_DEG: Increase by %.1f\n", -avg_pitch);
        printf("    (Current pitch reads %.1f, increase trim)\n", avg_pitch);
      }
    }

    if (!roll_ok) {
      if (avg_roll > 0) {
        printf("  → ROLL_TRIM_DEG: Decrease by %.1f\n", avg_roll);
        printf("    (Current roll reads +%.1f, reduce trim)\n", avg_roll);
      } else {
        printf("  → ROLL_TRIM_DEG: Increase by %.1f\n", -avg_roll);
        printf("    (Current roll reads %.1f, increase trim)\n", avg_roll);
      }
    }

    printf("\n");
    printf("  Edit lib/imu/imu.c lines 57-58 and rebuild.\n");
  }

  printf("========================================\n\n");
  // ========== END LEVEL CHECK ==========

  // 5. Init Control Loops
  rate_control_init();
  angle_control_init();

  // 6. Start Control Loop on Core 1 (dedicated for flight-critical code)
  // Core 0: WiFi, Webserver, Logger (background I/O)
  // Core 1: Control Loop, Sensor Fusion, PWM (real-time flight critical)
  xTaskCreatePinnedToCore(control_loop_task, // Task function
                          "control_loop",    // Task name
                          4096,              // Stack size (bytes)
                          NULL,              // Task parameters
                          24,   // Priority (high - flight critical)
                          NULL, // Task handle (not needed)
                          1     // Core 1 (isolated from WiFi)
  );

  printf("Control Loop Task created on Core 1.\n");

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

        // Check battery voltage before arming
        // Prevent arming if: battery = 0 (ADC error) OR battery invalid OR
        // battery critical
        uint16_t bat_check = adc_read_battery_voltg();
        bool battery_ok = (bat_check > 9900); // Must be above 9.9V to arm

        if (!battery_ok) {
          static int bat_warn_counter = 0;
          if (bat_warn_counter++ % 50 == 0)
            printf("CANNOT ARM: Battery invalid or critical! (%.2fV)\n",
                   bat_check / 1000.0f);
        } else if (rx_thr_check >= 1150) {
          static int warn_counter = 0;
          if (warn_counter++ % 50 == 0)
            printf("CANNOT ARM: Throttle not low!\n");
        } else {
          // All checks passed - ARM!
          system_armed = true;

          // Reset PIDs to prevent I-term windup from ground handling
          rate_control_init();
          angle_control_init();

          mixer_arm(true);
          blackbox_clear(); // Clear old data
          blackbox_start(); // Start recording new flight
          printf("ARMED! (Switch High)\n");
        }
      }
    } else {
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        blackbox_stop(); // Stop recording, preserve data for download
        if (!rx_ok)
          printf("DISARMED! (RX Signal Lost - Failsafe Triggered)\n");
        else
          printf("DISARMED! (Switch Low)\n");
      }

      // Clear error state if disarmed and switch is low (allow re-arming)
      if (error_state && !arm_switch_high) {
        error_state = false;
        printf("Error State Cleared. Ready to Arm.\n");
      }
    }

    // Low Battery Check (Slow)
    debug_vbat = adc_read_battery_voltg();
    bool low_battery_warning =
        (debug_vbat < sys_cfg.low_bat_threshold); // Below 10.5V

    // LED Status: Blink if low battery, otherwise solid when armed
    if (low_battery_warning) {
      // Blink LED as warning (toggle every ~25 iterations = ~250ms at 10ms
      // loop)
      led_blink_counter++;
      if (led_blink_counter >= 25) {
        led_blink_counter = 0;
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN)); // Toggle LED
      }
    } else {
      led_blink_counter = 0;
      gpio_set_level(LED_PIN, system_armed ? 1 : 0);
    }

    // Check Emergency Stop Button
    if (gpio_get_level(BUTTON_PIN) == 0) {
      system_armed = false;
      mixer_arm(false);
      printf("EMERGENCY STOP TRIGGERED!\n");
      error_state = true;
      snprintf(system_status_msg, sizeof(system_status_msg),
               "EMERGENCY STOP: Button Pressed");
    }

    // Critical Battery Check - Disarm if critically low
    // Only check if voltage reading is valid (> 5V = wire connected)
    if (system_armed && debug_vbat > 5000 &&
        debug_vbat <= 9900) { // Critical <= 9.9V
      system_armed = false;
      mixer_arm(false);
      gpio_set_level(LED_PIN, 1); // LED solid ON for critical battery
      printf("CRITICAL BATTERY! DISARMED.\n");
      error_state = true; // Ensure error state is set
      snprintf(system_status_msg, sizeof(system_status_msg),
               "CRITICAL BATTERY: %.2fV", debug_vbat / 1000.0f);
    }

    if (control_loop_flag) {
      control_loop_flag = false;

      uint16_t rx_ch[RX_CHANNEL_COUNT];
      rx_get_all(rx_ch);

      // === FAST DEBUG PRINT - For bench testing ===
      // Format: Angles | PID Outputs | Motors
      // R/P = Roll/Pitch angles (deg)
      // Pr/Pp = PID output for roll/pitch (what goes to mixer)
      // M1-M4 = Motor PWM values
      // Format: Setpoints | Angles | I-Term | PID Outputs | Motors
      // S: Setpoint, A: Angle, I: Integral Active (1=Yes, 0=No)
      printf("S:%+5.1f|%+5.1f A:%+5.1f|%+5.1f I:%d | P:%+6.1f|%+6.1f | M:%4d "
             "%4d %4d %4d\n",
             debug_setpoint[0], debug_setpoint[1], // Setpoints
             debug_angle[0], debug_angle[1],       // Actual Angles
             debug_integral_active,                // Integral Active?
             debug_pid[0], debug_pid[1],           // PID outputs
             debug_motors[0], debug_motors[1], debug_motors[2],
             debug_motors[3]);

      // === OLD 5Hz DEBUG PRINT (Commented for testing) ===
      // static uint32_t last_debug_print = 0;
      // uint32_t now = esp_timer_get_time() / 1000;
      // if (now - last_debug_print > 200) { // 5Hz
      //   printf("GZ:%6.2f | M1:%4d M2:%4d M3:%4d M4:%4d\n",
      //          imu_get_data()->gyro_z_dps, debug_motors[0],
      //          debug_motors[1], debug_motors[2], debug_motors[3]);
      //   last_debug_print = now;
      // }

      // if (error_state) {
      //   printf("!!! SYSTEM ERROR / DISARMED !!!\n");
      // }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
