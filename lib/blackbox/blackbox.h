#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

// Buffer configuration
// Reduced entries to fit expanded structure in RAM
#define BLACKBOX_MAX_ENTRIES 1200 // ~132KB with expanded structure

// Log rate: 250Hz / 3 = 83Hz (captures sub-second dynamics for PID tuning)
// At 83Hz with 1200 entries = ~14.4 seconds of recording
#define BLACKBOX_LOG_DIVIDER 3

// ============================================================================
// EXPANDED LOG ENTRY STRUCTURE (100 bytes, packed)
// ============================================================================
// This captures the COMPLETE control loop state for deep analysis
// ============================================================================
typedef struct __attribute__((packed)) {
  // === TIME & STATUS (6 bytes) ===
  uint32_t timestamp_ms; // Time since boot
  uint16_t flags;        // Bit flags (armed, error, recovery, etc.)

  // === RAW IMU DATA (24 bytes) ===
  // Allows post-flight filter tuning and noise analysis
  float gyro_x, gyro_y, gyro_z; // Gyro rates (deg/s) - AFTER software filter
  float accel_x, accel_y,
      accel_z; // Accelerometer (g) - RAW for vibration analysis

  // === FUSED ANGLES (8 bytes) ===
  float angle_roll, angle_pitch; // Complementary filter output (deg)

  // === ANGLE LOOP - OUTER (20 bytes) ===
  // Critical for understanding self-leveling behavior
  float angle_setpoint_roll;  // Target angle from RC (deg)
  float angle_setpoint_pitch; // Target angle from RC (deg)
  float angle_error_roll;     // Setpoint - Actual (deg)
  float angle_error_pitch;    // Setpoint - Actual (deg)
  float angle_i_term_roll;    // Angle I-term (for drift diagnosis)
  float angle_i_term_pitch;   // Angle I-term (for drift diagnosis)

  // === RATE LOOP - INNER (28 bytes) ===
  // Critical for understanding oscillation and response
  float rate_setpoint_roll;  // Output from angle loop (deg/s)
  float rate_setpoint_pitch; // Output from angle loop (deg/s)
  float rate_error_roll;     // Setpoint - Actual (deg/s)
  float rate_error_pitch;    // Setpoint - Actual (deg/s)
  float rate_i_term_roll;    // Rate I-term (for windup diagnosis)
  float rate_i_term_pitch;   // Rate I-term (for windup diagnosis)
  float rate_i_term_yaw;     // Rate I-term (for windup diagnosis)

  // === PID OUTPUTS (12 bytes) ===
  float pid_roll, pid_pitch, pid_yaw; // Final PID outputs to mixer

  // === MOTORS (8 bytes) ===
  uint16_t motor[4]; // Motor PWM values (μs)

  // === RC INPUTS (6 bytes) ===
  uint16_t rc_throttle; // Throttle stick (μs)
  uint16_t rc_roll;     // Roll stick (μs)
  uint16_t rc_pitch;    // Pitch stick (μs)

  // === SYSTEM HEALTH (6 bytes) ===
  uint16_t battery_mv;   // Battery voltage (mV)
  uint16_t loop_time_us; // Control loop execution time (μs)
  int8_t cpu_temp;       // ESP32 temperature (°C) - optional
  uint8_t pad;           // Padding for alignment

} blackbox_entry_t; // Total: ~110 bytes

// ============================================================================
// FLAG BIT DEFINITIONS (expanded)
// ============================================================================
#define BLACKBOX_FLAG_ARMED (1 << 0)    // Motors armed
#define BLACKBOX_FLAG_ERROR (1 << 1)    // System error
#define BLACKBOX_FLAG_RECOVERY (1 << 2) // In angle recovery mode (>45°)
#define BLACKBOX_FLAG_I_FROZEN (1 << 3) // I-term frozen (low throttle)
#define BLACKBOX_FLAG_LOW_BAT (1 << 4)  // Low battery warning
#define BLACKBOX_FLAG_RX_LOSS (1 << 5)  // RC signal lost
#define BLACKBOX_FLAG_GYRO_SAT (1 << 6) // Gyro saturation detected
#define BLACKBOX_FLAG_PID_SAT (1 << 7)  // PID output saturated

// API Functions

/**
 * @brief Initialize the blackbox system.
 *        Creates a FreeRTOS task on Core 0 for processing log entries.
 */
void blackbox_init(void);

/**
 * @brief Log a flight data entry (non-blocking).
 *        Safe to call from ISR or control loop.
 *        If queue is full, entry is dropped (no blocking).
 * @param entry Pointer to the entry data to log.
 */
void blackbox_log(const blackbox_entry_t *entry);

/**
 * @brief Clear all logged entries.
 */
void blackbox_clear(void);

/**
 * @brief Get the number of logged entries.
 * @return Number of entries in the buffer.
 */
uint16_t blackbox_get_count(void);

/**
 * @brief Get a logged entry by index.
 * @param index Index of the entry (0 to count-1).
 * @return Pointer to the entry, or NULL if index out of range.
 */
const blackbox_entry_t *blackbox_get_entry(uint16_t index);

/**
 * @brief Start recording (enabled by default after init).
 */
void blackbox_start(void);

/**
 * @brief Stop recording.
 */
void blackbox_stop(void);

#endif // BLACKBOX_H
