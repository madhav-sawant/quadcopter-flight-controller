#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

// Buffer configuration
#define BLACKBOX_MAX_ENTRIES 2000 // ~96KB (48 bytes * 2000 = 96000 bytes)
#define BLACKBOX_LOG_DIVIDER 10 // Log every 10th control loop iteration (50Hz)

// Log entry structure (48 bytes, packed)
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;              // 4 bytes - Time since boot
  float gyro_x, gyro_y, gyro_z;       // 12 bytes - Gyro rates (deg/s)
  float angle_roll, angle_pitch;      // 8 bytes - Fused angles (deg)
  float pid_roll, pid_pitch, pid_yaw; // 12 bytes - PID outputs
  uint16_t motor[4];                  // 8 bytes - Motor PWM values
  uint16_t throttle;                  // 2 bytes - Throttle input
  uint16_t flags;   // 2 bytes - Bit flags (armed, error, etc.)
} blackbox_entry_t; // Total: 48 bytes

// Flag bit definitions
#define BLACKBOX_FLAG_ARMED (1 << 0)
#define BLACKBOX_FLAG_ERROR (1 << 1)

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
