#ifndef ANGLE_CONTROL_H
#define ANGLE_CONTROL_H

#include <stdint.h>

/**
 * @brief Output from the Angle PID Loop
 * These are the desired rates (deg/s) that should be fed into the Rate Loop.
 */
typedef struct {
  float roll_rate_setpoint;
  float pitch_rate_setpoint;
} angle_output_t;

/**
 * @brief Initialize Angle PID controllers
 * Sets up P-only controllers for Roll and Pitch.
 */
void angle_control_init(void);

/**
 * @brief Update Angle PID Loop
 *
 * @param roll_actual_deg Current Roll angle from IMU (degrees)
 * @param pitch_actual_deg Current Pitch angle from IMU (degrees)
 * @param roll_desired_deg Desired Roll angle (usually 0 for level)
 * @param pitch_desired_deg Desired Pitch angle (usually 0 for level)
 * @param dt_sec Time step in seconds
 */
void angle_control_update(float roll_actual_deg, float pitch_actual_deg,
                          float roll_desired_deg, float pitch_desired_deg,
                          float dt_sec);

/**
 * @brief Get the latest output from the Angle Loop
 *
 * @return const angle_output_t* Pointer to the output struct
 */
const angle_output_t *angle_control_get_output(void);

#endif // ANGLE_CONTROL_H
