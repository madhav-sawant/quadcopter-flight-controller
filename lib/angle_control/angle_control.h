#ifndef ANGLE_CONTROL_H
#define ANGLE_CONTROL_H

/**
 * @file angle_control.h
 * @brief Angle Mode Controller (Outer Loop)
 *
 * Converts target angles to target rates for the inner rate loop.
 * Simple P controller for easy tuning.
 */

/**
 * @brief Initialize angle controller (reset state)
 */
void angle_control_init(void);

/**
 * @brief Reset integral terms (call on disarm)
 */
void angle_control_reset(void);

/**
 * @brief Run angle controller
 *
 * @param target_roll_deg   Desired roll angle (degrees)
 * @param target_pitch_deg  Desired pitch angle (degrees)
 * @param current_roll_deg  Current roll angle from IMU (degrees)
 * @param current_pitch_deg Current pitch angle from IMU (degrees)
 * @param out_roll_rate     Output: target roll rate (deg/s)
 * @param out_pitch_rate    Output: target pitch rate (deg/s)
 */
void angle_control_update(float target_roll_deg, float target_pitch_deg,
                          float current_roll_deg, float current_pitch_deg,
                          float *out_roll_rate, float *out_pitch_rate);

#endif // ANGLE_CONTROL_H
