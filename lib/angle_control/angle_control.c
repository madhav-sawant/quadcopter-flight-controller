/**
 * @file angle_control.c
 * @brief Angle Mode Controller Implementation
 *
 * PI controller that converts angle error to rate setpoint.
 * P-term: Proportional response to angle error
 * I-term: Eliminates steady-state drift
 */

#include "angle_control.h"
#include "../config/config.h"

// Control loop frequency (must match main.c)
#define CONTROL_FREQ_HZ 250.0f
#define DT (1.0f / CONTROL_FREQ_HZ)

// Maximum rate output from angle controller (deg/s)
#define MAX_ANGLE_OUTPUT_RATE 150.0f

// Maximum I-term contribution (deg/s) - prevents windup
#define MAX_I_OUTPUT 30.0f

// Internal I-term state
static float i_roll = 0.0f;
static float i_pitch = 0.0f;

void angle_control_init(void) {
  i_roll = 0.0f;
  i_pitch = 0.0f;
}

void angle_control_reset(void) {
  i_roll = 0.0f;
  i_pitch = 0.0f;
}

void angle_control_update(float target_roll_deg, float target_pitch_deg,
                          float current_roll_deg, float current_pitch_deg,
                          float *out_roll_rate, float *out_pitch_rate) {
  // Calculate angle errors
  float roll_error = target_roll_deg - current_roll_deg;
  float pitch_error = target_pitch_deg - current_pitch_deg;

  // I-term integration (only if Ki > 0)
  if (sys_cfg.angle_ki > 0.0f) {
    // Calculate max integral to prevent windup
    float max_integral = MAX_I_OUTPUT / sys_cfg.angle_ki;

    // Integrate
    i_roll += roll_error * DT;
    i_pitch += pitch_error * DT;

    // Anti-windup clamp
    if (i_roll > max_integral)
      i_roll = max_integral;
    if (i_roll < -max_integral)
      i_roll = -max_integral;
    if (i_pitch > max_integral)
      i_pitch = max_integral;
    if (i_pitch < -max_integral)
      i_pitch = -max_integral;
  } else {
    // Ki is 0, clear integrals
    i_roll = 0.0f;
    i_pitch = 0.0f;
  }

  // PI controller: P + I
  float roll_rate =
      (sys_cfg.angle_kp * roll_error) + (sys_cfg.angle_ki * i_roll);
  float pitch_rate =
      (sys_cfg.angle_kp * pitch_error) + (sys_cfg.angle_ki * i_pitch);

  // Clamp output rates
  if (roll_rate > MAX_ANGLE_OUTPUT_RATE)
    roll_rate = MAX_ANGLE_OUTPUT_RATE;
  if (roll_rate < -MAX_ANGLE_OUTPUT_RATE)
    roll_rate = -MAX_ANGLE_OUTPUT_RATE;
  if (pitch_rate > MAX_ANGLE_OUTPUT_RATE)
    pitch_rate = MAX_ANGLE_OUTPUT_RATE;
  if (pitch_rate < -MAX_ANGLE_OUTPUT_RATE)
    pitch_rate = -MAX_ANGLE_OUTPUT_RATE;

  *out_roll_rate = roll_rate;
  *out_pitch_rate = pitch_rate;
}
