#include "angle_control.h"
#include "../config/config.h"
#include "../pid/pid.h"
#include <math.h>
#include <stdbool.h>

// Safety Limits
#define MAX_RATE_SETPOINT_DPS 150.0f // Clamp desired rate to +/- 150 deg/s
#define MAX_SAFE_ANGLE 45.0f         // Recovery zone starts here
#define RECOVERY_GAIN 2.5f           // Aggressive correction multiplier

// PID Instances
static pid_controller_t pid_roll_angle;
static pid_controller_t pid_pitch_angle;

// Output State
static angle_output_t angle_output;

// Recovery zone status (for debugging/logging)
static bool in_recovery_mode = false;

void angle_control_init(void) {
  // Initialize PIDs with config values (full P/I/D for outer loop)
  // Output limit is the Max Rate we want to command
  pid_init(&pid_roll_angle, sys_cfg.angle_roll_kp, sys_cfg.angle_roll_ki,
           sys_cfg.angle_roll_kd, MAX_RATE_SETPOINT_DPS, 50.0f); // I-limit 50
  pid_init(&pid_pitch_angle, sys_cfg.angle_pitch_kp, sys_cfg.angle_pitch_ki,
           sys_cfg.angle_pitch_kd, MAX_RATE_SETPOINT_DPS, 50.0f);

  angle_output.roll_rate_setpoint = 0.0f;
  angle_output.pitch_rate_setpoint = 0.0f;
  in_recovery_mode = false;
}

void angle_control_update(float roll_actual_deg, float pitch_actual_deg,
                          float roll_desired_deg, float pitch_desired_deg,
                          float dt_sec) {

  // ========================================================================
  // RECOVERY ZONE: If angle > 45°, override pilot and aggressively correct
  // ========================================================================
  float roll_target = roll_desired_deg;
  float pitch_target = pitch_desired_deg;
  float roll_gain_mult = 1.0f;
  float pitch_gain_mult = 1.0f;

  // Clamp pilot input to safe angle
  if (roll_target > MAX_SAFE_ANGLE)
    roll_target = MAX_SAFE_ANGLE;
  if (roll_target < -MAX_SAFE_ANGLE)
    roll_target = -MAX_SAFE_ANGLE;
  if (pitch_target > MAX_SAFE_ANGLE)
    pitch_target = MAX_SAFE_ANGLE;
  if (pitch_target < -MAX_SAFE_ANGLE)
    pitch_target = -MAX_SAFE_ANGLE;

  // Check if we're in recovery zone (45° < angle < 60°)
  if (fabsf(roll_actual_deg) > MAX_SAFE_ANGLE) {
    // RECOVERY MODE: Force target to 0°, boost correction
    roll_target = 0.0f;
    roll_gain_mult = RECOVERY_GAIN;
    in_recovery_mode = true;
  }

  if (fabsf(pitch_actual_deg) > MAX_SAFE_ANGLE) {
    // RECOVERY MODE: Force target to 0°, boost correction
    pitch_target = 0.0f;
    pitch_gain_mult = RECOVERY_GAIN;
    in_recovery_mode = true;
  }

  if (fabsf(roll_actual_deg) <= MAX_SAFE_ANGLE &&
      fabsf(pitch_actual_deg) <= MAX_SAFE_ANGLE) {
    in_recovery_mode = false;
  }

  // Calculate Desired Rates with potential recovery boost
  angle_output.roll_rate_setpoint =
      roll_gain_mult *
      pid_calculate(&pid_roll_angle, roll_target, roll_actual_deg, dt_sec);
  angle_output.pitch_rate_setpoint =
      pitch_gain_mult *
      pid_calculate(&pid_pitch_angle, pitch_target, pitch_actual_deg, dt_sec);
}

const angle_output_t *angle_control_get_output(void) { return &angle_output; }

float angle_control_get_i_roll(void) {
  return pid_roll_angle.integral * pid_roll_angle.ki;
}

float angle_control_get_i_pitch(void) {
  return pid_pitch_angle.integral * pid_pitch_angle.ki;
}

bool angle_control_in_recovery(void) { return in_recovery_mode; }

void angle_control_freeze_integral(bool freeze) {
  pid_freeze_integral(&pid_roll_angle, freeze);
  pid_freeze_integral(&pid_pitch_angle, freeze);
}
