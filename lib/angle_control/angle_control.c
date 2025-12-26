#include "angle_control.h"
#include "../pid/pid.h"

// Tuning Constants (P-Only for now)
// Kp = 2.0 means for 10 degrees error, we ask for 20 deg/s correction rate.
#define ANGLE_ROLL_KP 2.0f
#define ANGLE_ROLL_KI 0.0f
#define ANGLE_ROLL_KD 0.0f

#define ANGLE_PITCH_KP 2.0f
#define ANGLE_PITCH_KI 0.0f
#define ANGLE_PITCH_KD 0.0f

// Safety Limits
#define MAX_RATE_SETPOINT_DPS 150.0f // Clamp desired rate to +/- 150 deg/s

// PID Instances
static pid_controller_t pid_roll_angle;
static pid_controller_t pid_pitch_angle;

// Output State
static angle_output_t angle_output;

void angle_control_init(void) {
  // Initialize PIDs with limits
  // Output limit is the Max Rate we want to command
  // Integral limit is 0 since Ki is 0, but good to set safely
  pid_init(&pid_roll_angle, ANGLE_ROLL_KP, ANGLE_ROLL_KI, ANGLE_ROLL_KD,
           MAX_RATE_SETPOINT_DPS, 0.0f);
  pid_init(&pid_pitch_angle, ANGLE_PITCH_KP, ANGLE_PITCH_KI, ANGLE_PITCH_KD,
           MAX_RATE_SETPOINT_DPS, 0.0f);

  angle_output.roll_rate_setpoint = 0.0f;
  angle_output.pitch_rate_setpoint = 0.0f;
}

void angle_control_update(float roll_actual_deg, float pitch_actual_deg,
                          float roll_desired_deg, float pitch_desired_deg,
                          float dt_sec) {

  // Calculate Desired Rates
  // PID Calculate: Error = Setpoint - Measurement
  // If we want 0 deg, and are at 10 deg, Error = -10.
  // Output = Kp * -10 = -20 deg/s.
  // Negative Rate means roll left, which corrects positive roll. Correct.

  angle_output.roll_rate_setpoint =
      pid_calculate(&pid_roll_angle, roll_desired_deg, roll_actual_deg, dt_sec);
  angle_output.pitch_rate_setpoint = pid_calculate(
      &pid_pitch_angle, pitch_desired_deg, pitch_actual_deg, dt_sec);
}

const angle_output_t *angle_control_get_output(void) { return &angle_output; }
