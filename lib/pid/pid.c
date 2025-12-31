#include "pid.h"
#include <stdbool.h>

// D-term low-pass filter coefficient
// At 250Hz loop, alpha=0.15 gives ~30Hz cutoff - stronger filtering for high-KV
// vibration
#define D_TERM_LPF_ALPHA 0.42f

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_limit = output_limit;
  pid->integral_limit = integral_limit;
  pid->integral = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->filtered_derivative = 0.0f;
  pid->integral_frozen = false;
}

float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec) {
  float error = setpoint - measurement;

  // P-term: Proportional to error
  float p_out = pid->kp * error;

  // I-term: Only accumulate if not frozen (prevents windup during ground idle)
  if (!pid->integral_frozen) {
    pid->integral += error * dt_sec;
    if (pid->integral > pid->integral_limit)
      pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
      pid->integral = -pid->integral_limit;
  }
  float i_out = pid->ki * pid->integral;

  // D-term: Derivative on MEASUREMENT (not error) to prevent derivative kick
  // When setpoint changes suddenly, derivative of measurement is smooth
  // Negative sign because d(error)/dt = d(setpoint)/dt - d(measurement)/dt
  // and we want to dampen measurement changes
  float raw_derivative = -(measurement - pid->prev_measurement) / dt_sec;
  pid->prev_measurement = measurement;

  // Low-pass filter the derivative to reduce high-frequency noise
  pid->filtered_derivative =
      D_TERM_LPF_ALPHA * raw_derivative +
      (1.0f - D_TERM_LPF_ALPHA) * pid->filtered_derivative;
  float d_out = pid->kd * pid->filtered_derivative;

  // Sum and clamp output
  float output = p_out + i_out + d_out;
  if (output > pid->output_limit)
    output = pid->output_limit;
  else if (output < -pid->output_limit)
    output = -pid->output_limit;

  return output;
}

void pid_freeze_integral(pid_controller_t *pid, bool freeze) {
  pid->integral_frozen = freeze;
  // Don't reset integral on freeze - just pause accumulation
  // Integral is reset on arm via pid_init()
}
