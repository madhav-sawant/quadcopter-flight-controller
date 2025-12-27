#include "pid.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_limit = output_limit;
  pid->integral_limit = integral_limit;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec) {
  float error = setpoint - measurement;
  float p_out = pid->kp * error;

  pid->integral += error * dt_sec;
  if (pid->integral > pid->integral_limit)
    pid->integral = pid->integral_limit;
  else if (pid->integral < -pid->integral_limit)
    pid->integral = -pid->integral_limit;
  float i_out = pid->ki * pid->integral;

  float derivative = (error - pid->prev_error) / dt_sec;
  float d_out = pid->kd * derivative;
  pid->prev_error = error;

  float output = p_out + i_out + d_out;
  if (output > pid->output_limit)
    output = pid->output_limit;
  else if (output < -pid->output_limit)
    output = -pid->output_limit;

  return output;
}
