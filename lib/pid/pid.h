#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float output_limit;
  float integral_limit;
  float integral;
  float prev_error;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit);

float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec);

void pid_reset(pid_controller_t *pid);

#endif // PID_H
