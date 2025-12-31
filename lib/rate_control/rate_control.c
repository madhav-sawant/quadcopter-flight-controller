#include "rate_control.h"
#include "../pid/pid.h"

static pid_controller_t pid_roll_rate;
static pid_controller_t pid_pitch_rate;
static pid_controller_t pid_yaw_rate;
static rate_output_t rate_output;

void rate_control_init(void) {
  pid_init(&pid_roll_rate, sys_cfg.roll_kp, sys_cfg.roll_ki, sys_cfg.roll_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  pid_init(&pid_pitch_rate, sys_cfg.pitch_kp, sys_cfg.pitch_ki,
           sys_cfg.pitch_kd, sys_cfg.rate_output_limit,
           sys_cfg.rate_integral_limit);
  pid_init(&pid_yaw_rate, sys_cfg.yaw_kp, sys_cfg.yaw_ki, sys_cfg.yaw_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  rate_output.roll = 0.0f;
  rate_output.pitch = 0.0f;
  rate_output.yaw = 0.0f;
}

void rate_control_update(float desired_roll_rate, float desired_pitch_rate,
                         float desired_yaw_rate, float gyro_roll_rate,
                         float gyro_pitch_rate, float gyro_yaw_rate) {
  // Direct PID calculation - DLPF handles noise filtering in hardware
  rate_output.roll = pid_calculate(&pid_roll_rate, desired_roll_rate,
                                   gyro_roll_rate, RATE_LOOP_DT_SEC);
  rate_output.pitch = pid_calculate(&pid_pitch_rate, desired_pitch_rate,
                                    gyro_pitch_rate, RATE_LOOP_DT_SEC);
  rate_output.yaw = pid_calculate(&pid_yaw_rate, desired_yaw_rate,
                                  gyro_yaw_rate, RATE_LOOP_DT_SEC);
}

const rate_output_t *rate_control_get_output(void) { return &rate_output; }

float rate_control_get_i_roll(void) {
  return pid_roll_rate.integral * pid_roll_rate.ki;
}

float rate_control_get_i_pitch(void) {
  return pid_pitch_rate.integral * pid_pitch_rate.ki;
}

float rate_control_get_i_yaw(void) {
  return pid_yaw_rate.integral * pid_yaw_rate.ki;
}

void rate_control_freeze_integral(bool freeze) {
  pid_freeze_integral(&pid_roll_rate, freeze);
  pid_freeze_integral(&pid_pitch_rate, freeze);
  pid_freeze_integral(&pid_yaw_rate, freeze);
}
