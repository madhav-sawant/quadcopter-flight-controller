#include "config.h"

system_config_t sys_cfg;

void config_load_defaults(void) {
  // Roll Rate PID
  sys_cfg.roll_kp = sys_cfg.pitch_kp = 1.2f;
  sys_cfg.roll_ki = sys_cfg.pitch_ki = 0.0f;
  sys_cfg.roll_kd = sys_cfg.pitch_kd = 0.00f;

  // Yaw Rate PID
  sys_cfg.yaw_kp = 3.0f;
  sys_cfg.yaw_ki = 0.0f;
  sys_cfg.yaw_kd = 0.0f;

  // Limits
  sys_cfg.rate_output_limit = 500.0f;
  sys_cfg.rate_integral_limit = 200.0f;

  // Safety
  sys_cfg.crash_angle_deg = 60.0f;
  sys_cfg.low_bat_threshold = 10500; // 10.5V
}
