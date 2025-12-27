#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

system_config_t sys_cfg;

#define NVS_NAMESPACE "pid_cfg"

void config_load_defaults(void) {
  // Roll Rate PID
  sys_cfg.roll_kp = 0.8f;
  sys_cfg.roll_ki = 0.2f;
  sys_cfg.roll_kd = 0.05f;

  // Pitch Rate PID
  sys_cfg.pitch_kp = 0.8f;
  sys_cfg.pitch_ki = 0.2f;
  sys_cfg.pitch_kd = 0.05f;

  // Yaw Rate PID (disabled for testing)
  sys_cfg.yaw_kp = 0.0f;
  sys_cfg.yaw_ki = 0.0f;
  sys_cfg.yaw_kd = 0.0f;

  // Angle PID (disabled - rate mode only)
  sys_cfg.angle_roll_kp = 0.0f;
  sys_cfg.angle_pitch_kp = 0.0f;

  // Limits
  sys_cfg.rate_output_limit = 400.0f;
  sys_cfg.rate_integral_limit = 200.0f;

  // Safety
  sys_cfg.crash_angle_deg = 60.0f;
  sys_cfg.low_bat_threshold = 10500; // 10.5V
}

void config_save_to_nvs(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK)
    return;

  // Save all PID values as blobs
  nvs_set_blob(handle, "roll_kp", &sys_cfg.roll_kp, sizeof(float));
  nvs_set_blob(handle, "roll_ki", &sys_cfg.roll_ki, sizeof(float));
  nvs_set_blob(handle, "roll_kd", &sys_cfg.roll_kd, sizeof(float));

  nvs_set_blob(handle, "pitch_kp", &sys_cfg.pitch_kp, sizeof(float));
  nvs_set_blob(handle, "pitch_ki", &sys_cfg.pitch_ki, sizeof(float));
  nvs_set_blob(handle, "pitch_kd", &sys_cfg.pitch_kd, sizeof(float));

  nvs_set_blob(handle, "yaw_kp", &sys_cfg.yaw_kp, sizeof(float));
  nvs_set_blob(handle, "yaw_ki", &sys_cfg.yaw_ki, sizeof(float));
  nvs_set_blob(handle, "yaw_kd", &sys_cfg.yaw_kd, sizeof(float));

  nvs_set_blob(handle, "ang_roll_kp", &sys_cfg.angle_roll_kp, sizeof(float));
  nvs_set_blob(handle, "ang_pitch_kp", &sys_cfg.angle_pitch_kp, sizeof(float));

  nvs_commit(handle);
  nvs_close(handle);
}

static bool load_float(nvs_handle_t handle, const char *key, float *out) {
  size_t len = sizeof(float);
  return nvs_get_blob(handle, key, out, &len) == ESP_OK;
}

bool config_load_from_nvs(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK)
    return false;

  bool success = true;

  // Load all values - if any fail, keep defaults
  success &= load_float(handle, "roll_kp", &sys_cfg.roll_kp);
  success &= load_float(handle, "roll_ki", &sys_cfg.roll_ki);
  success &= load_float(handle, "roll_kd", &sys_cfg.roll_kd);

  success &= load_float(handle, "pitch_kp", &sys_cfg.pitch_kp);
  success &= load_float(handle, "pitch_ki", &sys_cfg.pitch_ki);
  success &= load_float(handle, "pitch_kd", &sys_cfg.pitch_kd);

  success &= load_float(handle, "yaw_kp", &sys_cfg.yaw_kp);
  success &= load_float(handle, "yaw_ki", &sys_cfg.yaw_ki);
  success &= load_float(handle, "yaw_kd", &sys_cfg.yaw_kd);

  success &= load_float(handle, "ang_roll_kp", &sys_cfg.angle_roll_kp);
  success &= load_float(handle, "ang_pitch_kp", &sys_cfg.angle_pitch_kp);

  nvs_close(handle);
  return success;
}
