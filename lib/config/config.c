#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

system_config_t sys_cfg;

#define NVS_NAMESPACE "pid_cfg"

void config_load_defaults(void) {
  // Roll Rate PID
  // TUNED for 1045 props + Correction for drift
  sys_cfg.roll_kp = 0.80f;
  sys_cfg.roll_ki = 1.50f; // Increased to hold angle against drift
  sys_cfg.roll_kd = 0.05f; // Slight D boost for 10" props

  // Pitch Rate PID (symmetric)
  sys_cfg.pitch_kp = 0.80f;
  sys_cfg.pitch_ki = 1.50f; // Increased to hold angle against drift
  sys_cfg.pitch_kd = 0.05f;

  // Yaw Rate PID (Yaw is mechanically weaker)
  sys_cfg.yaw_kp = 2.50f;
  sys_cfg.yaw_ki = 2.50f;
  sys_cfg.yaw_kd = 0.00f; // Yaw usually doesn't need D

  // Limits
  sys_cfg.rate_output_limit =
      110.0f; // Safer limit for 1400KV motors (Range 1100-1400)
  sys_cfg.rate_integral_limit =
      50.0f; // Limit I-term authority to 50 (Output limit is 110)

  // Angle Mode (Outer Loop) - Self-Leveling
  sys_cfg.angle_kp =
      5.0f; // Increased from 3.0 to 5.0 based on stability analysis
  sys_cfg.angle_ki = 0.5f; // Added I-term (0.5) to fix steady-state error/drift
  sys_cfg.angle_max = 45.0f; // Max tilt angle in degrees

  // Safety
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

  // Angle Mode
  nvs_set_blob(handle, "angle_kp", &sys_cfg.angle_kp, sizeof(float));

  nvs_commit(handle);
  nvs_close(handle);
}

// Load float with optional min/max validation
static bool load_float_clamped(nvs_handle_t handle, const char *key, float *out,
                               float min_val, float max_val) {
  size_t len = sizeof(float);
  float val;
  if (nvs_get_blob(handle, key, &val, &len) != ESP_OK)
    return false;
  // Clamp to sane range to prevent corrupt NVS causing issues
  if (val < min_val)
    val = min_val;
  else if (val > max_val)
    val = max_val;
  *out = val;
  return true;
}

static bool load_float(nvs_handle_t handle, const char *key, float *out) {
  return load_float_clamped(handle, key, out, -1000.0f, 1000.0f);
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

  // Angle Mode (optional - use default if not found)
  load_float(handle, "angle_kp", &sys_cfg.angle_kp);

  nvs_close(handle);
  return success;
}
