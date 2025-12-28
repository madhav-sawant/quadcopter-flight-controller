#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

system_config_t sys_cfg;

#define NVS_NAMESPACE "pid_cfg"

void config_load_defaults(void) {
  // Roll Rate PID
  // Tuned for F450 + 1400KV + 8045 props (Dec 2024 - 3.5hr tuning session)
  // Conservative values for stable liftoff - increase gradually
  sys_cfg.roll_kp = 0.08f; // Low P to prevent oscillation
  sys_cfg.roll_ki = 0.01f; // Minimal I for rate loop
  sys_cfg.roll_kd = 0.08f; // Reduced D to dampen oscillations

  // Pitch Rate PID (same as roll for symmetric response)
  sys_cfg.pitch_kp = 0.08f;
  sys_cfg.pitch_ki = 0.01f;
  sys_cfg.pitch_kd = 0.08f;

  // Yaw Rate PID
  sys_cfg.yaw_kp = 2.0f; // Reduced from 3.0 for smoother yaw
  sys_cfg.yaw_ki = 0.02f;
  sys_cfg.yaw_kd = 0.15f;

  // Angle PID (outer loop - provides rate setpoint to inner loop)
  sys_cfg.angle_roll_kp = 1.0f;  // Reduced for gentler self-leveling
  sys_cfg.angle_roll_ki = 0.25f; // Helps correct steady-state drift
  sys_cfg.angle_roll_kd = 0.0f;

  sys_cfg.angle_pitch_kp = 1.0f;
  sys_cfg.angle_pitch_ki = 0.25f;
  sys_cfg.angle_pitch_kd = 0.0f;

  // Limits (reduced for high-gain motors)
  sys_cfg.rate_output_limit = 350.0f;
  sys_cfg.rate_integral_limit = 100.0f;

  // Safety (conservative for tuning - change to 60 when stable)
  sys_cfg.crash_angle_deg = 45.0f;
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
  nvs_set_blob(handle, "ang_roll_ki", &sys_cfg.angle_roll_ki, sizeof(float));
  nvs_set_blob(handle, "ang_roll_kd", &sys_cfg.angle_roll_kd, sizeof(float));
  nvs_set_blob(handle, "ang_pitch_kp", &sys_cfg.angle_pitch_kp, sizeof(float));
  nvs_set_blob(handle, "ang_pitch_ki", &sys_cfg.angle_pitch_ki, sizeof(float));
  nvs_set_blob(handle, "ang_pitch_kd", &sys_cfg.angle_pitch_kd, sizeof(float));

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

  // Load angle PID values with validation (0-20 is sane for angle P)
  success &= load_float_clamped(handle, "ang_roll_kp", &sys_cfg.angle_roll_kp,
                                0.0f, 20.0f);
  load_float_clamped(handle, "ang_roll_ki", &sys_cfg.angle_roll_ki, 0.0f,
                     5.0f); // Optional, don't fail
  load_float_clamped(handle, "ang_roll_kd", &sys_cfg.angle_roll_kd, 0.0f,
                     1.0f); // Optional
  success &= load_float_clamped(handle, "ang_pitch_kp", &sys_cfg.angle_pitch_kp,
                                0.0f, 20.0f);
  load_float_clamped(handle, "ang_pitch_ki", &sys_cfg.angle_pitch_ki, 0.0f,
                     5.0f); // Optional
  load_float_clamped(handle, "ang_pitch_kd", &sys_cfg.angle_pitch_kd, 0.0f,
                     1.0f); // Optional

  nvs_close(handle);
  return success;
}
