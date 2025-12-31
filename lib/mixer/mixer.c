#include "mixer.h"
#include "../pwm/pwm.h"

static bool mixer_armed = false;
static bool mixer_is_throttle_idle =
    false; // Track idle state for I-term freeze
static uint16_t motor_cmds[4] = {1000, 1000, 1000, 1000};

// Motor filtering REMOVED - direct output for fastest response (~0ms delay)

static uint16_t clamp_motor(int32_t val) {
  // When armed, motors should never go below IDLE or above MAX
  if (val < MIXER_IDLE_THROTTLE)
    return MIXER_IDLE_THROTTLE;
  if (val > MIXER_MAX_THROTTLE)
    return MIXER_MAX_THROTTLE;
  return (uint16_t)val;
}

void mixer_init(void) {
  mixer_armed = false;
  for (int i = 0; i < 4; i++)
    motor_cmds[i] = MIXER_STOP_CMD;
}

void mixer_update(uint16_t throttle_us, float roll_pid, float pitch_pid,
                  float yaw_pid) {
  if (!mixer_armed) {
    for (int i = 0; i < 4; i++) {
      motor_cmds[i] = MIXER_STOP_CMD;
      pwm_set_motor(i, MIXER_STOP_CMD);
    }
    return;
  }

  int32_t t = throttle_us;

  // If throttle is below idle threshold, just spin at idle without PID mixing
  // This prevents motor fluctuations when on the ground
  // NOTE: Integral freeze is handled by main.c - no duplicate call here
  if (t < MIXER_IDLE_THROTTLE + 50) { // Below 1150 = no PID mixing
    mixer_is_throttle_idle = true;
    for (int i = 0; i < 4; i++) {
      motor_cmds[i] = MIXER_IDLE_THROTTLE;
      pwm_set_motor(i, MIXER_IDLE_THROTTLE);
    }
    return;
  }

  // Throttle is above idle - main.c handles I-term freeze state
  mixer_is_throttle_idle = false;

  if (t > MIXER_MAX_THROTTLE)
    t = MIXER_MAX_THROTTLE;

  // QUAD-X MIXER (Per DOCUMENTATION.md)
  // Motor positions: 1=Rear Right, 2=Front Right, 3=Rear Left, 4=Front Left
  // Motor rotation: 1=CCW, 2=CW, 3=CW, 4=CCW (Standard Props-Out)
  //
  // Roll (+) = Right wing down  -> M3,M4 speed up, M1,M2 slow down
  // Pitch (+) = Nose up         -> M1,M3 speed up, M2,M4 slow down
  // Yaw (+) = Counter-Clockwise -> M2,M3 speed up (CW motors), M1,M4 slow down
  // (CCW motors)
  //

  // HARDWARE ADAPTATION:
  // Pitch sign is INVERTED relative to standard Quad-X to match specific
  // motor/ESC wiring.
  int32_t m1 = t - (int32_t)roll_pid + (int32_t)pitch_pid -
               (int32_t)yaw_pid; // Rear Right
  int32_t m2 = t - (int32_t)roll_pid - (int32_t)pitch_pid +
               (int32_t)yaw_pid; // Front Right
  int32_t m3 = t + (int32_t)roll_pid + (int32_t)pitch_pid +
               (int32_t)yaw_pid; // Rear Left
  int32_t m4 = t + (int32_t)roll_pid - (int32_t)pitch_pid -
               (int32_t)yaw_pid; // Front Left

  // Clamp and output motor values directly (NO filtering for fastest response)
  motor_cmds[0] = clamp_motor(m1);
  motor_cmds[1] = clamp_motor(m2);
  motor_cmds[2] = clamp_motor(m3);
  motor_cmds[3] = clamp_motor(m4);

  pwm_set_motor(0, motor_cmds[0]);
  pwm_set_motor(1, motor_cmds[1]);
  pwm_set_motor(2, motor_cmds[2]);
  pwm_set_motor(3, motor_cmds[3]);
}

void mixer_arm(bool armed) {
  mixer_armed = armed;
  if (!armed)
    mixer_update(1000, 0, 0, 0);
}

void mixer_get_outputs(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4) {
  *m1 = motor_cmds[0];
  *m2 = motor_cmds[1];
  *m3 = motor_cmds[2];
  *m4 = motor_cmds[3];
}

bool mixer_is_idle(void) { return mixer_is_throttle_idle; }
