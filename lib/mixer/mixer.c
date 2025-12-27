#include "mixer.h"
#include "../pwm/pwm.h"

static bool mixer_armed = false;
static uint16_t motor_cmds[4] = {1000, 1000, 1000, 1000};

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
  if (t < MIXER_IDLE_THROTTLE + 50) { // Below 1150 = no PID mixing
    for (int i = 0; i < 4; i++) {
      motor_cmds[i] = MIXER_IDLE_THROTTLE;
      pwm_set_motor(i, MIXER_IDLE_THROTTLE);
    }
    return;
  }

  if (t > MIXER_MAX_THROTTLE)
    t = MIXER_MAX_THROTTLE;

  // QUAD-X MIXER (Standard Betaflight/Cleanflight layout)
  // Motor positions: 1=Rear Right, 2=Front Right, 3=Rear Left, 4=Front Left
  // Motor rotation: 1=CCW, 2=CW, 3=CW, 4=CCW
  //
  // Roll (+) = Right wing down  -> M3,M4 speed up, M1,M2 slow down
  // Pitch (+) = Nose up         -> M2,M4 speed up, M1,M3 slow down
  // Yaw (+) = Clockwise         -> M1,M4 speed up (CCW motors), M2,M3 slow down
  // (CW motors)

  int32_t m1 = t - (int32_t)roll_pid - (int32_t)pitch_pid +
               (int32_t)yaw_pid; // Rear Right CCW
  int32_t m2 = t - (int32_t)roll_pid + (int32_t)pitch_pid -
               (int32_t)yaw_pid; // Front Right CW
  int32_t m3 = t + (int32_t)roll_pid - (int32_t)pitch_pid -
               (int32_t)yaw_pid; // Rear Left CW
  int32_t m4 = t + (int32_t)roll_pid + (int32_t)pitch_pid +
               (int32_t)yaw_pid; // Front Left CCW

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
