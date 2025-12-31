#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>
#include <stdint.h>

#define MIXER_IDLE_THROTTLE 1100 // Back to 1100 to allow motor differentiation
#define MIXER_MAX_THROTTLE 1500  // Limited for safe testing (was 1850)
#define MIXER_STOP_CMD 1000

void mixer_init(void);
void mixer_update(uint16_t throttle_us, float roll_pid, float pitch_pid,
                  float yaw_pid);
void mixer_arm(bool armed);
void mixer_get_outputs(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4);
bool mixer_is_idle(void); // Returns true when throttle below idle threshold

#endif // MIXER_H
