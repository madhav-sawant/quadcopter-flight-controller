# 🎓 ESP32 Quadcopter Firmware - Complete Explanation

**Author's Note**: This document explains every part of the firmware like a teacher explaining to a student. Each concept is broken down with "WHY" it's done this way, not just "WHAT" it does.

---

## 📋 Table of Contents

1. [System Overview](#-system-overview)
2. [The Control Loop - How a Drone Flies](#-the-control-loop---how-a-drone-flies)
3. [PID Controller - The Brain](#-pid-controller---the-brain)
4. [IMU - The Eyes and Ears](#-imu---the-eyes-and-ears)  
5. [Motor Mixer - The Muscles](#-motor-mixer---the-muscles)
6. [Rate Control - Inner Loop](#-rate-control---inner-loop)
7. [Angle Control - Outer Loop](#-angle-control---outer-loop)
8. [Configuration System](#-configuration-system)
9. [RC Receiver - The Remote Control](#-rc-receiver---the-remote-control)
10. [PWM Output - Talking to Motors](#-pwm-output---talking-to-motors)
11. [ADC - Battery Monitoring](#-adc---battery-monitoring)
12. [Main.c - Putting It All Together](#-mainc---putting-it-all-together)
13. [The Cascade Control Architecture](#-the-cascade-control-architecture)
14. [Bug Fixes We Made](#-bug-fixes-we-made)

---

## 🌐 System Overview

### What is this firmware?

This is a **flight controller firmware** for a quadcopter. It runs on an ESP32 microcontroller and does these jobs:

```
┌─────────────────────────────────────────────────────────────────┐
│                    QUADCOPTER CONTROL SYSTEM                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   [RC Remote] ──► [RX Receiver] ──► [ESP32 Flight Controller]   │
│                                              │                   │
│                                              ▼                   │
│   [Battery] ──► [ADC] ──────────────► [Safety Checks]           │
│                                              │                   │
│                                              ▼                   │
│   [IMU Sensor] ──────────────────────► [PID Controller]         │
│                                              │                   │
│                                              ▼                   │
│                                        [Motor Mixer]             │
│                                              │                   │
│                                              ▼                   │
│                                     [PWM to 4 Motors]            │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### The Basic Idea

1. **You move the joystick** → RC transmitter sends signal
2. **Receiver gets signal** → ESP32 reads it
3. **IMU measures orientation** → "Am I tilting? Which way?"
4. **PID calculates correction** → "How much should I fix?"
5. **Mixer distributes power** → "Which motors need more/less?"
6. **PWM drives motors** → Motors spin at calculated speeds
7. **Loop repeats 250 times per second!**

---

## 🔄 The Control Loop - How a Drone Flies

### Why 250Hz? (Every 4 milliseconds)

```c
#define CONTROL_LOOP_FREQ_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ)  // = 4000us = 4ms
```

**Why this frequency?**
- **Too slow (50Hz)**: Drone reacts slowly, feels "mushy", hard to control
- **Too fast (1000Hz)**: CPU can't keep up, sensor noise becomes problem
- **250Hz is the sweet spot**: Fast enough for stability, slow enough for the ESP32

### The Timer Callback

```c
static void IRAM_ATTR control_loop_callback(void *arg) {
  // This function runs exactly 250 times per second
  // IRAM_ATTR = "Put this code in fast RAM for quick execution"
```

**Why IRAM_ATTR?**
Normal code lives in Flash memory (slow). `IRAM_ATTR` puts the code in RAM (fast). For time-critical code that runs 250 times/second, every microsecond matters!

---

## 🧠 PID Controller - The Brain

### The Magic Formula

```
Output = P + I + D
       = (Kp × Error) + (Ki × ∫Error) + (Kd × dError/dt)
```

Let me explain each term like you're 10 years old:

### P-Term (Proportional) - "How far am I from where I want to be?"

```c
float error = setpoint - measurement;  // How far off?
float p_out = pid->kp * error;         // React proportionally
```

**Example**: 
- You want 0° tilt (setpoint = 0)
- Drone is at 10° tilt (measurement = 10)
- Error = 0 - 10 = -10°
- If Kp = 0.3: P-term = 0.3 × (-10) = -3

**Analogy**: Like a spring. The more you stretch it, the harder it pulls back.

**Problem with P alone**: It can never fully correct! There's always some "steady-state error".

### I-Term (Integral) - "How long have I been off?"

```c
if (!pid->integral_frozen) {
  pid->integral += error * dt_sec;  // Add up error over time
  // Clamp to prevent "windup"
  if (pid->integral > pid->integral_limit)
    pid->integral = pid->integral_limit;
}
float i_out = pid->ki * pid->integral;
```

**Why we need it**:
- P-term alone: Drone hovers at 5° when you want 0°
- I-term accumulates: "I've been off for 2 seconds, time to push harder!"
- Eventually eliminates any steady-state error

**The Windup Problem**:
```c
if (pid->integral > pid->integral_limit)
  pid->integral = pid->integral_limit;
```
If drone is stuck on ground, I-term keeps growing → when it takes off, MASSIVE correction → flip!
**Solution**: Clamp the integral to a maximum value.

**The Freeze Feature**:
```c
if (!pid->integral_frozen) {
```
When throttle is low (on ground), we FREEZE the I-term so it doesn't wind up.

### D-Term (Derivative) - "How fast am I changing?"

```c
// D-term: Derivative on MEASUREMENT (not error) to prevent derivative kick
float raw_derivative = -(measurement - pid->prev_measurement) / dt_sec;
pid->prev_measurement = measurement;
```

**Why derivative on MEASUREMENT, not ERROR?**

If pilot suddenly moves stick (setpoint changes instantly):
- `d(error)/dt` = HUGE spike → motor jerk!
- `d(measurement)/dt` = smooth (physical sensor can't jump instantly)

**The negative sign**: 
- If measurement is INCREASING, we want to slow down
- Negative sign makes the D-term work against the change

### D-Term Low-Pass Filter

```c
#define D_TERM_LPF_ALPHA 0.2f

pid->filtered_derivative =
    D_TERM_LPF_ALPHA * raw_derivative +
    (1.0f - D_TERM_LPF_ALPHA) * pid->filtered_derivative;
```

**Why filter the D-term?**
- Derivative amplifies noise: tiny sensor jitter → huge D output
- Low-pass filter smooths it out
- Alpha = 0.2 at 250Hz ≈ 40Hz cutoff (blocks noise above 40Hz)

**The formula is a simple IIR filter**:
```
new_value = 0.2 × raw + 0.8 × old_value
```
"Trust the new reading 20%, keep 80% of the old."

### The PID Structure

```c
typedef struct {
  float kp;                    // Proportional gain
  float ki;                    // Integral gain  
  float kd;                    // Derivative gain
  float output_limit;          // Max output (e.g., 350)
  float integral_limit;        // Max I-term accumulation
  float integral;              // Current accumulated error
  float prev_measurement;      // Last sensor reading (for D-term)
  float filtered_derivative;   // Smoothed D-term
  bool integral_frozen;        // Freeze I when on ground
} pid_controller_t;
```

---

## 👁️ IMU - The Eyes and Ears

### What is an IMU?

**IMU = Inertial Measurement Unit**

The MPU6050 contains:
- **Gyroscope**: Measures rotation RATE (degrees per second)
- **Accelerometer**: Measures acceleration (including gravity!)

### I2C Communication

```c
#define MPU6050_ADDR 0x68        // Sensor's address on the I2C bus
#define I2C_MASTER_FREQ_HZ 400000 // 400kHz = "Fast Mode" I2C
```

**Why I2C?**
- Only needs 2 wires (SDA for data, SCL for clock)
- Multiple sensors can share the same wires
- MPU6050 supports up to 400kHz

### Sensor Configuration

```c
// Set Digital Low Pass Filter to 20Hz
write_register(REG_CONFIG, DLPF_CFG_20HZ);

// Set Gyro range to ±2000 degrees/second
write_register(REG_GYRO_CONFIG, FS_SEL_2000);

// Set Accelerometer range to ±8G
write_register(REG_ACCEL_CONFIG, AFS_SEL_8G);
```

**Why 20Hz DLPF?**
- Motors vibrate at high frequencies (100-500Hz)
- DLPF filters this out IN HARDWARE (before we even read)
- 20Hz keeps the useful motion data, removes vibration noise

**Why ±2000°/s?**
- During flips/crashes, drone can spin 500+ °/s
- Need headroom to measure extreme movements
- Trade-off: Less resolution, but covers full range

### Reading Raw Data

```c
int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
```

**What's happening here?**
1. Sensor gives us 2 bytes: HIGH byte and LOW byte
2. `buffer[0] << 8` = shift HIGH byte left 8 bits
3. `| buffer[1]` = combine with LOW byte
4. Cast to `int16_t` = interpret as signed 16-bit number

**Example**:
- buffer[0] = 0x01, buffer[1] = 0x23
- (0x01 << 8) | 0x23 = 0x0123 = 291 in decimal

### Converting to Real Units

```c
#define GYRO_SCALE_FACTOR 16.4f   // For ±2000°/s range
#define ACCEL_SCALE_FACTOR 4096.0f // For ±8G range

imu_state.gyro_x_dps = (gx_raw / GYRO_SCALE_FACTOR) - gyro_bias_x;
```

**Where do these numbers come from?**
- MPU6050 outputs 16-bit signed values (-32768 to +32767)
- At ±2000°/s: 32767 / 2000 = 16.4 counts per degree/second
- At ±8G: 32767 / 8 = 4096 counts per G

### Gyro Calibration - Removing Bias

```c
void imu_calibrate_gyro(void) {
  const int samples = 1000;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  
  for (int i = 0; i < samples; i++) {
    // Read gyro 1000 times
    sum_x += raw_x;
    // ...
  }
  gyro_bias_x = (sum_x / samples) / GYRO_SCALE_FACTOR;
}
```

**Why calibrate?**
- Even when perfectly still, gyro readings aren't exactly 0
- This "bias" or "offset" drifts with temperature
- We measure the average while still, then subtract it forever

### Accelerometer Calibration

```c
void imu_calibrate_accel(void) {
  // Calculate the angle from accelerometer when quad is level
  float pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
  float roll = atan2f(ay_g, az_g) * RAD_TO_DEG;
  
  accel_offset_pitch = sum_pitch / samples;
  accel_offset_roll = sum_roll / samples;
}
```

**Why?**
- Frame might not be perfectly level when IMU is mounted
- This measures the "zero" position
- Now "level" means "same as calibration position"

### The Complementary Filter - Combining Gyro and Accel

```c
#define COMPLEMENTARY_ALPHA 0.96f

imu_state.pitch_deg =
    COMPLEMENTARY_ALPHA *
        (imu_state.pitch_deg + imu_state.gyro_y_dps * dt_sec) +
    (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
```

**The Problem**:
| Sensor | Good At | Bad At |
|--------|---------|--------|
| Gyroscope | Fast response, smooth | Drifts over time |
| Accelerometer | No drift (gravity is constant) | Noisy, affected by motion |

**The Solution - Combine them!**

```
angle = 0.96 × (old_angle + gyro × dt) + 0.04 × accel_angle
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^
        Trust gyro for short-term          Use accel to fix drift
```

**Why 0.96?**
- Higher = trust gyro more, smoother but slower drift correction
- Lower = trust accel more, faster drift correction but noisier
- 0.96 at 250Hz = about 1 second to correct 63% of drift

### First Read Initialization

```c
if (first_read) {
  imu_state.pitch_deg = accel_pitch;
  imu_state.roll_deg = accel_roll;
  first_read = false;
}
```

**Why?**
- Complementary filter needs a starting angle
- If we start at 0 but quad is at 30°, it takes forever to converge
- Initialize FROM accelerometer on first read = instant correct starting angle

---

## 💪 Motor Mixer - The Muscles

### What is Mixing?

The PID outputs roll/pitch/yaw corrections. The mixer decides HOW MUCH to speed up/slow down EACH motor.

```
      FRONT
    M2      M4
     \    /
      \  /
       \/
       /\
      /  \
     /    \
    M1      M3
      REAR
```

### The Mixing Formulas

```c
int32_t m1 = t - roll_pid + pitch_pid - yaw_pid;  // Rear Right CCW
int32_t m2 = t - roll_pid - pitch_pid + yaw_pid;  // Front Right CW
int32_t m3 = t + roll_pid + pitch_pid + yaw_pid;  // Rear Left CW
int32_t m4 = t + roll_pid - pitch_pid - yaw_pid;  // Front Left CCW
```

Let's decode this step by step:

### Roll Control (Left/Right Tilt)

**To roll RIGHT (right wing down):**
- Speed up LEFT motors (M3, M4)
- Slow down RIGHT motors (M1, M2)

```
M1 = t - roll_pid  (slow down)
M2 = t - roll_pid  (slow down)
M3 = t + roll_pid  (speed up)
M4 = t + roll_pid  (speed up)
```

### Pitch Control (Forward/Backward Tilt)

**To pitch NOSE UP:**
- Speed up REAR motors (M1, M3)
- Slow down FRONT motors (M2, M4)

```
M1 = t + pitch_pid  (speed up)
M2 = t - pitch_pid  (slow down)
M3 = t + pitch_pid  (speed up)
M4 = t - pitch_pid  (slow down)
```

### Yaw Control (Rotation) - THE TRICKY ONE!

**Physics of Yaw**:
- CCW spinning prop creates CW torque on frame
- CW spinning prop creates CCW torque on frame

**To yaw CLOCKWISE:**
- Speed up CW motors (M2, M3) → creates CCW torque → wait, that's wrong?

Actually, the REACTION force matters:
- CW props create MORE air flow when sped up
- This pushes the frame to rotate CCW
- To counter this and get CW rotation, we need CW props to push harder

**Our BUG FIX was here!** The signs were originally wrong.

```c
// FIXED: CCW motors (M1, M4) get -yaw_pid
//        CW motors (M2, M3) get +yaw_pid
M1 = t - yaw_pid;  // CCW
M2 = t + yaw_pid;  // CW  
M3 = t + yaw_pid;  // CW
M4 = t - yaw_pid;  // CCW
```

### Motor Clamping

```c
static uint16_t clamp_motor(int32_t val) {
  if (val < MIXER_IDLE_THROTTLE)  // 1100
    return MIXER_IDLE_THROTTLE;
  if (val > MIXER_MAX_THROTTLE)   // 1850
    return MIXER_MAX_THROTTLE;
  return (uint16_t)val;
}
```

**Why clamp?**
- Math might give M1 = 900 or M1 = 2100
- ESCs expect 1000-2000µs
- Below 1100: Motor might stop (bad for stability)
- Above 1850: Leave headroom for corrections

### Throttle Idle Detection

```c
if (t < MIXER_IDLE_THROTTLE + 50) {  // Below 1150
  mixer_is_throttle_idle = true;
  rate_control_freeze_integral(true);  // Freeze I-term!
  // Set all motors to idle
}
```

**Why freeze I-term at idle?**
- On the ground, wind or vibration causes small tilts
- PID tries to correct, I-term accumulates
- When you take off → HUGE I-term → FLIP!
- **Solution**: Freeze I-term when throttle is low

---

## 🔄 Rate Control - Inner Loop

### What is Rate Control?

It controls the **rotation RATE** (degrees per second).

**Example**:
- Pilot wants 0°/s rotation (hover still)
- Drone is rotating at 20°/s
- Rate controller outputs correction to stop the rotation

```c
void rate_control_update(float desired_roll_rate, float desired_pitch_rate,
                         float desired_yaw_rate, float gyro_roll_rate,
                         float gyro_pitch_rate, float gyro_yaw_rate) {
  rate_output.roll = pid_calculate(&pid_roll_rate, desired_roll_rate,
                                   gyro_roll_rate, RATE_LOOP_DT_SEC);
  // ... same for pitch and yaw
}
```

### Initialization

```c
void rate_control_init(void) {
  pid_init(&pid_roll_rate, sys_cfg.roll_kp, sys_cfg.roll_ki, sys_cfg.roll_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
```

**sys_cfg values come from the configuration system** (can be changed via webserver!)

---

## 📐 Angle Control - Outer Loop

### What is Angle Control?

It controls the **actual angle** (degrees from level).

**Example**:
- Pilot holds stick at 15° forward
- Drone should tilt to 15° and HOLD IT there
- Angle controller outputs the RATE needed to get there

```c
void angle_control_update(float roll_actual_deg, float pitch_actual_deg,
                          float roll_desired_deg, float pitch_desired_deg,
                          float dt_sec) {
  angle_output.roll_rate_setpoint =
      pid_calculate(&pid_roll_angle, roll_desired_deg, roll_actual_deg, dt_sec);
}
```

### The Output is a RATE!

Notice: Angle PID output → Rate setpoint

This creates the **CASCADE** structure:
```
[Angle Controller] → desired rate → [Rate Controller] → mixer output
```

### Why Cascade?

| Single Loop | Cascade |
|-------------|---------|
| "Achieve 15° tilt" | "Achieve 15° tilt by controlling rotation rate" |
| Hard to tune | Each loop is simple |
| Oscillates easily | Inner loop dampens outer loop |

---

## ⚙️ Configuration System

### Default Values

```c
void config_load_defaults(void) {
  // Roll Rate PID - Tuned for F450 + 1400KV motors
  sys_cfg.roll_kp = 0.35f;
  sys_cfg.roll_ki = 0.18f;
  sys_cfg.roll_kd = 0.1f;
  
  // Limits
  sys_cfg.rate_output_limit = 350.0f;   // Max PID output
  sys_cfg.crash_angle_deg = 45.0f;      // Disarm if tilted > 45°
}
```

### NVS Storage (Non-Volatile Storage)

```c
void config_save_to_nvs(void) {
  nvs_handle_t handle;
  nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  nvs_set_blob(handle, "roll_kp", &sys_cfg.roll_kp, sizeof(float));
  nvs_commit(handle);
  nvs_close(handle);
}
```

**Why NVS?**
- PID values persist after power off
- No need to reflash to change values
- Webserver changes are SAVED permanently

### Value Validation

```c
static bool load_float_clamped(nvs_handle_t handle, const char *key, 
                               float *out, float min_val, float max_val) {
  // Clamp to sane range to prevent corrupt NVS causing issues
  if (val < min_val)
    val = min_val;
  else if (val > max_val)
    val = max_val;
```

**Why validate?**
- Corrupted flash could give roll_kp = 9999999
- This would make the drone uncontrollable
- Clamping ensures values stay in safe ranges

---

## 📻 RC Receiver - The Remote Control

### PPM Signal

```c
#define RX_MIN_US 1000     // Minimum valid pulse
#define RX_MAX_US 2000     // Maximum valid pulse  
#define RX_SYNC_MIN_US 3000 // Sync gap between frames
```

**PPM (Pulse Position Modulation) works like this**:
```
  ___      _____      ___          _______________
_|   |____|     |____|   |________|               |___
   CH1       CH2       CH3            SYNC GAP
   (1500)  (1800)    (1200)
```

- Time between pulses = channel value
- Long gap (>3000µs) = start of new frame
- Repeat ~50 times per second

### Interrupt Service Routine

```c
static void IRAM_ATTR rx_isr_handler(void *arg) {
  int64_t now = esp_timer_get_time();
  int64_t dt = now - last_time_us;
  last_time_us = now;

  // Sync Pulse Detection
  if (dt > RX_SYNC_MIN_US) {
    current_channel = 0;  // Reset to channel 0
    connected = true;
  }
  else if (dt >= RX_MIN_US && dt <= RX_MAX_US) {
    rx_channels[current_channel] = (uint16_t)dt;
    current_channel++;
  }
}
```

**How it works**:
1. GPIO interrupt fires on every pulse edge
2. Measure time since last edge
3. If time > 3000µs → sync gap → reset channel counter
4. Else → it's a channel pulse → store value, move to next

### Failsafe

```c
bool rx_is_connected(void) {
  // Check if we received a frame recently (within 100ms)
  if (esp_timer_get_time() - last_frame_time_us > 100000) {
    connected = false;
  }
  return connected;
}
```

**Why?**
- If transmitter signal is lost, drone should know!
- If no valid frame for 100ms, declare "not connected"
- Main loop will DISARM the motors

---

## ⚡ PWM Output - Talking to Motors

### ESC Communication

ESCs (Electronic Speed Controllers) expect a PWM signal:
- 1000µs = Motor OFF
- 1500µs = 50% throttle
- 2000µs = 100% throttle

```c
#define PWM_FREQ_HZ 400       // 400Hz refresh rate
#define PWM_MIN_PULSE_US 1000
#define PWM_MAX_PULSE_US 2000
```

**Why 400Hz?**
- Faster update = smoother control
- Most ESCs support up to 400-500Hz
- Traditional servos need 50Hz, but ESCs can go faster

### ESP32 LEDC (PWM) Peripheral

```c
void pwm_init(void) {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_num = LEDC_TIMER,
    .duty_resolution = LEDC_DUTY_RES,  // 14-bit
    .freq_hz = LEDC_FREQUENCY,         // 400Hz
  };
```

**Why LEDC?**
- ESP32's hardware PWM peripheral
- Precise timing, no CPU load
- Multiple independent channels

### Converting Pulse Width to Duty Cycle

```c
void pwm_set_motor(int motor_index, uint32_t pulse_width_us) {
  uint32_t max_duty = (1 << PWM_RES_BIT) - 1;  // 2^14 - 1 = 16383
  uint32_t duty = (pulse_width_us * max_duty * PWM_FREQ_HZ) / 1000000ULL;
```

**The Math**:
- At 400Hz, period = 2500µs
- 1500µs pulse = 1500/2500 = 60% duty
- With 14-bit resolution: 60% × 16383 = 9830

---

## 🔋 ADC - Battery Monitoring

### Why Monitor Battery?

- LiPo batteries are damaged if discharged too low
- Voltage sag under load can cause brownouts
- Low battery = weak control = dangerous!

### Voltage Divider

```c
#define VOLTAGE_SCALE_MV 5
#define VOLTAGE_SCALE_DIV 1
```

**The Problem**: ESP32 ADC can only measure 0-3.3V, but battery is 11-12V!

**Solution**: Voltage divider on the board

```
VBAT ───[R1]───┬───[R2]───GND
               │
               └──► ADC Pin
```

The formula: `V_adc = V_bat × R2 / (R1 + R2)`

### IIR Smoothing

```c
adc_filtered_raw = (current >> 2) + (adc_filtered_raw - (adc_filtered_raw >> 2));
```

**What's >> 2?**
- Right shift by 2 = divide by 4
- This implements: `filtered = 0.25 × new + 0.75 × old`
- Removes noise from ADC readings

---

## 🚀 Main.c - Putting It All Together

### Boot Sequence

```c
void app_main(void) {
  // 1. CRITICAL: Set motor pins LOW immediately
  gpio_set_level(PWM_MOTOR_1_GPIO, 0);
  //... for all motors
```

**Why?**
- During ESP32 boot, GPIO pins are undefined (floating)
- ESC might see random voltage and arm motor!
- Setting pins LOW immediately = safe boot

### ESC Initialization

```c
for (int i = 0; i < 4; i++) {
  pwm_set_motor(i, 1000);  // Send "stop" signal
}
vTaskDelay(pdMS_TO_TICKS(3000));  // Wait 3 seconds
```

**Why 3 seconds?**
- ESCs need time to initialize
- Some ESCs play startup tones
- Sending consistent "stop" signal = ESC knows we're in control

### System Initialization Order

```c
// 1. NVS (for saved config)
nvs_flash_init();
config_load_defaults();
config_load_from_nvs();

// 2. GPIO (LED, button)
gpio_reset_pin(LED_PIN);

// 3. Hardware peripherals
adc_init();
rx_init();
mixer_init();

// 4. IMU with calibration
imu_init();
if (!imu_calibration_load_from_nvs()) {
  imu_calibrate_gyro();
  imu_calibrate_accel();
}

// 5. Control loops
rate_control_init();
angle_control_init();

// 6. Start timer
esp_timer_start_periodic(control_timer, CONTROL_LOOP_PERIOD_US);
```

**Order matters!**
- Config must load BEFORE control init (PID values needed)
- IMU must init BEFORE control loop starts
- Timer LAST (control loop needs everything ready)

### Arming State Machine

```c
bool rx_ok = rx_is_connected();
uint16_t rx_aux1 = rx_get_channel(4);
bool arm_switch_high = (rx_aux1 > 1600);

if (rx_ok && arm_switch_high && !error_state) {
  if (!system_armed) {
    // Check throttle safety
    if (rx_thr_check < 1150) {
      system_armed = true;
      mixer_arm(true);
    }
  }
}
```

**Safety requirements to ARM**:
1. RX signal must be connected
2. Arm switch must be HIGH
3. No error state
4. Throttle must be LOW (prevent takeoff surprise)

### Safety Checks

```c
// Crash angle detection
if (fabs(imu->roll_deg) > sys_cfg.crash_angle_deg ||
    fabs(imu->pitch_deg) > sys_cfg.crash_angle_deg) {
  mixer_arm(false);
  system_armed = false;
  error_state = true;
}

// Gyro rate detection (catch rapid spins)
if (fabs(imu->gyro_x_dps) > 500.0f || ...) {
  // Same disarm logic
}
```

**Why both angle AND rate?**
- Angle check: Catches tilting past safe angle
- Rate check: Catches impact/crash where angle filter is slow to respond

---

## 🔀 The Cascade Control Architecture

### The Full Picture

```
                    OUTER LOOP (Slower)           INNER LOOP (Faster)
                    ──────────────────           ──────────────────
[Stick Position]                                 
       │                                                    
       ▼                                                    
┌─────────────────┐                                         
│  Map to Angle   │                                         
│  (0-45 degrees) │                                         
└────────┬────────┘                                         
         │                                                  
         ▼                                                  
┌─────────────────┐     Desired      ┌─────────────────┐    Motor
│  ANGLE CONTROL  │───► Rate DPS ───►│  RATE CONTROL   │───► Commands
│     (PID)       │                  │     (PID)       │
└────────┬────────┘                  └────────┬────────┘
         │                                    │
         │  Actual Angle                      │  Actual Rate
         └───────────────┐    ┌───────────────┘
                         │    │
                         ▼    ▼
                    ┌──────────────┐
                    │     IMU      │
                    │  (Gyro+Accel)│
                    └──────────────┘
```

### Why This Works

1. **Angle Loop** says: "I want 0°, I'm at 10°, so give me -30°/s rate"
2. **Rate Loop** says: "I want -30°/s, I'm at -25°/s, push a bit more"
3. **Result**: Smooth, stable control

### When Angle P = 0 (Pure Rate Mode)

```c
// When P = 0:
angle_out->roll_rate_setpoint = pid_calculate(...);  // = 0 always!
```

- Angle loop output = 0
- Rate loop targets 0°/s rotation
- Result: Drone resists rotation but doesn't self-level

---

## 🔧 Bug Fixes We Made

### Fix 1: Pitch Axis Inversion

**Problem**: Drone flipped forward on takeoff

**Root Cause**: When IMU sensed pitch forward, mixer commanded motors to pitch forward MORE instead of correcting.

**Fix in mixer.c**:
```c
// BEFORE (Wrong):
int32_t m1 = t - roll_pid - pitch_pid + yaw_pid;

// AFTER (Correct):
int32_t m1 = t - roll_pid + pitch_pid - yaw_pid;
```

### Fix 2: Yaw Axis Inversion

**Problem**: Drone spun uncontrollably

**Root Cause**: When drone tried to stop spinning, it sped up the wrong motors.

**Fix in mixer.c**:
```c
// Yaw signs inverted for CCW/CW motor pairing:
// M1, M4 (CCW props) get -yaw_pid
// M2, M3 (CW props) get +yaw_pid
```

### Why Did This Happen?

The original code assumed a specific:
- Motor numbering (which motor is where)
- Motor rotation direction (CW or CCW)
- IMU orientation (which axis is which)

When ANY of these don't match assumptions, signs need adjustment!

---

## 📊 Quick Reference

### PID Tuning Guidelines

| Problem | Solution |
|---------|----------|
| Oscillation | Lower P, increase D |
| Slow response | Increase P |
| Drifting | Increase I (carefully) |
| Motor hot | Lower D |
| Overshoots | Lower P, increase D |
| Can't hold angle | Increase I |

### Safe Starting Values (Your Setup)

| Parameter | Rate Loop | Angle Loop |
|-----------|-----------|------------|
| P | 0.25-0.35 | 3.0 |
| I | 0.01-0.05 | 0.0 |
| D | 0.15-0.30 | 0.0 |

---

## 🎓 Summary

You now understand:

1. **PID** - The math that makes drones stable
2. **IMU** - How the drone knows its orientation  
3. **Mixer** - How PID outputs become motor speeds
4. **Cascade Control** - Why we have two loops
5. **Safety** - Multiple layers prevent crashes
6. **The bugs we fixed** - And WHY they happened

**Next Steps**:
- Enable Angle Mode (set Angle P to 3.0) for self-leveling
- Fine-tune PID values using blackbox data
- Add GPS hold, altitude hold, etc.!

---

*Created for the ESP32 Quadcopter Project - December 2024*
