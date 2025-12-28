# ESP32 Quadcopter Flight Controller Firmware: Complete Engineering Guide

> **A comprehensive line-by-line explanation for engineering students**
> 
> This document explains every design decision, language construct, and engineering principle behind a real-world UAV flight controller written in embedded C for the ESP32 microcontroller.

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Header Files and Preprocessor Macros](#2-header-files-and-preprocessor-macros)
3. [Constants and Configuration Parameters](#3-constants-and-configuration-parameters)
4. [Global Variables and Data Structures](#4-global-variables-and-data-structures)
5. [System Initialization Sequence](#5-system-initialization-sequence)
6. [Main Control Loop](#6-main-control-loop)
7. [IMU Driver and State Estimation](#7-imu-driver-and-state-estimation)
8. [PID Controller Implementation](#8-pid-controller-implementation)
9. [Cascaded Control Architecture](#9-cascaded-control-architecture)
10. [Motor Mixing Algorithm](#10-motor-mixing-algorithm)
11. [PWM Generation for ESCs](#11-pwm-generation-for-escs)
12. [RC Receiver Interface](#12-rc-receiver-interface)
13. [Safety and Failure Handling](#13-safety-and-failure-handling)
14. [Memory and Real-Time Considerations](#14-memory-and-real-time-considerations)

---

## 1. System Architecture Overview

### Control System Block Diagram

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌─────────────┐
│  RC Input   │────▶│  Angle Loop  │────▶│  Rate Loop  │────▶│   Mixer     │
│ (Setpoint)  │     │ (Outer PID)  │     │ (Inner PID) │     │ (Quad-X)    │
└─────────────┘     └──────────────┘     └─────────────┘     └─────────────┘
                           ▲                    ▲                    │
                           │                    │                    ▼
                    ┌──────┴──────┐      ┌──────┴──────┐     ┌─────────────┐
                    │  Angles     │      │  Gyro Rates │     │  4x Motors  │
                    │  (Fused)    │      │  (deg/s)    │     │  (PWM Out)  │
                    └─────────────┘      └─────────────┘     └─────────────┘
                           ▲                    ▲
                           └────────────────────┘
                                    │
                           ┌────────┴────────┐
                           │  Complementary  │
                           │     Filter      │
                           └────────┬────────┘
                                    ▲
                           ┌────────┴────────┐
                           │   MPU6050 IMU   │
                           │ (Accel + Gyro)  │
                           └─────────────────┘
```

### Why Cascaded Control?

The firmware uses a **cascaded PID architecture** with two loops:

1. **Outer Loop (Angle Control)**: Runs at 250 Hz. Compares desired angle to actual angle. Output is a *rate setpoint*.
2. **Inner Loop (Rate Control)**: Runs at 250 Hz. Compares desired rate to actual gyro rate. Output is *motor command*.

**Why not a single loop?** A single loop would have to convert angle error directly to motor commands. This creates several problems:

- **Derivative term issues**: Taking the derivative of angle introduces noise and lag
- **Coupling problems**: Roll/pitch/yaw dynamics interact in complex ways
- **Tuning difficulty**: One set of gains can't handle both position and velocity control well

The cascaded approach separates concerns: the outer loop handles *where you want to be*, the inner loop handles *how fast you're getting there*.

---

## 2. Header Files and Preprocessor Macros

### main.c Header Includes

```c
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
```

#### Line-by-Line Analysis

**`#include "driver/gpio.h"`**
- **What**: ESP-IDF GPIO driver header
- **Why needed**: Configures digital I/O pins for LED, button, and motor outputs
- **Hardware level**: Provides access to GPIO matrix registers
- **If removed**: Cannot control LED or read emergency stop button

**`#include "esp_attr.h"`**
- **What**: ESP32 attribute macros
- **Why needed**: Provides `IRAM_ATTR` for placing ISR code in IRAM
- **Critical for**: Ensuring timer callbacks execute from fast internal RAM, not flash
- **If removed**: Timer ISR would run from flash, causing unpredictable latency (flash cache misses)

**`#include "esp_timer.h"`**
- **What**: High-resolution timer API
- **Why needed**: Creates precise 250 Hz control loop timer
- **Alternative considered**: FreeRTOS software timers (rejected due to jitter)
- **Design decision**: Hardware timer guarantees microsecond-accurate intervals

**`#include "freertos/FreeRTOS.h"` and `#include "freertos/task.h"`**
- **What**: Real-time operating system primitives
- **Why needed**: `vTaskDelay()` for non-blocking waits, task scheduling
- **Memory consideration**: Each FreeRTOS task has its own stack (minimum 2KB recommended)
- **If removed**: Would need bare-metal timing loops (wastes CPU cycles)

**`#include "nvs_flash.h"`**
- **What**: Non-Volatile Storage (flash filesystem)
- **Why needed**: Persist PID gains and calibration across power cycles
- **Wear consideration**: NVS uses wear-leveling to prevent flash degradation
- **If removed**: Must recalibrate and tune every power cycle

**`#include <math.h>`**
- **What**: Standard C math library
- **Why needed**: `fabs()`, `atan2f()`, `sqrtf()` for angle calculations
- **Why `f` suffix**: `atan2f()` is float version; `atan2()` is double. ESP32 has single-precision FPU only
- **Performance**: Float operations ~1 cycle on ESP32; double ~10x slower (software emulation)

**`#include <stdbool.h>`**
- **What**: C99 Boolean type
- **Why needed**: `bool`, `true`, `false` keywords
- **Why not `int`**: Explicit boolean type improves code clarity and compiler optimization
- **Memory**: `bool` is typically 1 byte, same as `uint8_t`

**`#include <stdint.h>`**
- **What**: Fixed-width integer types
- **Why needed**: `uint16_t`, `int16_t`, `int32_t` for precise data sizes
- **Critical for**: I2C data parsing, PWM values, timer calculations
- **Why not `int`**: `int` size varies by platform (16/32/64-bit); embedded code needs guarantees

### Include Guard Pattern

```c
#ifndef IMU_H
#define IMU_H
// ... header contents ...
#endif // IMU_H
```

**What it does**: Prevents multiple inclusion of the same header

**Why required**: 
- C preprocessor textually inserts headers
- Without guards: `struct imu_data_t` would be defined twice → compilation error
- **All headers must have guards in embedded C**

**How an engineer derives this**:
1. Try compiling without guard
2. See "redefinition of struct" error
3. Add standard guard pattern

---

## 3. Constants and Configuration Parameters

### Timing Constants (main.c)

```c
#define CONTROL_LOOP_FREQ_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ)
```

**Line 1: Loop frequency = 250 Hz**
- **Why 250 Hz?** This is a control theory decision:
  - Nyquist theorem: Sample at least 2x the fastest dynamics you want to control
  - Quadcopter roll/pitch: Natural frequency ~5-10 Hz
  - With margin: 10x oversampling → 100 Hz minimum
  - 250 Hz provides headroom for derivative term, filter bandwidth
  - **Not 1000 Hz**: ESP32 I2C IMU read takes ~1ms; 1kHz impossible

- **Why not 500 Hz?** 
  - Would work, but IMU DLPF (Digital Low-Pass Filter) set to 20 Hz
  - Higher loop rate won't capture more signal, just more noise

**Line 2: Period in microseconds**
- **Why calculated as macro?** Compile-time computation → no runtime division
- **Result**: 4000 µs = 4 ms per iteration
- **Why microseconds?** Timer API uses µs; avoids floating-point in timing code

### Motor Limits (mixer.h)

```c
#define MIXER_IDLE_THROTTLE 1100
#define MIXER_MAX_THROTTLE 1850
#define MIXER_STOP_CMD 1000
```

**MIXER_IDLE_THROTTLE = 1100**
- **What**: Minimum throttle when armed (motors spinning slowly)
- **Why 1100, not 1000?** 
  - ESCs typically arm at ~1000 µs pulse
  - At 1000 µs, motors may not spin reliably
  - 1100 µs guarantees all motors spin even with ESC variation
- **Safety purpose**: Spinning props are visible/audible warning that system is armed

**MIXER_MAX_THROTTLE = 1850**
- **What**: Maximum commanded throttle
- **Why not 2000?** Two reasons:
  1. **Control headroom**: PID needs room to add/subtract for stabilization
  2. If throttle = 2000 and PID says "speed up motor 1", it can't
  3. 150 µs margin ensures PID always has authority
- **Physical effect**: ~7.5% throttle reserved for control

**MIXER_STOP_CMD = 1000**
- **What**: Command that stops motors (minimum ESC pulse)
- **Why separate from IDLE?** 
  - When disarmed: send 1000 (stopped)
  - When armed but low throttle: send 1100 (idle spin)
  - Clear semantic distinction

### PWM Configuration (pwm.h)

```c
#define PWM_FREQ_HZ 250
#define PWM_RES_BIT 12
#define PWM_MIN_PULSE_US 1000
#define PWM_MAX_PULSE_US 2000
```

**PWM_FREQ_HZ = 250**
- **What**: PWM update rate for ESCs
- **Why 250 Hz?**
  - Standard ESCs expect 50-500 Hz
  - Higher = faster response, but diminishing returns
  - 250 Hz matches control loop for synchronization
  - **Matches loop rate**: New motor command every control cycle

**PWM_RES_BIT = 12**
- **What**: Timer resolution (12-bit = 4096 levels)
- **Why 12-bit?**
  - At 250 Hz, period = 4000 µs
  - Pulse range = 1000-2000 µs (1000 µs range)
  - 12-bit → 4096 steps → 4000/4096 ≈ 0.98 µs resolution
  - 1000 µs range / 0.98 µs = ~1020 discrete throttle levels
  - Sufficient for smooth motor control

### IMU Configuration (imu.c)

```c
#define MPU6050_ADDR 0x68
#define DLPF_CFG_20HZ 0x04
#define FS_SEL_2000 0x18
#define AFS_SEL_8G 0x10
#define COMPLEMENTARY_ALPHA 0.96f
#define GYRO_SCALE_FACTOR 16.4f
#define ACCEL_SCALE_FACTOR 4096.0f
```

**MPU6050_ADDR = 0x68**
- **What**: 7-bit I2C slave address
- **How derived**: MPU6050 datasheet; AD0 pin grounded selects 0x68

**DLPF_CFG_20HZ = 0x04**
- **What**: Digital Low-Pass Filter setting
- **Why 20 Hz?**
  - Quadcopter motors vibrate at ~100-400 Hz
  - 20 Hz cutoff aggressively filters motor noise
  - Tradeoff: Adds ~8ms group delay (acceptable at 250 Hz loop)
- **If removed**: Gyro readings would contain high-frequency noise → oscillations

**FS_SEL_2000 = 0x18**
- **What**: Gyro full-scale range ±2000 deg/s
- **Why 2000?**
  - Fast flips can exceed 500 deg/s
  - 2000 provides headroom without sacrificing too much resolution
- **GYRO_SCALE_FACTOR = 16.4**: Raw ADC counts per deg/s at ±2000 range

**AFS_SEL_8G = 0x10**
- **What**: Accelerometer full-scale range ±8g
- **Why 8g?**
  - Aggressive maneuvers can hit 2-3g
  - Crashes can spike higher
  - ±8g captures most scenarios with reasonable resolution
- **ACCEL_SCALE_FACTOR = 4096.0**: LSB per g at ±8g range

**COMPLEMENTARY_ALPHA = 0.96f**
- **What**: Complementary filter coefficient
- **Meaning**: 
  - 96% weight on gyro integration
  - 4% weight on accelerometer
- **Why this ratio?**
  - Gyro: Fast, accurate short-term, but drifts
  - Accel: Noisy, but no drift (gravity reference)
  - At 250 Hz: Time constant ≈ 1/(250 × 0.04) ≈ 0.1 seconds
  - Corrects gyro drift in ~100ms, fast enough to prevent significant error

---

## 4. Global Variables and Data Structures

### State Variables (main.c)

```c
static volatile bool control_loop_flag = false;
static int debug_counter = 0;
bool system_armed = false;
static bool error_state = false;
```

**`static volatile bool control_loop_flag`**
- **`static`**: File scope only; not visible to other .c files
- **`volatile`**: Tells compiler "this variable can change outside normal program flow"
- **Why volatile?** Modified by timer ISR, read by main loop
- **If volatile removed**: Compiler might optimize away the check, causing infinite loop

**`bool system_armed`**
- **Not static**: Other modules (mixer.c) need to read it
- **extern declaration**: Would be in header if multiple files needed it
- **Safety-critical**: Controls whether motors can spin

### IMU Data Structure

```c
typedef struct {
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float roll_deg;
  float pitch_deg;
} imu_data_t;
```

**Why a struct?**
- Groups related data together
- Passed as single pointer (4 bytes) instead of 8 floats (32 bytes)
- Enables atomic updates (write all fields, then publish pointer)

**Field naming convention**: `_g` for gravitational units, `_dps` for degrees/second
- Self-documenting code
- Prevents unit confusion bugs

**Why `float` not `double`?**
- ESP32 has single-precision FPU
- `double` operations are software-emulated (10x slower)
- 32-bit float precision (7 significant digits) sufficient for angles

### PID Controller Structure

```c
typedef struct {
  float kp;
  float ki;
  float kd;
  float output_limit;
  float integral_limit;
  float integral;
  float prev_measurement;
  float filtered_derivative;
  bool integral_frozen;
} pid_controller_t;
```

**Why `prev_measurement` instead of `prev_error`?**
- Enables **derivative-on-measurement** technique
- Standard PID: D = Kd × d(error)/dt
- Problem: If setpoint changes suddenly, derivative spikes
- Solution: D = -Kd × d(measurement)/dt
- Setpoint changes don't cause derivative kick

**`integral_frozen` flag**
- Purpose: Prevent integral windup on ground
- When throttle low: Freeze I-term accumulation
- Without this: I-term builds up while waiting on ground → aggressive response on takeoff

---

## 5. System Initialization Sequence

### Critical ESC Boot Fix

```c
void app_main(void) {
  // ========== CRITICAL: ESC BOOT FIX ==========
  gpio_reset_pin(PWM_MOTOR_1_GPIO);
  gpio_reset_pin(PWM_MOTOR_2_GPIO);
  gpio_reset_pin(PWM_MOTOR_3_GPIO);
  gpio_reset_pin(PWM_MOTOR_4_GPIO);
  gpio_set_direction(PWM_MOTOR_1_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_2_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_3_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_4_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(PWM_MOTOR_1_GPIO, 0);
  gpio_set_level(PWM_MOTOR_2_GPIO, 0);
  gpio_set_level(PWM_MOTOR_3_GPIO, 0);
  gpio_set_level(PWM_MOTOR_4_GPIO, 0);
```

**What problem does this solve?**

During ESP32 boot:
1. GPIO pins are in undefined state (floating)
2. ESCs continuously sample their input signal
3. Floating pin may be read as random pulses
4. ESC might interpret this as valid throttle command
5. **Motors could spin unexpectedly during boot** ← DANGEROUS

**How the engineer derived this solution:**
1. Observed motors twitching on power-up
2. Traced to GPIO state during boot sequence
3. Solution: Force pins LOW immediately, before any other code

**Why repeat for all 4 pins?**
- Each GPIO must be individually configured
- No ESP-IDF function to configure multiple pins atomically

**`gpio_reset_pin()`** 
- Disconnects pin from any previous peripheral
- Returns to default state
- Required before reconfiguring

### PWM Initialization and ESC Arming

```c
pwm_init();

for (int i = 0; i < 4; i++) {
  pwm_set_motor(i, 1000);
}
vTaskDelay(pdMS_TO_TICKS(3000));
```

**Why send 1000 µs for 3 seconds?**

ESC arming sequence:
1. ESCs power on expecting minimum throttle signal
2. Must see stable low signal for 2-3 seconds
3. ESC then "arms" and becomes responsive
4. If signal is erratic, ESC stays in safe mode

**`pdMS_TO_TICKS(3000)`**
- Converts milliseconds to FreeRTOS ticks
- Tick rate configurable (typically 100 Hz = 10ms tick)
- Abstractly: "delay for 3000 ms worth of ticks"

### NVS Initialization

```c
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
  ESP_ERROR_CHECK(nvs_flash_erase());
  ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);
```

**Why check for these specific errors?**

- `ESP_ERR_NVS_NO_FREE_PAGES`: Flash partition is full/corrupted
- `ESP_ERR_NVS_NEW_VERSION_FOUND`: NVS format version mismatch after firmware update

**Recovery strategy**: Erase and reinitialize
- Loses saved PID gains but allows boot to continue
- Better than failing to boot entirely

**`ESP_ERROR_CHECK()`**
- Macro that aborts program on error
- Appropriate here: If NVS truly fails, system cannot function safely

### IMU Initialization

```c
if (imu_init() != ESP_OK) {
  printf("IMU Init Failed!\n");
  while (1)
    vTaskDelay(100);
}
```

**Why infinite loop on IMU failure?**

- IMU is essential for flight control
- Without it: No attitude data → no control → crash
- Infinite loop prevents dangerous operation
- LED could blink error pattern (future enhancement)

**Inside `imu_init()`:**

```c
// Check WHO_AM_I
uint8_t who_am_i;
if (read_registers(REG_WHO_AM_I, &who_am_i, 1) != ESP_OK) {
  return ESP_FAIL;
}
if (who_am_i != MPU6050_ADDR) {
  return ESP_FAIL;
}
```

**WHO_AM_I register verification**:
- Every I2C device has identification register
- MPU6050 returns 0x68 when read
- If response differs: Wrong sensor or wiring fault
- Catches hardware problems early

### Control Loop Timer Setup

```c
const esp_timer_create_args_t timer_args = {
    .callback = &control_loop_callback,
    .name = "control_loop",
    .dispatch_method = ESP_TIMER_TASK,
};
esp_timer_handle_t control_timer;
ESP_ERROR_CHECK(esp_timer_create(&timer_args, &control_timer));
ESP_ERROR_CHECK(esp_timer_start_periodic(control_timer, CONTROL_LOOP_PERIOD_US));
```

**`ESP_TIMER_TASK` vs `ESP_TIMER_ISR`**

- `ESP_TIMER_ISR`: Callback runs in ISR context (highest priority, most restrictions)
- `ESP_TIMER_TASK`: Callback runs from high-priority task (can use FreeRTOS APIs)

**Why TASK dispatch?**
- Control loop calls I2C functions
- I2C uses FreeRTOS semaphores internally
- Semaphores cannot be used from ISR context
- TASK dispatch allows full API access with minimal latency penalty

---

## 6. Main Control Loop

### Timer Callback Structure

```c
static void IRAM_ATTR control_loop_callback(void *arg) {
  (void)arg;
  int64_t start_time = esp_timer_get_time();
  
  // 1. Read IMU
  // 2. Safety Checks
  // 3. Read RX
  // 4. Angle Control
  // 5. Rate Control
  // 6. Mixer Update
  // 7. Debug/Logging
}
```

**`IRAM_ATTR`**
- Places function code in Internal RAM (not flash)
- Flash access can be blocked during WiFi operations
- IRAM access is always available
- **Critical for timing**: Prevents cache-miss jitter

**`(void)arg`**
- Explicitly ignores unused parameter
- Prevents compiler warning
- C idiom for "I know this exists but don't need it"

**Why measure execution time?**
```c
int64_t start_time = esp_timer_get_time();
// ... loop body ...
int64_t end_time = esp_timer_get_time();
debug_exec_time_us = end_time - start_time;
```

- Verify loop completes within deadline (4000 µs)
- If execution > period: Loop falls behind
- Logged to blackbox for post-flight analysis

### IMU Read and Processing

```c
imu_read(1.0f / CONTROL_LOOP_FREQ_HZ);
const imu_data_t *imu = imu_get_data();
```

**Why pass `dt` to imu_read?**
- Complementary filter needs time delta for gyro integration
- angle += gyro_rate × dt
- Passing dt rather than computing inside avoids coupling

**Why return pointer instead of struct?**

```c
const imu_data_t *imu = imu_get_data();
```

vs

```c
imu_data_t imu = imu_get_data();  // Would copy 32 bytes
```

- Pointer return: 4 bytes copied
- Struct return: 32 bytes copied (8 floats)
- `const` pointer: Caller can read but not modify
- Pattern: "Read-only access to internal state"

### RC Input Mapping

```c
float target_roll = 0.0f;
if (abs(rx_roll - 1500) > RC_DEADBAND_US) {
  target_roll = (float)(rx_roll - 1500) / 500.0f * RC_MAX_ANGLE_DEG;
}
```

**Deadband application**
- RC sticks have mechanical centering imprecision
- Without deadband: Tiny drift causes constant small commands
- 20 µs deadband: Ignore ±20 µs from center (1500 µs)

**Mapping calculation breakdown:**
1. `rx_roll - 1500`: Center to zero (range: -500 to +500)
2. `/ 500.0f`: Normalize to -1.0 to +1.0
3. `* RC_MAX_ANGLE_DEG`: Scale to ±45 degrees

**Why check deadband first?**
- If within deadband: target = 0 (level flight)
- Cleaner than computing tiny non-zero angle

---

## 7. IMU Driver and State Estimation

### I2C Communication

```c
static esp_err_t write_register(uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}
```

**I2C Write Sequence:**
1. **START condition**: SDA goes low while SCL high
2. **Address byte**: 7-bit address + R/W bit (0 = write)
3. **Register address**: Which internal register to write
4. **Data byte**: Value to write
5. **STOP condition**: SDA goes high while SCL high

**`(MPU6050_ADDR << 1) | I2C_MASTER_WRITE`**
- I2C addresses are 7-bit; LSB is R/W flag
- Shift left to make room for R/W bit
- OR with 0 for write, 1 for read

**`i2c_cmd_link_create/delete`**
- ESP-IDF command queue pattern
- Build sequence of I2C operations, execute atomically
- Memory efficient for variable-length transactions

### Raw Data Parsing

```c
int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
```

**Why this byte order?**
- MPU6050 uses big-endian format
- High byte first (buffer[0]), low byte second (buffer[1])
- ESP32 is little-endian
- Manual reconstruction required

**Why cast to `int16_t`?**
- Raw values are signed 16-bit
- Gyro can be positive or negative
- Default C promotion to `int` would give wrong sign for negative values

### Gyro Calibration

```c
void imu_calibrate_gyro(void) {
  const int samples = 1000;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  
  for (int i = 0; i < samples; i++) {
    // Read and accumulate
    sum_x += raw_x;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  gyro_bias_x = (sum_x / samples) / GYRO_SCALE_FACTOR;
}
```

**Why calibrate gyro?**
- MEMS gyros have DC offset (bias)
- Even stationary, reports small non-zero rate
- Bias integrates over time → angle drift
- Calibration measures and subtracts this bias

**Why 1000 samples?**
- More samples = better average (noise cancels)
- 1000 @ 2ms delay = 2 seconds calibration
- Balance between accuracy and user patience

**Why save to NVS?**
- Bias is relatively stable (temperature-dependent)
- Save at first calibration
- Skip calibration on subsequent boots (faster startup)

### Complementary Filter

```c
imu_state.pitch_deg =
    COMPLEMENTARY_ALPHA * (imu_state.pitch_deg + imu_state.gyro_y_dps * dt_sec) +
    (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
```

**Mathematical breakdown:**

$$\theta_{n} = \alpha \cdot (\theta_{n-1} + \omega \cdot \Delta t) + (1 - \alpha) \cdot \theta_{accel}$$

Where:
- $\theta_{n}$ = New angle estimate
- $\alpha$ = 0.96 (complementary coefficient)
- $\theta_{n-1}$ = Previous angle estimate
- $\omega$ = Gyro rate (deg/s)
- $\Delta t$ = Time step (0.004 s)
- $\theta_{accel}$ = Angle from accelerometer

**Intuition:**
- **Gyro integration** (first term): Fast, accurate short-term, but drifts
- **Accelerometer angle** (second term): Noisy, but absolute reference (gravity)
- **Blending**: Gyro dominates for fast motions; accel slowly corrects drift

**First-read initialization:**
```c
if (first_read) {
  imu_state.pitch_deg = accel_pitch;
  imu_state.roll_deg = accel_roll;
  first_read = false;
}
```

**Why special case first read?**
- Filter starts with angle = 0
- If drone boots tilted, takes many iterations to converge
- Initialize directly from accelerometer for instant correct orientation

---

## 8. PID Controller Implementation

### Full PID Calculate Function

```c
float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec) {
  float error = setpoint - measurement;

  // P-term
  float p_out = pid->kp * error;

  // I-term with freeze
  if (!pid->integral_frozen) {
    pid->integral += error * dt_sec;
    if (pid->integral > pid->integral_limit)
      pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
      pid->integral = -pid->integral_limit;
  }
  float i_out = pid->ki * pid->integral;

  // D-term on measurement
  float raw_derivative = -(measurement - pid->prev_measurement) / dt_sec;
  pid->prev_measurement = measurement;
  
  // Low-pass filter derivative
  pid->filtered_derivative =
      D_TERM_LPF_ALPHA * raw_derivative +
      (1.0f - D_TERM_LPF_ALPHA) * pid->filtered_derivative;
  float d_out = pid->kd * pid->filtered_derivative;

  // Sum and clamp
  float output = p_out + i_out + d_out;
  if (output > pid->output_limit)
    output = pid->output_limit;
  else if (output < -pid->output_limit)
    output = -pid->output_limit;

  return output;
}
```

### P-Term Analysis

```c
float p_out = pid->kp * error;
```

**What it does**: Proportional response to current error
**Physical meaning**: "I'm off by X degrees, apply X×Kp correction"

**If Kp too low**: Sluggish response, doesn't track setpoint well
**If Kp too high**: Overshoot, oscillation, instability

### I-Term Analysis

```c
pid->integral += error * dt_sec;
```

**What it does**: Accumulates error over time

**Physical meaning**: "I've been off by this much for this long"

**Purpose**: Eliminates steady-state error
- Example: Wind pushes drone left
- P-term counters, but can't fully eliminate offset (needs constant error to produce constant output)
- I-term builds up until steady-state error is zero

**Anti-windup:**
```c
if (pid->integral > pid->integral_limit)
  pid->integral = pid->integral_limit;
```

**What is windup?**
- Drone on ground, props not spinning
- I-term accumulates large value (trying to correct un-correctable error)
- On takeoff: Massive I-term causes aggressive, uncontrolled response

**Integral freeze:**
```c
if (!pid->integral_frozen) {
  pid->integral += error * dt_sec;
}
```

**When frozen**: At low throttle (ground idle)
**Why**: No point accumulating integral that can't produce motor output

### D-Term Analysis

```c
float raw_derivative = -(measurement - pid->prev_measurement) / dt_sec;
```

**Derivative on measurement (not error):**

Standard D-term: `Kd × d(error)/dt`
Problem: If setpoint changes suddenly (stick input), derivative spikes

Our D-term: `Kd × d(measurement)/dt`
Result: Only responds to actual aircraft motion, not command changes

**Negative sign**: 
- We want to dampen *increases* in measurement
- If measurement increases, derivative is positive
- Negative sign makes output oppose the increase

**Low-pass filter:**
```c
pid->filtered_derivative =
    D_TERM_LPF_ALPHA * raw_derivative +
    (1.0f - D_TERM_LPF_ALPHA) * pid->filtered_derivative;
```

**Why filter derivative?**
- Derivative amplifies high-frequency content
- Sensor noise gets amplified
- Filter smooths derivative, reduces noise sensitivity
- α = 0.2: Cutoff ~40 Hz at 250 Hz loop rate

---

## 9. Cascaded Control Architecture

### Angle Control (Outer Loop)

```c
void angle_control_update(float roll_actual_deg, float pitch_actual_deg,
                          float roll_desired_deg, float pitch_desired_deg,
                          float dt_sec) {
  angle_output.roll_rate_setpoint =
      pid_calculate(&pid_roll_angle, roll_desired_deg, roll_actual_deg, dt_sec);
}
```

**Input**: Desired angle (from RC), Actual angle (from IMU)
**Output**: Desired rate (deg/s) for the inner loop

**Physical interpretation:**
- "I want to be at 0° roll, but I'm at 10° roll"
- Angle PID outputs "-30 deg/s"
- Meaning: "Roll left at 30 deg/s to correct"

### Rate Control (Inner Loop)

```c
void rate_control_update(float desired_roll_rate, ...) {
  rate_output.roll = pid_calculate(&pid_roll_rate, desired_roll_rate,
                                   gyro_roll_rate, RATE_LOOP_DT_SEC);
}
```

**Input**: Desired rate (from angle loop), Actual rate (from gyro)
**Output**: Motor command adjustment

**Physical interpretation:**
- "I want to roll at -30 deg/s, but I'm only rolling at -10 deg/s"
- Rate PID outputs: "Increase aileron authority"

### Why This Architecture Works

```
┌──────────────────────────────────────────────────────────────┐
│                     CASCADED CONTROL                          │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│   RC Stick ──▶ ANGLE PID ──▶ RATE PID ──▶ Motors             │
│   (degrees)    │            │             (PWM)               │
│                │            │                                 │
│                ▼            ▼                                 │
│           Error         Error                                 │
│            ↑              ↑                                   │
│            │              │                                   │
│       Fused Angle      Gyro Rate                              │
│            │              │                                   │
│            └──────┬───────┘                                   │
│                   │                                           │
│               IMU Sensors                                     │
└──────────────────────────────────────────────────────────────┘
```

**Benefits:**
1. **Bandwidth separation**: Outer loop ~10 Hz, inner loop ~50 Hz
2. **Physical meaning**: Outer = position, inner = velocity
3. **Rate loop rejects disturbances** before they affect angle
4. **Tuning order**: Tune inner loop first, then outer loop

---

## 10. Motor Mixing Algorithm

### Quad-X Mixer

```c
int32_t m1 = t - (int32_t)roll_pid + (int32_t)pitch_pid - (int32_t)yaw_pid;
int32_t m2 = t - (int32_t)roll_pid - (int32_t)pitch_pid + (int32_t)yaw_pid;
int32_t m3 = t + (int32_t)roll_pid + (int32_t)pitch_pid + (int32_t)yaw_pid;
int32_t m4 = t + (int32_t)roll_pid - (int32_t)pitch_pid - (int32_t)yaw_pid;
```

### Motor Layout

```
       FRONT
    M4(CCW)  M2(CW)
         \  /
          \/
          /\
         /  \
    M3(CW)  M1(CCW)
        REAR
```

### Derivation of Mixing Equations

**Roll Control** (bank left/right):
- Roll right (+) requires: Left side faster, right side slower
- M3, M4 (left) speed up: +roll_pid
- M1, M2 (right) slow down: -roll_pid

**Pitch Control** (nose up/down):
- Pitch up (+) requires: Rear faster, front slower
- M1, M3 (rear) speed up: +pitch_pid
- M2, M4 (front) slow down: -pitch_pid

**Yaw Control** (rotate CW/CCW):
- Yaw CW (+) requires: More torque from CCW motors
- Physics: Every motor produces thrust AND torque
- CW motor torque points down; CCW motor torque points up
- Net CCW torque rotates aircraft CW (Newton's 3rd law)
- To yaw CW: Speed up CW motors, slow down CCW motors

**Mixing Matrix:**
```
        Throttle  Roll   Pitch   Yaw
M1 =      1       -1      +1     -1
M2 =      1       -1      -1     +1
M3 =      1       +1      +1     +1
M4 =      1       +1      -1     -1
```

### Clamping

```c
static uint16_t clamp_motor(int32_t val) {
  if (val < MIXER_IDLE_THROTTLE)
    return MIXER_IDLE_THROTTLE;
  if (val > MIXER_MAX_THROTTLE)
    return MIXER_MAX_THROTTLE;
  return (uint16_t)val;
}
```

**Why clamp to IDLE, not 0?**
- Motors at 0 stop spinning
- Stopped motor takes time to resume spinning
- Creates asymmetric response (only 3 motors available)
- Minimum idle keeps all 4 motors "in the game"

**Why use `int32_t` for calculation?**
- Mixing can result in negative intermediate values
- `uint16_t` would wrap (65535 instead of -1)
- Compute in signed, clamp, then convert to unsigned

---

## 11. PWM Generation for ESCs

### LEDC Timer Configuration

```c
void pwm_init(void) {
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = LEDC_DUTY_RES,
      .freq_hz = LEDC_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);
}
```

**LEDC (LED Control)**: ESP32's PWM peripheral
- Name derives from LED dimming use case
- Works for any PWM application

**Why LEDC vs MCPWM?**
- MCPWM has motor control features (dead time, fault handling)
- ESCs only need simple PWM
- LEDC is simpler, uses fewer resources

### Pulse Width Calculation

```c
void pwm_set_motor(int motor_index, uint32_t pulse_width_us) {
  uint32_t max_duty = (1 << PWM_RES_BIT) - 1;  // 4095 for 12-bit
  uint32_t duty = (uint32_t)(((uint64_t)pulse_width_us * (uint64_t)max_duty *
                              (uint64_t)PWM_FREQ_HZ) / 1000000ULL);
  ledc_set_duty(LEDC_MODE, motor_channels[motor_index], duty);
  ledc_update_duty(LEDC_MODE, motor_channels[motor_index]);
}
```

**Calculation explained:**

At 250 Hz, period = 4000 µs
12-bit resolution = 4096 duty levels

For 1500 µs pulse:
```
duty = (1500 × 4095 × 250) / 1000000
     = 1,535,625,000 / 1000000
     = 1536 (out of 4095)
     = 37.5% duty cycle
```

Verification: 37.5% of 4000 µs = 1500 µs ✓

**Why `uint64_t` intermediate?**
- `1500 × 4095 × 250 = 1,535,625,000`
- Exceeds 32-bit range (4,294,967,295)
- Use 64-bit for multiplication, result fits in 32-bit

---

## 12. RC Receiver Interface

### PPM Signal Processing

```c
static void IRAM_ATTR rx_isr_handler(void *arg) {
  int64_t now = esp_timer_get_time();
  int64_t dt = now - last_time_us;
  last_time_us = now;

  if (dt > RX_SYNC_MIN_US) {
    current_channel = 0;
    connected = true;
    last_frame_time_us = now;
  }
  else if (dt >= RX_MIN_US && dt <= RX_MAX_US) {
    if (current_channel < RX_CHANNEL_COUNT) {
      rx_channels[current_channel] = (uint16_t)dt;
      current_channel++;
    }
  }
}
```

### PPM Signal Format

```
      ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐         ┌──┐
──────┘  └─┘  └─┘  └─┘  └─┘  └─┘  └─────────┘  └───
      │Ch1│Ch2│Ch3│Ch4│Ch5│Ch6│   SYNC     │Ch1 ...
      
Time between rising edges encodes channel value:
- 1000 µs = minimum (stick low)
- 1500 µs = center
- 2000 µs = maximum (stick high)
- >2100 µs = sync pulse (frame separator)
```

**Sync detection:**
```c
if (dt > RX_SYNC_MIN_US) {  // > 2100 µs
  current_channel = 0;       // Next pulse is channel 0
}
```

**Channel storage:**
```c
rx_channels[current_channel] = (uint16_t)dt;
current_channel++;
```

**Atomic read with interrupt disable:**
```c
uint16_t rx_get_channel(uint8_t channel_index) {
  portDISABLE_INTERRUPTS();
  uint16_t val = rx_channels[channel_index];
  portENABLE_INTERRUPTS();
  return val;
}
```

**Why disable interrupts?**
- ISR might update `rx_channels` while main loop reads
- Could read partially-updated value (torn read)
- Brief interrupt disable ensures atomic read
- Cost: ~1 µs, negligible

### Connection Detection

```c
bool rx_is_connected(void) {
  if (esp_timer_get_time() - last_frame_time_us > 100000) {
    connected = false;
  }
  return connected;
}
```

**100ms timeout:**
- PPM frames arrive every ~20ms
- 5 missed frames = connection lost
- Triggers failsafe in main loop

---

## 13. Safety and Failure Handling

### Crash Detection

```c
// Angle-based crash detection
if (fabs(imu->roll_deg) > sys_cfg.crash_angle_deg ||
    fabs(imu->pitch_deg) > sys_cfg.crash_angle_deg) {
  mixer_arm(false);
  system_armed = false;
  error_state = true;
}

// Rate-based crash detection
const float CRASH_GYRO_RATE_DPS = 500.0f;
if (fabs(imu->gyro_x_dps) > CRASH_GYRO_RATE_DPS || ...) {
  // Same disarm sequence
}
```

**Why two methods?**

**Angle-based:**
- Catches slow tips-over
- Drone on its side = definitely crashed
- Limitation: Angle filter has ~100ms lag

**Rate-based:**
- Catches sudden impacts
- 500 deg/s = something hit the drone
- Faster than angle detection (raw gyro)

**Why not just stop motors?**

```c
mixer_arm(false);      // Stop motors
system_armed = false;  // Clear armed state
error_state = true;    // Flag error (requires reset)
```

- `error_state` prevents immediate re-arming
- Pilot must cycle arm switch to acknowledge
- Prevents motors restarting if still inverted

### Arming Safety

```c
if (rx_ok && arm_switch_high && !error_state) {
  if (!system_armed) {
    uint16_t rx_thr_check = rx_get_channel(2);
    if (rx_thr_check < 1150) {  // Throttle low
      system_armed = true;
      // ... arm sequence ...
    }
  }
}
```

**Three conditions for arming:**
1. RX connected (link is healthy)
2. Arm switch high (intentional pilot action)
3. No error state (system healthy)
4. Throttle low (motors won't immediately spin high)

**Why check throttle?**
- Prevents arming with high throttle → sudden jump
- Standard safety practice in all flight controllers

### Battery Protection

```c
if (system_armed && debug_vbat < sys_cfg.low_bat_threshold) {
  if (debug_vbat < 9500) {  // 9.5V critical
    system_armed = false;
    mixer_arm(false);
  }
}
```

**Why 9.5V cutoff for 3S LiPo?**
- 3S LiPo: 3 cells × 3.0V minimum = 9.0V absolute minimum
- 9.5V provides margin before cell damage
- Below 3.0V/cell causes permanent capacity loss

**Why only critical cutoff, not warning?**
- Warning could be added (LED, beeper)
- Critical cutoff prevents battery destruction

### Emergency Stop

```c
if (gpio_get_level(BUTTON_PIN) == 0) {
  system_armed = false;
  mixer_arm(false);
  error_state = true;
}
```

**Physical button on ESP32:**
- GPIO 0 (boot button) used as kill switch
- Active low (pressed = 0)
- Immediate motor stop, no questions asked
- Sets error state (requires arm switch cycle)

---

## 14. Memory and Real-Time Considerations

### Stack Usage

Each FreeRTOS task has its own stack:

```c
xTaskCreatePinnedToCore(blackbox_task, "blackbox", 2048, ...);
```

**Stack size choices:**
- Main task: FreeRTOS default (~4KB)
- Blackbox task: 2KB (simple, no recursion)
- Control loop: Runs in esp_timer task (~4KB)

**Stack overflow risk:**
- Large local arrays
- Deep recursion
- FreeRTOS `uxTaskGetStackHighWaterMark()` to measure usage

### IRAM Placement

```c
static void IRAM_ATTR rx_isr_handler(void *arg) { ... }
static void IRAM_ATTR control_loop_callback(void *arg) { ... }
```

**Why IRAM for control loop?**
- ESP32 executes code from external SPI flash
- Flash access has cache; cache misses cause delays
- Worst case: 30+ µs stall during flash operation
- IRAM: 200KB internal, always available
- Timing-critical code must be in IRAM

### Memory Layout

```
┌─────────────────────────────────────────┐
│              IRAM (200KB)               │
│  - ISR handlers                         │
│  - Time-critical functions              │
│  - FreeRTOS kernel                      │
├─────────────────────────────────────────┤
│              DRAM (320KB)               │
│  - Global variables                     │
│  - Heap (malloc)                        │
│  - Task stacks                          │
│  - Blackbox buffer (96KB)               │
├─────────────────────────────────────────┤
│           External Flash (4MB)          │
│  - Program code                         │
│  - NVS partition                        │
│  - Constants (strings)                  │
└─────────────────────────────────────────┘
```

### Real-Time Guarantees

**Control loop deadline: 4000 µs**

Budget breakdown:
- IMU I2C read: ~500 µs
- Sensor processing: ~50 µs
- PID calculations: ~20 µs
- Mixer + PWM: ~30 µs
- Total: ~600 µs
- Margin: 3400 µs (85% slack)

**What could violate deadline?**
1. Flash cache miss (prevented by IRAM)
2. Higher priority interrupt (none configured)
3. I2C bus stall (rare, handled by timeout)

### Volatile and Atomicity

```c
static volatile bool control_loop_flag = false;
```

**Why volatile?**
- Modified by timer callback
- Read by main loop
- Without volatile: Compiler may cache in register
- Loop might never see the change

**What about atomicity?**
- Boolean: Single byte, inherently atomic on ESP32
- If larger type: Need mutex or interrupt disable

```c
portDISABLE_INTERRUPTS();
uint16_t val = rx_channels[channel_index];
portENABLE_INTERRUPTS();
```

---

## Summary: How an Engineer Designs This System

### 1. Start with Requirements
- 250 Hz control loop (sufficient for quadcopter dynamics)
- Cascaded PID (industry standard for flight control)
- Multiple safety systems (crash detection, battery, link loss)

### 2. Choose Architecture
- Hardware timer for deterministic loop timing
- RTOS for non-critical tasks (WiFi, blackbox)
- Interrupt-driven RX for minimal latency

### 3. Implement Bottom-Up
1. HAL drivers (PWM, I2C, ADC)
2. Sensor modules (IMU with filtering)
3. Control algorithms (PID)
4. System integration (main loop, safety)

### 4. Validate Each Layer
- PWM: Oscilloscope to verify pulse widths
- IMU: Static tests for bias, rate tests for calibration
- PID: Bench test with restricted props
- Full system: Tethered hover tests

### 5. Tune Iteratively
- Start with very low gains
- Increase P until oscillation, back off 30%
- Add I for steady-state
- Add D for damping
- Use blackbox for data-driven tuning

---

*This document provides the theoretical foundation and practical knowledge needed to understand, maintain, or extend this flight controller firmware.*
