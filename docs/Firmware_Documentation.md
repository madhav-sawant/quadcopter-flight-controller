# Main Firmware Documentation

**File**: `src/main.c`
**Purpose**: The entry point and "Conductor" of the system. It initializes all subsystems, manages the multi-core task scheduling, and executes the critical Flight Control Loop.

---

## System Architecture

The ESP32 has 2 CPU Cores. We leverage this for high performance:

### **Core 1 (Application Core)** - **Flight Critical**
- **Task**: `control_loop_task`
- **Priority**: 24 (Real-time).
- **Frequency**: 250 Hz (4ms period).
- **Jobs**:
  - Read IMU.
  - Calculate PID.
  - Mix Motors.
  - Send PWM.
- **Why?** Core 1 is isolated from WiFi interrupts. This ensures the drone flies smoothly without "jitters" caused by network traffic.

### **Core 0 (Pro Core)** - **Support**
- **Tasks**: WiFi, Webserver, Blackbox Logger.
- **Priority**: Lower.
- **Jobs**:
  - Maintain WiFi connection.
  - Serve Web Dashboard.
  - Check Battery (Slow).
  - Write Logs to RAM.

---

## The Flight Control Loop (Step-by-Step)

The `control_loop_task` function runs an infinite loop aimed at exactly 250Hz.

1.  **Read Sensors (`imu_read`)**:
    - Fetches latest Gyro/Accel data.
    - Fuses them into Tilt Angles (Roll/Pitch).
2.  **Safety Checks**:
    - **Crash Detection**: If Tilt > 60° OR Gyro Rate > 500dps -> **DISARM**.
    - **Failsafe**: If RX signal lost -> **DISARM**.
3.  **Read Pilot Input (`rx_get_channel`)**:
    - Converts Stick positions (1000-2000us) into Desired Rates (e.g., +/- 100 deg/s).
    - Applies Deadband to ignore small jitters.
4.  **PID Control (`rate_control_update`)**:
    - Calculates the difference between Pilot Request and Gyro Reality.
    - Computes corrections used to drive motors.
5.  **Mixer**:
    - Combines Throttle + PID corrections.
    - Sets physical motor speeds.
6.  **Logging**:
    - Sends data to Blackbox Queue (every 3rd loop, ~83Hz).

---

## Code Explanation (Line-by-Line)

### `src/main.c`

```c
105: static void control_loop_task(void *arg) {
...
111:   int64_t next_cycle_time = esp_timer_get_time();
```
Establish precise timing baseline using microsecond timer.

```c
117:   esp_task_wdt_add(NULL);
```
**Watchdog**: Since this task loops forever at high priority, we must manually feed the Watchdog. If the code freezes, the hardware will reboot the drone to save it.

```c
130:     const float CRASH_ANGLE_DEG = 60.0f;
131:     if (fabs(imu->roll_deg) > CRASH_ANGLE_DEG ...
```
**Crash Safety**: Prevents the drone from destroying itself if it flips over. Motors cut instantly.

```c
142:     const float CRASH_GYRO_RATE_DPS = 500.0f;
```
**Vibration/Impact Safety**: If the drone hits the ground or a wall, it spins violently (>500dps). We detect this spike and cut motors to prevent prop damage.

```c
190:     if (throttle < 1000) throttle = 1000;
192:     if (throttle > TUNING_THROTTLE_LIMIT) ...
```
**Throttle Clamping**:
- `TUNING_THROTTLE_LIMIT` (e.g., 1400) prevents the drone from blindly flying into the stratosphere during initial testing.
- Should be raised to 2000 for full flight capabilities.

```c
282:     // Wait for next cycle - precise timing
286:     while (esp_timer_get_time() < next_cycle_time) {
288:       taskYIELD();
```
**Precision Timing**: We don't use `vTaskDelay(4ms)` because it only has 1ms resolution (could be 3ms or 5ms). Instead, we spin-wait (busy loop) for the final microseconds to hit exactly 4000us. `taskYIELD` lets lower-priority tasks (like Logging) run in the gaps.

### `void app_main(void)`

```c
302:   gpio_reset_pin(PWM_MOTOR_1_GPIO);
...
320:     pwm_set_motor(i, 1000);
```
**ESC Safety Boot**: Immediately pulls motor pins LOW and sends 1000us (Stop). This prevents ESCs from entering calibration mode or spinning up randomly during the few seconds of ESP32 boot time.

```c
325:   esp_err_t ret = nvs_flash_init();
```
Initializes storage for PIDs and Calibration.

```c
458:   xTaskCreatePinnedToCore(control_loop_task, ..., 1);
```
Launches the critical flight code on **Core 1**.

```c
470:   while (1) {
```
The "Main Loop" (running on Core 0 effectively, or simply main thread logic):
- Handles Arming/Disarming logic (running at ~100Hz).
- Checks Battery (Low speed).
- Blinks LED.
- It does **NOT** fly the drone. The Task does that.
# Rate Control Library Documentation

**Files**: `lib/rate_control/rate_control.c`, `lib/rate_control/rate_control.h`
**Purpose**: The central "Flight Controller" logic. It bridges the Gap between Gyroscope data and Motor Mixing. It manages 3 separate PID controllers (Roll, Pitch, Yaw) to ensure the drone rotates at the speed requested by the pilot (`deg/s`).

---

## Architecture

1.  **Input**:
    *   **Desired Rates**: What the pilot wants (stick input converted to deg/s).
    *   **Actual Rates**: Measurement from the Gyroscope (deg/s).
2.  **Processing**:
    *   `Error = Desired - Actual`
    *   `PID Output = P(Error) + I(Error) + D(Measurement)`
3.  **Output**:
    *   Passes the 3 PID results to the Mixer.

---

## Function Reference

### `void rate_control_init(void)`
**Description**: Initializes the 3 PID controllers with gains loaded from `sys_cfg` (Config).
- **Details**: Also resets the `rate_output` struct.

### `void rate_control_update(...)`
**Description**: The main loop function.
- **Parameters**: 3 Desired Rates (Setpoints), 3 Gyro Rates (Measurements).
- **Process**:
  - specific `pid_calculate` calls for Roll, Pitch, and Yaw.
  - Constant Loop Time (`RATE_LOOP_DT_SEC` = 0.004s = 250Hz).
- **Output**: Updates the internal `rate_output` structure.

### `const rate_output_t *rate_control_get_output(void)`
**Description**: Accessor for the calculated PID values. Used by `main.c` to send data to the Mixer.

### `void rate_control_freeze_integral(bool freeze)`
**Description**: Safety feature.
- **Usage**: When throttle is low (Ground Idle), we call this with `true`.
- **Effect**: Pauses I-term accumulation. This ensures that if the drone isn't perfectly level on the ground, the "Error" doesn't build up over time and cause the drone to flip the moment you touch the throttle.

---

## Code Explanation (Line-by-Line)

### `lib/rate_control/rate_control.c`

```c
7: #define RATE_LOOP_DT_SEC 0.004f
```
Hardcoded time delta (4ms). This relies on the Main Loop running accurately at 250Hz.

```c
9: void rate_control_init(void) {
10:   pid_init(&pid_roll_rate, sys_cfg.roll_kp, ...
```
Links the generic PID library to our specific Configuration values. This separation allows `pid.c` to be pure logic and `rate_control.c` to handle the specific application.

```c
22: void rate_control_update(...) {
26:   rate_output.roll = pid_calculate(&pid_roll_rate, desired_roll_rate, ...
```
Runs the math. Note that we pass `RATE_LOOP_DT_SEC` every time.
# Mixer Library Documentation

**Files**: `lib/mixer/mixer.c`, `lib/mixer/mixer.h`
**Purpose**: Translates abstract Roll/Pitch/Yaw PID commands into concrete Motor PWM signals. It implements the "mixing table" for a Quadcopter in X-Configuration.

---

## Configuration
- **Frame Type**: Quad-X (Propellers at 45° to the center).
- **Idle Throttle**: 1100μs. Minimum speed when armed.
- **Max Throttle**: 1500μs (Currently limited for safety, adjustable to 2000).

## Function Reference

### `void mixer_update(uint16_t throttle, float roll, float pitch, float yaw)`
**Description**: The core mixing function.
- **Inputs**:
  - `throttle`: Base throttle from RC (1000-2000).
  - `roll, pitch, yaw`: PID outputs (Differential corrections).
- **Logic**:
  1. **Idle Check**: If throttle < Idle Threshold (1150), motors spin at low IDLE speed and PIDs are ignored. This prevents flipping on the ground.
  2. **Mixing**: Combines inputs according to motor position.
  3. **Clamping**: Ensures output never exceeds [IDLE, MAX] range.
  4. **Output**: Calls `pwm_set_motor` to drive the hardware.

### `void mixer_arm(bool armed)`
**Description**: Arms or Disarms the mixer.
- **Disarmed**: All motors forced to STOP (1000μs).
- **Armed**: Motors allowed to spin.

---

## Code Explanation (Line-by-Line)

### `lib/mixer/mixer.c`

```c
7: static uint16_t motor_cmds[4] = {1000, 1000, 1000, 1000};
```
Stores current motor command for debugging/blackbox logging.

```c
38:   if (t < MIXER_IDLE_THROTTLE + 50) {
```
**Ground Idle Protection**: If the throttle stick is low, we don't apply PID corrections. This is critical because on the ground, the drone can't move. If the PID loop tries to correct an angle, the error persists (integration windup), causing the motors to spin up aggressively and flip the drone.

```c
66:   // HARDWARE ADAPTATION:
67:   // Pitch sign is INVERTED relative to standard Quad-X
```
**Custom Hardware Fix**:
Standard Quad-X pitch logic is: Nose Up -> Rear motors speed up, Front motors slow down.
This specific build required inverting this logic due to ESC signal wiring relative to the physical front.

```c
69:   int32_t m1 = t - roll + pitch - yaw; // Rear Right
70:   int32_t m2 = t - roll - pitch + yaw; // Front Right
71:   int32_t m3 = t + roll + pitch + yaw; // Rear Left
72:   int32_t m4 = t + roll - pitch - yaw; // Front Left
```
**The Mixing Table**:
- **Roll**: Positive Roll (Right wing down) -> Left motors (M3, M4) increase, Right motors (M1, M2) decrease.
- **Pitch**: Positive Pitch (Nose Up) -> Rear motors (M1, M3) increase. (Note: Signs here reflect the *inverted* logic mentioned above).
- **Yaw**: Positive Yaw (CCW rotation) -> CW spinning motors (M2, M3) increase torque to rotate body CCW.

```c
79:   motor_cmds[0] = clamp_motor(m1);
```
Result is clamped to safe limits (1100-1500 currently).
# PID Library Documentation

**Files**: `lib/pid/pid.c`, `lib/pid/pid.h`
**Purpose**: A generic PID (Proportional-Integral-Derivative) controller implementation tailored for flight control. Includes features like Derivative-on-Measurement (for smooth setpoint changes), D-Term Low Pass Filtering (noise reduction), and Integral Windup Protection.

---

## Math & Theory

### The Control Equation
`Output = P_term + I_term + D_term`
1. **P (Proportional)**: `Kp * Error`. Immediate correction based on current error.
2. **I (Integral)**: `Ki * Sum(Error * dt)`. Accumulates error over time to correct steady-state drift (like being nose-heavy).
3. **D (Derivative)**: `Kd * -d(Measurement)/dt`. Resists change. Dampens oscillations and overshoots.

### Advanced Features
- **Derivative-on-Measurement**: Standard PID calculates D on Error. We calculate D on *Measurement* (Gyro Rate). This prevents "Derivative Kick" - a sudden spike in output when you move the stick (Change Setpoint).
- **Integral Freeze**: Ability to pause I-term accumulation (e.g., when throttle is low). Prevents "I-term Windup" on the ground.
- **D-Term LPF**: A Low-Pass Filter on the D-term. Raw derivative amplifies noise (vibrations). Filtering it is mandatory for quadcopters.

---

## Function Reference

### `void pid_init(...)`
**Description**: Resets the PID state (clears Integral sum, filters, etc.) and sets gains.
- **Usage**: Called when Arming to ensure a clean start.

### `float pid_calculate(pid_t *pid, float setpoint, float measurement, float dt)`
**Description**: The main update step.
- **Returns**: Control Output (e.g., motor mix value).
- **Details**:
  - Calculates P, I, D terms locally.
  - Updates history (previous measurement).
  - Clamps output to `output_limit`.

---

## Code Explanation (Line-by-Line)

### `lib/pid/pid.c`

```c
7: #define D_TERM_LPF_ALPHA 0.15f
```
**Filter Strength**: `0.15` at 250Hz provides a cutoff around ~30Hz. This heavily filters high-frequency motor noise from the sensitive D-term.

```c
24:   float error = setpoint - measurement;
27:   float p_out = pid->kp * error;
```
Standard P-term calculation.

```c
30:   if (!pid->integral_frozen) {
31:     pid->integral += error * dt_sec;
```
**Integral Accumulation**: Only runs if `!frozen`. `dt_sec` makes the integral time-independent (works same at 250Hz or 1kHz).

```c
32:     if (pid->integral > pid->integral_limit) ...
```
**Anti-Windup**: Hard clamps the I-term sum. Even if error persists for minutes, I-term won't grow to infinity.

```c
43:   float raw_derivative = -(measurement - pid->prev_measurement) / dt_sec;
```
**D on Measurement**: Note we use `-(measurement - prev)`.
We do NOT uses `error - prev_error`.
Why negative? If Measurement increases (Drone pitches up), we want to oppose it (Pitch down).

```c
47:   pid->filtered_derivative =
48:       D_TERM_LPF_ALPHA * raw_derivative +
49:       (1.0f - D_TERM_LPF_ALPHA) * pid->filtered_derivative;
```
**IIR Filter**: Smooths the noisy `raw_derivative`.
`Smoothed = 0.15 * New + 0.85 * Old`.
# IMU Library Documentation

**Files**: `lib/imu/imu.c`, `lib/imu/imu.h`
**Purpose**: Driver for the MPU6050 Gyro/Accelerometer. It handles I2C communication, sensor configuration (DLPF, Range), sensor calibration (Bias/Scale), and sensor fusion (Complementary Filter) to estimate the drone's attitude (Angles).

---

## Constants & Configuration

### I2C Config (ESP32)
- `SDA`: GPIO 21
- `SCL`: GPIO 22
- `Frequency`: 400kHz (Fast Mode)

### MPU6050 Settings
- `DLPF`: 98Hz (Delay ~4ms). Crucial trade-off: Lower Hz removes vibration but adds lag. 98Hz is fast enough for flight.
- `Gyro Range`: +/- 2000 dps (Maximum range for safety).
- `Accel Range`: +/- 8g (Handles flight vibrations).

### Filters
- **Software LPF**: IIR filter on the ESP32 side to smooth out motor noise that passes through the hardware DLPF.
  - `GYRO_LPF_ALPHA 0.40`: Strong filtering. 40% new data, 60% old.
  - `ACCEL_LPF_ALPHA 0.10`: Heavy accel filtering (Accels are very noisy on quads).
- **Complementary Filter**: `0.96` weight on Gyro, `0.04` on Accel. High gyro trust prevents drift during maneuvers.

---

## Function Reference

### `esp_err_t imu_init(void)`
**Description**: Initializes I2C bus and configures MPU6050 registers.
- **Checks**: Verifies `WHO_AM_I` register to ensure sensor is connected.
- **Config**: Sets Power Management (PLL Clock), DLPF, Gyro/Accel ranges.

### `void imu_read(float dt_sec)`
**Description**: Main sensor read loop. Called every cycle (250Hz).
- **Process**:
  1. Burst reads 14 bytes from MPU6050 (Accel XYZ, Temp, Gyro XYZ).
  2. Scales raw integer values to floats (dps / g).
  3. Applies Software Low-Pass Filter.
  4. Runs **Complementary Filter** to update `pitch_deg` and `roll_deg`.
  5. Applies **Calibration Offsets** (Gyro Bias subtracted, Accel Offset subtracted).

### `void imu_calibrate_gyro(void)` / `imu_calibrate_accel(void)`
**Description**: Calculates sensor offsets by averaging 1000 samples while still.
- **Gyro**: Calculates bias (drift when still) -> Saved to `gyro_bias_x/y/z`.
- **Accel**: Calculates angle offsets -> Saved to `accel_offset_roll/pitch`.
- **Accel Scale**: Calculates `Z` scale factor to ensure 1G reading when level.

### `imu_calibration_save_to_nvs(void)` / `load...`
**Description**: Saves/Loads calibration data to persistent flash memory (NVS). This ensures you don't need to calibrate on every boot.

---

## Code Explanation (Line-by-Line)

### `lib/imu/imu.c`

```c
10-22: Registers
```
Defines MPU6050 register addresses. `REG_ACCEL_XOUT_H` (0x3B) is the start of the burst-read block.

```c
32: #define COMPLEMENTARY_ALPHA 0.96f
```
The "Trust Factor". 96% of the new angle comes from integrating the Gyro (Rates). 4% comes from the Accelerometer (Gravity vector). This filters out short-term accel noise while preventing long-term gyro drift.

```c
75: static esp_err_t write_register(...)
88: static esp_err_t read_registers(...)
```
Low-level I2C transaction helpers using ESP-IDF `i2c_master_...` commands.

```c
106: esp_err_t imu_init(void) {
...
132:   write_register(REG_PWR_MGMT_1, BIT_CLKSEL_PLL_X);
```
Wakes up the chip (it sleeps by default) and sets the clock source to the X-Gyro PLL (more stable than internal oscillator).

```c
224: void imu_read(float dt_sec) {
...
227:   uint8_t buffer[14];
228:   read_registers(REG_ACCEL_XOUT_H, buffer, 14);
```
Efficient "Burst Read". Reads all 6 axes + temp in one I2C transaction. This ensures all samples are from the exact same time instant.

```c
245:   float accel_raw_x = (ax_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
```
Converts raw 16-bit int to G-force. `32768 / 8g = 4096 LSB/g`.
Multiplies by the calibrated `accel_scale_factor` (usually ~1.0) to correct for sensor variance.

```c
263:     // IIR low-pass filter
264:     accel_filtered_x = 0.10f * accel_raw_x + 0.90f * accel_filtered_x;
```
Software noise filter. Essential for eliminating frame vibrations (motors/props).

```c
291:   float accel_pitch = atan2f(...) * RAD_TO_DEG;
```
Calculates "Absolute Angle" from gravity vector.
Pitch = atan2(X, sqrt(Y² + Z²)).
Roll = atan2(Y, Z).

```c
324:     imu_state.pitch_deg =
325:         0.96f * (imu_state.pitch_deg + imu_state.gyro_y_dps * dt_sec) +
327:         0.04f * (accel_pitch + PITCH_TRIM_DEG);
```
The Core Complementary Filter:
`New_Angle = 0.96 * (Old_Angle + Gyro_Rate * dt) + 0.04 * (Accel_Angle)`.
Integrates gyro rate to update angle, then "nudges" it slightly towards the accelerometer reading to prevent drift.
# PWM Library Documentation

**Files**: `lib/pwm/pwm.c`, `lib/pwm/pwm.h`
**Purpose**: Generates Precise Pulse Width Modulation signals to drive the ESCs (Electronic Speed Controllers). Uses the ESP32's `LEDC` (LED Control) hardware peripheral, which allows hardware-timed pulses without consuming CPU.

---

## Configuration

- **Frequency**: 250 Hz.
  - Matches the main control loop rate.
  - Standard for older ESCs (OneShot125/42 would differ, but 250Hz PWM is a safe universal standard).
- **Resolution**: 12-bit (0-4095 steps).
  - At 250Hz, the period is 4000μs.
  - 12-bit resolution gives ~1μs precision, which is excellent for throttle control.

### Hardware Mapping
| Motor | GPIO |
|-------|------|
| M1    | 13   |
| M2    | 25   |
| M3    | 33   |
| M4    | 27   |

---

## Function Reference

### `void pwm_init(void)`
**Description**: Configures the LEDC Timer and Channels.
- **Details**:
  - Sets up `LEDC_TIMER_0` for 250Hz.
  - Configures 4 independent channels attached to the Motor GPIOs.
  - Initializes all motors to 1000μs (Stop).

### `void pwm_set_motor(int motor_index, uint32_t pulse_width_us)`
**Description**: Updates the pulse width for a specific motor.
- **Parameters**:
  - `motor_index`: 0-3.
  - `pulse_width_us`: Desired pulse length in microseconds (1000-2000).
- **Details**:
  - Clamps input to safe [1000, 2000] range.
  - Converts microseconds to "Duty Cycle" ticks based on the 12-bit resolution.
  - `ledc_update_duty`: Commits the new value to hardware.

---

## Code Explanation (Line-by-Line)

### `lib/pwm/pwm.c`

```c
10: static const int motor_gpios[4] = {13, 25, 33, 27};
```
Defines the physical pin mapping.

```c
16:   ledc_timer_config_t ledc_timer = {
19:       .freq_hz = 250,
20:       .clk_cfg = LEDC_AUTO_CLK};
```
Configures the hardware timer source. 250Hz means one cycle every 4ms.

```c
45:   uint32_t duty = (uint32_t)(((uint64_t)pulse_width_us * (uint64_t)max_duty *
46:                               (uint64_t)PWM_FREQ_HZ) /
47:                              1000000ULL);
```
**Duty Cycle Calculation**:
Why this formula?
`Duty_Ticks = (Time_us / Period_us) * Max_Ticks`
`Period_us = 1,000,000 / Frequency`
So: `Duty = (Time_us * Frequency * Max_Ticks) / 1,000,000`.
We use `uint64_t` to prevent overflow during the multiplication.
# RX Library Documentation

**Files**: `lib/rx/rx.c`, `lib/rx/rx.h`
**Purpose**: Handles Radio Receiver (RC) input decoding.
**NATIVE PROTOCOL**: **PPM (Pulse Position Modulation)** over a single wire.
*Note: Although "IBUS" is mentioned in some conversation contexts, the code here explicitly implements a PPM decoder using GPIO Interrupts.*

---

## Technical Details (PPM)
- **Pin**: GPIO 26 (`RX_PIN`).
- **Method**: GPIO Interrupt on Positive Edge (`GPIO_INTR_POSEDGE`).
- **Timing**:
  - **Frame**: A sequence of 6-8 channel pulses separated by short gaps, followed by a long "Sync" gap (> 2100μs).
  - **Channel Value**: Time difference between two consecutive Rising Edges.
    - Low Stick: ~1000μs
    - Center: ~1500μs
    - High Stick: ~2000μs

---

## Function Reference

### `void rx_init(void)`
**Description**: Sets up the GPIO interrupt service.
- **Details**:
  - Configures GPIO 26 as Input.
  - Installs ISR (Interrupt Service Routine).
  - Initializes all channels to safe centers (1500), except Throttle (1000).

### `bool rx_is_connected(void)`
**Description**: Safety check.
- **Returns**: `true` if a valid PPM frame was received in the last 100ms.
- **Usage**: Disarm immediately if this returns `false`.

### `uint16_t rx_get_channel(uint8_t channel_index)`
**Description**: Thread-safe channel read.
- **index**: 0 (Roll), 1 (Pitch), 2 (Throttle), 3 (Yaw), 4 (Aux1), 5 (Aux2).
- **Thread Safety**: Uses `portDISABLE_INTERRUPTS()` to ensure we don't read a 16-bit integer while it's being written by the ISR (which could result in torn reads).

---

## Code Explanation (Line-by-Line)

### `lib/rx/rx.c`

```c
15: static void IRAM_ATTR rx_isr_handler(void *arg) {
```
**The Interrupt Handler**:
- `IRAM_ATTR`: forces this function to stay in RAM (not Flash) for speed.
- Runs every time the voltage on GPIO 26 goes HIGH.

```c
21:   if (dt > RX_SYNC_MIN_US) {
22:     current_channel = 0;
23:     connected = true;
```
**Sync Detection**: If the time since the last edge is huge (> 2.1ms), it means the previous frame ended. Reset the channel counter to 0 to start reading the next frame.

```c
27:   else if (dt >= RX_MIN_US && dt <= RX_MAX_US) {
28:     if (current_channel < RX_CHANNEL_COUNT) {
29:       rx_channels[current_channel] = (uint16_t)dt;
30:       current_channel++;
```
**Channel Reading**:
- If time `dt` is between 900-2100μs, it's a valid channel value.
- Store it in `rx_channels[0]` then increment to `rx_channels[1]`, etc.

```c
36: void rx_init(void) {
...
41:   rx_channels[2] = 1000;
```
**Failsafe Initialization**: Before any radio signal is heard, we force Throttle to 0. This prevents the motors from blasting off if the FC boots before the Receiver.
# ADC Library Documentation

**Files**: `lib/adc/adc.c`, `lib/adc/adc.h`
**Purpose**: Manages the ESP32's Analog-to-Digital Converter (ADC) to read battery voltage. It uses the `adc_oneshot` driver for single-shot readings and implements a simple digital low-pass filter to smooth out noise.

---

## Function Reference

### `void adc_init(void)`
**Description**: Initializes the ADC unit and configures the specific channel (GPIO 35) for battery reading.
- **Parameters**: None.
- **Returns**: None.
- **Details**: 
  - Configures `ADC_UNIT_1`.
  - Configures `ADC_CHANNEL_7` (GPIO 35).
  - Sets bitwidth to 12-bit (0-4095 range).
  - Sets attenuation to `ADC_ATTEN_DB_12` (allows measuring up to ~3.3V, though reliable range is slightly less).

### `uint16_t adc_read_battery_voltg(void)`
**Description**: Reads the battery voltage, applies filtering, and converts the raw reading to millivolts.
- **Parameters**: None.
- **Returns**: Battery voltage in millivolts (e.g., 11200 for 11.2V). Returns 0 if voltage is below offset threshold (wire disconnected).
- **Details**:
  - Calls `adc_read_raw()` to get a filtered raw value.
  - Converts raw value to voltage using a linear mapping (0-4095 -> 0-3300mV).
  - Applies the voltage divider formula: `Voltage = ADC_Voltage * SCALE / 10000`.
  - Subtracts an offset (`VOLTAGE_OFFSET_MV`) to correct for diode drops or other hardware offsets.

### Internal Functions (Static)

#### `static uint16_t adc_read_raw(void)`
**Description**: Reads the raw ADC value with oversampling and IIR filtering.
- **Details**:
  - Takes 16 samples and averages them to tackle noise.
  - Applies an Infinite Impulse Response (IIR) filter: `New = 0.25 * Sample + 0.75 * Old`.
  - This makes the reading smooth but responsive enough for battery monitoring.

#### `static uint16_t adc_read_voltage(uint16_t raw)`
**Description**: Converts raw 12-bit ADC value to millivolts (board level, before divider).
- **Formula**: `mV = (raw * 3300) / 4095`.

---

## Code Explanation (Line-by-Line)

### `lib/adc/adc.h`

```c
1: #ifndef ADC_H
2: #define ADC_H
```
Standard include guard to prevent multiple inclusion of this header file.

```c
4: #include "esp_adc/adc_oneshot.h"
5: #include <stdint.h>
```
Includes the ESP-IDF ADC OneShot driver and standard integer types.

```c
7: // ADC1 Channel 7 (GPIO 35)
8: #define ADC_UNIT ADC_UNIT_1
9: #define ADC_CHANNEL ADC_CHANNEL_7
```
Defines the hardware resources. We use ADC Unit 1 and Channel 7, which corresponds to GPIO 35 on the ESP32 DevKit.

```c
10: #define ADC_ATTEN ADC_ATTEN_DB_12
11: #define ADC_WIDTH ADC_BITWIDTH_12
```
Configures ADC range and precision:
- `ATTEN_DB_12`: Max input voltage ~3.3V.
- `BITWIDTH_12`: Resolution 0 to 4095.

```c
14: #define VOLTAGE_SCALE_MV 41624
15: #define VOLTAGE_SCALE_DIV 10000
16: #define VOLTAGE_OFFSET_MV 180
```
Calibration constants for the voltage divider:
- `SCALE`: Represents the divider ratio (R1+R2)/R2. Here `4.1624`.
- `OFFSET`: Subtracts 180mV to account for persistent offset errors.

### `lib/adc/adc.c`

```c
6: void adc_init(void) {
7:   // 1. Init ADC Unit
8:   adc_oneshot_unit_init_cfg_t init_config = {
9:       .unit_id = ADC_UNIT,
10:  };
11:  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
```
Initialized the ADC hardware unit handle. `ESP_ERROR_CHECK` halts the program if initialization fails (critical error).

```c
13:  // 2. Config Channel
14:  adc_oneshot_chan_cfg_t config = {
15:      .bitwidth = ADC_WIDTH,
16:      .atten = ADC_ATTEN,
17:  };
18:  ESP_ERROR_CHECK(
19:      adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));
```
Configures the specific channel (GPIO 35) with the width and attenuation settings defined in the header.

```c
23: static uint16_t adc_filtered_raw = 0;
```
Static variable to hold the state of the IIR filter between function calls.

```c
25: static uint16_t adc_read_raw(void) {
...
29:   // Average 16 samples
30:   for (int i = 0; i < 16; i++) {
31:     ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw_val));
32:     sum += raw_val;
33:   }
35:   uint16_t current = (uint16_t)(sum / 16);
```
Oversampling loop: Reads the ADC 16 times and calculates the average. This significantly reduces random electrical noise.

```c
39:   if (adc_filtered_raw == 0) {
40:     adc_filtered_raw = current; // Initialize on first read
41:   } else {
42:     adc_filtered_raw =
43:         (current >> 2) + (adc_filtered_raw - (adc_filtered_raw >> 2));
44:   }
```
Software Filter (IIR):
- Logic: `Filtered = (New_Sample / 4) + (Old_Filtered * 3/4)`
- Implementation: Bitwise shifts (`>> 2` is divide by 4) are used for speed.
- Helps smooth out voltage fluctuations without requiring heavy CPU math.

```c
58: uint16_t adc_read_battery_voltg(void) {
59:   uint16_t raw = adc_read_raw();
60:   uint16_t adc_mv = adc_read_voltage(raw);
```
Gets the filtered raw value and converts it to the voltage seen at the ESP32 pin (0-3.3V range).

```c
63:   uint32_t scaled_mv =
64:       ((uint16_t)adc_mv * VOLTAGE_SCALE_MV) / VOLTAGE_SCALE_DIV;
```
Calculates actual battery voltage using the divider ratio.
`Battery = Pin_Voltage * 4.1624`.

```c
66:   if (scaled_mv < VOLTAGE_OFFSET_MV) {
67:     return 0;
68:   }
70:   uint32_t battery_mv = scaled_mv - VOLTAGE_OFFSET_MV;
```
Safety check: If reading is extremely low, assume 0 (disconnected). Otherwise, subtract the calibration offset.
# Webserver Library Documentation

**Files**: `lib/webserver/webserver.c`, `lib/webserver/webserver.h`
**Purpose**: Provides a WiFi-based user interface for the drone. Allows PID tuning, Sensor Calibration, Live Data monitoring, and Blackbox Log downloading without connecting a USB cable (Safety!).
**Network**: Creates an Access Point named "QuadPID" (Pass: "12345678").

---

## Function Reference

### `void webserver_init(void)`
**Description**: Sets up the WiFi Hardware and HTTP Server.
- **Details**:
  - Initializes ESP-NETIF and Event Loop.
  - Configures WiFi in **Access Point (SoftAP)** mode.
  - Starts the HTTP server on port 80.
  - Registers URL handlers (`/`, `/s`, `/r`, `/live`, etc.).

### `void webserver_set_rate_targets(...)`
**Description**: Updates the "Desired Rates" that are displayed on the live dashboard.
- **Usage**: Called from the main loop in `main.c` so the user can verify if their stick inputs match what the receiver sees.

---

## URL Handlers

### `GET /` (Root)
**Description**: Serves the main Dashboard HTML.
- **Features**:
  - Displays Battery Voltage and System Status (Armed/Disarmed).
  - Javascript polling fetches `/live` every 200ms to update values.
  - Forms for editing P, I, D gains (POST to `/s`).
  - Buttons for Reset (`/r`), Blackbox (`/blackbox`), and Calibration (`/calibrate`).

### `GET /live` (JSON)
**Description**: Returns real-time flight data.
- **Format**: JSON `{tr:0.0, tp:0.0, rr:1.5, rp:-0.5, armed:true}`.
- **Efficiency**: Allows the UI to update smoothly without reloading the heavy HTML page.

### `GET /blackbox` (CSV)
**Description**: Streaming download of the Flight Log.
- **Details**:
  - Sets MIME type to `text/csv`.
  - Iterates through the RAM ring buffer.
  - Formats each entry as a CSV line and streams it via `httpd_resp_sendstr_chunk` to manage memory usage.

### `POST /calibrate`
**Description**: Triggers IMU recalibration.
- **Safety**: Only allowed when **Disarmed**.
- **Process**: Runs Gyro & Accel calibration (~2 seconds), saves to NVS, then redirects back to home.

---

## Code Explanation (Line-by-Line)

### `lib/webserver/webserver.c`

```c
28: static const char *HTML_PAGE = ...
```
Embedded HTML/CSS/JS string.
- we use `fetch('/live')` in JS to update values dynamically.
- CSS makes it look decent on mobile phones.

```c
120: static esp_err_t save_handler(httpd_req_t *req) {
122:   if (system_armed) {
123:     return ESP_OK; // Reject changes if flying!
124:   }
```
**Safety Interlock**: PID tuning is dangerous if changed mid-flight (could cause instability). We block writes if `system_armed` is true.

```c
135:   sys_cfg.roll_kp = parse_float(buf, "rp", sys_cfg.roll_kp);
```
Parses form data. `rp` = Rate Roll P, `ri` = Rate Roll I, etc.
Updates the global `sys_cfg` struct.

```c
145:   config_save_to_nvs();
146:   rate_control_init();
```
Commit to flash immediately and Reload PIDs. The `rate_control_init` call is crucial—it updates the active PID controllers with the new values.

```c
173: static esp_err_t blackbox_handler(httpd_req_t *req) {
```
**Chunked Transfer**: The log file can be >100KB. We can't malloc a buffer that big. We build the file line-by-line and send it in chunks. This keeps RAM usage extremely low (only ~512 bytes buffer).
# Blackbox Library Documentation

**Files**: `lib/blackbox/blackbox.c`, `lib/blackbox/blackbox.h`
**Purpose**: Handles high-speed flight data logging. It captures critical flight data (Gyro, PID, Motors, RC) at a high rate (83Hz) and stores it in a RAM ring buffer. It uses a separate FreeRTOS task on Core 0 to process logs without creating jitter in the main flight control loop (Core 1).

---

## Function Reference

### `void blackbox_init(void)`
**Description**: Initializes the blackbox system.
- **Details**:
  - Creates a FreeRTOS Queue (`log_queue`) to buffer incoming logs.
  - Clears the internal ring buffer.
  - Starts a background task (`blackbox_task`) pinned to Core 0.

### `void blackbox_log(const blackbox_entry_t *entry)`
**Description**: Logs a single data entry. This is designed to be **non-blocking** and safe to call from the critical control loop.
- **Parameters**: `entry` - Pointer to the data structure.
- **Details**:
  - Checks if we are in an Interrupt (ISR) or Task context.
  - Sends the data to the `log_queue`.
  - If the queue is full, the data is **dropped** rather than blocking the CPU. This ensures flight stability is never compromised by logging.

### `void blackbox_start(void) / blackbox_stop(void)`
**Description**: Controls the recording state.
- **Details**: Logging only happens when `recording` is true (usually when Armed).

### `const blackbox_entry_t *blackbox_get_entry(uint16_t index)`
**Description**: Retrieves a logged entry for playback or download.
- **Details**: Handles ring-buffer wrapping logic to return expected data even if the buffer overwrote itself (though max size is tuned to avoid this for short tuning hops).

---

## Code Explanation (Line-by-Line)

### `lib/blackbox/blackbox.h`

```c
9: #define BLACKBOX_MAX_ENTRIES 1200
```
Defines buffer size. 1200 entries * 100 bytes = ~120KB. The ESP32 has ~320KB RAM, so this is a large chunk but safe.

```c
20: typedef struct __attribute__((packed)) { ... } blackbox_entry_t;
```
Defines the binary structure of a log entry. `__attribute__((packed))` ensures there is no padding bytes between fields, minimizing size.
Includes: Timestamp, Flags, Raw IMU (Gyro/Accel), Rate Loop data (Setpoints, Errors, I-terms), PID outputs, Motor values, and RC inputs.

### `lib/blackbox/blackbox.c`

```c
13: static blackbox_entry_t buffer[BLACKBOX_MAX_ENTRIES];
```
The main storage. It is a static array in RAM.

```c
23: static QueueHandle_t log_queue = NULL;
```
A FreeRTOS Queue used as a buffer between the Flight Loop (Producer) and the Blackbox Task (Consumer).

```c
32: static void blackbox_task(void *arg) {
...
38:   while (1) {
41:     if (xQueueReceive(log_queue, &entry, portMAX_DELAY) == pdTRUE) {
```
The Background Task:
- Runs in an infinite loop.
- `xQueueReceive` blocks (sleeps) until data arrives.
- `portMAX_DELAY`: Waits forever if empty, using 0 CPU.

```c
44:         memcpy(&buffer[write_index], &entry, sizeof(blackbox_entry_t));
47:         write_index = (write_index + 1) % BLACKBOX_MAX_ENTRIES;
```
When data arrives:
- Copies it into the main ring buffer.
- Updates `write_index`. The `%` operator handles the "Ring" behavior (wrapping back to 0 after the end).

```c
72:   xTaskCreatePinnedToCore(blackbox_task, "blackbox", 2048, NULL,
73:                           5, // Priority (lower than control loop)
74:                           &blackbox_task_handle, 0 // Core 0
```
Initialization:
- Task is pinned to **Core 0**.
- Core 1 is dedicated to the Flight Control Loop.
- Priority 5 is extremely low (Control loop is usually ~24+). This ensures logging never starves flight code.

```c
82: void blackbox_log(const blackbox_entry_t *entry) {
...
87:   // Non-blocking send - if queue is full, drop the entry
99:     xQueueSend(log_queue, entry, 0);
```
The Logging Function:
- Uses `0` timeout (`xQueueSend(..., 0)`).
- If the queue is full (Backlog task is too slow), it returns immediately (False). The data is lost, but the drone keeps flying perfectly.
# Config Library Documentation

**Files**: `lib/config/config.c`, `lib/config/config.h`
**Purpose**: Manages persistent system configuration (PID gains, limits, etc.) using the ESP32's Non-Volatile Storage (NVS). PIDs can be tuned via the web interface and saved across reboots.

---

## Data Structures

### `system_config_t`
The main configuration struct (defined in `config.h`):
- `roll_kp, _ki, _kd`: PID gains for Roll rate.
- `pitch_kp, ...`: PID gains for Pitch rate.
- `yaw_kp, ...`: PID gains for Yaw rate.
- `rate_output_limit`: Max Allowed PID Output (prevents motor saturation).
- `rate_integral_limit`: Max I-term (anti-windup).
- `low_bat_threshold`: Voltage (mV) to trigger warning LED.

---

## Function Reference

### `void config_load_defaults(void)`
**Description**: Loads hardcoded "Safe Start" PID values. called if NVS load fails or is empty.
- **Values**:
  - Roll/Pitch P=0.4, I=0.2, D=0.03. (Conservative for F450).
  - Yaw P=2.5, I=2.5.
  - Rate Output Limit: 400.

### `void config_save_to_nvs(void)`
**Description**: Saves the current `sys_cfg` struct to NVS.
- **Details**:
  - Opens NVS namespace "pid_cfg".
  - Writes each float value individually using `nvs_set_blob`.
  - Commits changes to flash.

### `bool config_load_from_nvs(void)`
**Description**: Loads configuration from NVS.
- **Returns**: `true` if all values loaded successfully, `false` otherwise.
- **Details**:
  - Tries to read each PID key.
  - If any key is missing, it returns false (triggering default load).
  - Includes input clamping (`load_float_clamped`) to prevent corrupted flash data from causing dangerous PID values (NaN or Infinity).

---

## Code Explanation (Line-by-Line)

### `lib/config/config.c`

```c
6: system_config_t sys_cfg;
```
Global instance of the configuration struct. This is accessed by `pid.c` / `rate_control.c` during flight.

```c
8: #define NVS_NAMESPACE "pid_cfg"
```
Defines the partition name in NVS. This separates PID config from other data (like IMU calibration).

```c
10: void config_load_defaults(void) {
14:   sys_cfg.roll_kp = 0.40f;
...
```
Sets the "Safe" default values. These are tuned for a 450mm frame, 1000KV motors, 10-inch props style quad.

```c
36: void config_save_to_nvs(void) {
37:   nvs_handle_t handle;
38:   esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
```
Opens the Non-Volatile Storage for writing.

```c
43:   nvs_set_blob(handle, "roll_kp", &sys_cfg.roll_kp, sizeof(float));
...
55:   nvs_commit(handle);
```
Saves each variable as a binary "blob" (block of bytes) and commits to flash memory.

```c
60: static bool load_float_clamped(...) {
64:   if (nvs_get_blob(handle, key, &val, &len) != ESP_OK)
65:     return false;
67:   if (val < min_val) val = min_val;
```
Helper function:
1. reads a float from NVS.
2. Checks if the read was successful.
3. **Clamps** the value to a safe range (-1000 to +1000). This protects against flash corruption turning P=0.4 into P=99999.0 which would crash the drone instantly.
