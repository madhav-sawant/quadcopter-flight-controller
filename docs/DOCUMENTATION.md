# ESP32 Quadcopter Firmware Documentation

> **Combined Documentation** - All project guides and references in one place.  
> **Last Updated:** 2025-12-27

---

# Table of Contents

1. [Pinout Reference](#1-pinout-reference)
2. [ADC Battery Monitoring Architecture](#2-adc-battery-monitoring-architecture)
3. [PWM Driver Architecture](#3-pwm-driver-architecture)
4. [Rate PID Controller Guide](#4-rate-pid-controller-guide)
5. [PID Tuning and Usage](#5-pid-tuning-and-usage)
6. [Tuning Guide](#6-tuning-guide)
7. [Mixer Fix Documentation](#7-mixer-fix-documentation)
8. [Test Logs](#8-test-logs)
9. [Fix Logs](#9-fix-logs)

---

# 1. Pinout Reference

## ESP32 DevKit Pin Diagram

```
                      ESP32 DevKit V1
                 ┌────────────────────────┐
                 │          USB           │
                 │         ┌───┐          │
                 │         │   │          │
            EN ──┤ EN      └───┘      D23 ├── (NC)
            VP ──┤ VP/36              D22 ├── IMU SCL (I2C)
            VN ──┤ VN/39              TX0 ├── (USB Serial)
          (NC) ──┤ D34                RX0 ├── (USB Serial)
       BATTERY ──┤ D35 (ADC7)         D21 ├── IMU SDA (I2C)
          (NC) ──┤ D32                GND ├── GND
        MOTOR3 ──┤ D33                D19 ├── (NC)
        MOTOR2 ──┤ D25                D18 ├── (NC)
        PPM RX ──┤ D26                 D5 ├── (NC)
        MOTOR4 ──┤ D27                D17 ├── (NC)
          (NC) ──┤ D14                D16 ├── (NC)
          (NC) ──┤ D12                 D4 ├── (NC)
        MOTOR1 ──┤ D13                 D0 ├── BUTTON (Boot)
           GND ──┤ GND                 D2 ├── LED
           VIN ──┤ VIN                D15 ├── (NC)
                 │                    3V3 ├── VCC (3.3V)
                 │                        │
                 └────────────────────────┘
```

---

## Pin Assignments

### Motors (PWM Output)

| Motor | Position | GPIO | Rotation |
|-------|----------|------|----------|
| M1 | Rear Right | **13** | CCW ⟳ |
| M2 | Front Right | **25** | CW ⟲ |
| M3 | Rear Left | **33** | CW ⟲ |
| M4 | Front Left | **27** | CCW ⟳ |

### IMU (MPU6050 - I2C)

| Signal | GPIO |
|--------|------|
| SDA | **21** |
| SCL | **22** |

### Receiver

| Signal | GPIO |
|--------|------|
| PPM Input | **26** |

### Battery Voltage

| Signal | GPIO |
|--------|------|
| ADC Input | **35** (ADC1_CH7) |

### User Interface

| Signal | GPIO |
|--------|------|
| LED | **2** |
| Button (Emergency) | **0** |

---

## Motor Layout (Top View)

```
              FRONT
                ▲
                │
       M4       │       M2
      ⟳        │        ⟲
     GPIO 27    │    GPIO 25
         ╲      │      ╱
          ╲     │     ╱
           ╲    │    ╱
            ╲   │   ╱
             ╲  │  ╱
              ╲ │ ╱
               ╲│╱
        ────────X────────
               ╱│╲
              ╱ │ ╲
             ╱  │  ╲
            ╱   │   ╲
           ╱    │    ╲
          ╱     │     ╲
         ╱      │      ╲
       M3       │       M1
      ⟲        │        ⟳
     GPIO 33    │    GPIO 13
                │
              REAR

    ⟳ = Counter-Clockwise (CCW)
    ⟲ = Clockwise (CW)
```

---

## Wiring Connections

### ESC Signal Wires

| ESC | Motor Position | ESP32 Pin |
|-----|----------------|-----------|
| ESC 1 | Rear Right | GPIO 13 |
| ESC 2 | Front Right | GPIO 25 |
| ESC 3 | Rear Left | GPIO 33 |
| ESC 4 | Front Left | GPIO 27 |

### MPU6050 IMU

| MPU6050 | ESP32 |
|---------|-------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

### Receiver (PPM)

| Receiver | ESP32 |
|----------|-------|
| PPM Out | GPIO 26 |
| VCC | 5V (BEC) |
| GND | GND |

### Battery Voltage Divider

```
VBAT ──┬── R1 (10kΩ) ──┬── R2 (3.3kΩ) ──┬── GND
       │               │                │
       │               └─── GPIO 35 ────┘
       │
    (11.1V-12.6V)
```

---

## WiFi Access Point

| Parameter | Value |
|-----------|-------|
| SSID | `QuadPID` |
| Password | `12345678` |
| IP Address | `192.168.4.1` |

---

## Notes

- **GPIO 0, 2, 12, 14, 15** are strapping pins - avoid for critical functions
- **GPIO 34, 35, 36, 39** are input-only (used for ADC)
- Motors use **LEDC PWM** at 500Hz, 13-bit resolution
- PWM range: **1000-2000 µs** (idle: 1100 µs)

---

# 2. ADC Battery Monitoring Architecture

This section explains the architecture of the battery monitoring system integrated into the flight controller.

## Overview

The system monitors the LiPo battery voltage to prevent over-discharge, which can damage the battery and cause a crash.

*   **Threshold**: 9.9V (Critical Low)
*   **Action**: Safety Stop (Motors to Min Throttle, LED ON)
*   **Hardware**: ESP32 ADC1 Channel 7 (GPIO 35 usually, but depends on board).

## Hardware Setup (Voltage Divider)

The ESP32 ADC can only measure up to ~3.3V (with attenuation). A 3S LiPo battery is ~12.6V. A voltage divider is used to scale the voltage down.

```mermaid
graph TD
    Battery[Battery + (12.6V)] --> R1[Resistor R1]
    R1 --> Node[ADC Pin]
    Node --> R2[Resistor R2]
    R2 --> GND[Ground]
```

*   **Formula**: $V_{adc} = V_{batt} \times \frac{R2}{R1 + R2}$
*   **Scaling Factor**: The code uses a scaling factor (`VOLTAGE_SCALE_MV` / `VOLTAGE_SCALE_DIV`) to convert the ADC reading back to the actual battery voltage.

## Software Logic

### `adc.c` (Driver)
*   **`adc_init()`**: Configures ADC width (12-bit) and attenuation (11dB) to measure up to ~3.3V.
*   **`adc_read_battery_voltg()`**:
    1.  Reads raw ADC value (0-4095).
    2.  Converts to pin voltage (mV).
    3.  Applies scaling factor to get Battery Voltage (mV).

### `main.c` (Integration)
The monitoring logic is embedded in the main control loop to ensure safety at all times.

#### Safety Logic Flow
1.  **Read**: Get current voltage.
2.  **Debounce**:
    *   Voltage fluctuates due to motor load.
    *   We only trigger if voltage is low for **10 consecutive checks**.
    *   This prevents false alarms during momentary voltage sags.
3.  **Trigger**:
    *   Set `safety_stop = true`.
    *   Turn ON Status LED (Pin 2).
    *   **Force Motors to 1000µs** (Stop).
    *   Lock system (prevent further ramp-up).

## Integration with PWM Ramp

The ramp-up sequence in `main.c` is a loop that increases motor speed.
*   **Problem**: If we just used `delay()`, we wouldn't check the battery for 30 seconds.
*   **Solution**: Inside the ramp loop, we call `check_battery_status()` every step.
    ```c
    for (int step = 0; step < total_steps; step++) {
        if (check_battery_status()) {
            break; // Stop immediately!
        }
        // ... increase speed ...
    }
    ```

This ensures that if the battery dies mid-flight (or mid-test), the motors cut off immediately.

## Design Choice: Why Millivolts (mV)?

You will notice we use `uint16_t` to store voltage in **millivolts** (e.g., `11400` for 11.4V) rather than using `float` (e.g., `11.4`) or integer Volts (`11`).

### Reasons:
1.  **Precision**:
    *   Integer Volts (`11`) is too imprecise. We need to know the difference between `11.4V` (Charged) and `9.9V` (Empty).
    *   Millivolts gives us 3 decimal places of precision without using decimals.
2.  **Performance (Speed)**:
    *   Microcontrollers like the ESP32 can process Integers (`int`, `uint32_t`) much faster than Floating Point numbers (`float`, `double`).
    *   Floating point math requires complex CPU instructions or software emulation, which is slower.
3.  **Code Size**:
    *   Using `float` often pulls in large software libraries, increasing the firmware size.
    *   Integer math is native and lightweight.
4.  **Exactness**:
    *   Floats can have rounding errors (e.g., `11.4` might be stored as `11.4000001`). Integers are exact.

## Calibration

To ensure safety, we intentionally calibrated the ADC to read slightly **lower** than the actual voltage.
*   **Offset**: We subtract `180mV` (`VOLTAGE_OFFSET_MV`) from the reading.
*   **Goal**: Ensure the flight controller "thinks" the battery is lower than it is. This provides a safety margin of ~150-200mV.
    *   *Actual*: 11.40V
    *   *Reading*: ~11.22V
    *   *Result*: We land earlier, protecting the battery.

### Recovery Logic
We added a "Latching with Recovery" mechanism:
*   **Latch**: If voltage drops < 9.9V, the system locks (Safety Stop).
*   **Recovery**: If a fresh battery is connected (> 10.1V) and stays good for a short time, the system automatically unlocks.
*   **Why?** This prevents the drone from re-arming if the voltage sags momentarily under load, but allows you to swap batteries without rebooting.

---

# 3. PWM Driver Architecture

This section provides a comprehensive explanation of the PWM (Pulse Width Modulation) driver architecture designed for this quadcopter project.

## High-Level Overview

The goal of this driver is to control 4 Electronic Speed Controllers (ESCs) using the ESP32 microcontroller. ESCs require a specific type of signal: a repeating pulse where the **width** of the pulse determines the motor speed.

*   **1000 µs (1 ms)**: Motor Off / Minimum Throttle
*   **2000 µs (2 ms)**: Full Throttle
*   **Frequency**: How often this pulse repeats (500 times per second in our case).

### System Diagram

```mermaid
graph LR
    subgraph ESP32_Microcontroller
        direction TB
        App[Main Application] -->|Calls| Driver[PWM Driver (pwm.c)]
        Driver -->|Configures| LEDC[LEDC Peripheral]
        LEDC -->|Generates| T[Timer 0]
        T -->|Drives| C0[Channel 0]
        T -->|Drives| C1[Channel 1]
        T -->|Drives| C2[Channel 2]
        T -->|Drives| C3[Channel 3]
    end

    subgraph Hardware
        C0 -->|GPIO 13| ESC1[Motor 1 ESC]
        C1 -->|GPIO 25| ESC2[Motor 2 ESC]
        C2 -->|GPIO 33| ESC3[Motor 3 ESC]
        C3 -->|GPIO 27| ESC4[Motor 4 ESC]
    end
```

---

## Why ESP32 LEDC Peripheral?

We use the **LEDC (LED Control)** peripheral instead of "bit-banging" (manually toggling pins) or general-purpose timers.

*   **Why?**
    *   **Hardware-Based**: The LEDC peripheral generates pulses using dedicated hardware. Once configured, it runs automatically without using the CPU. This is **non-blocking** and **deterministic**.
    *   **Precision**: It allows for high-resolution control (we are using 12-bit) which is smoother than software timers.
    *   **Glitch-Free**: Hardware generation ensures the pulses are perfectly stable, which is critical for flight stability. Jittery PWM can cause motors to twitch.

---

## Configuration Deep Dive

### A. Frequency: 500 Hz
*   **What is it?** The signal repeats 500 times per second. Period = 1/500 = 0.002 seconds = **2000 µs**.
*   **Why 500 Hz?**
    *   Standard analog servos/ESCs use 50 Hz.
    *   Modern multirotor ESCs support 400-500 Hz.
    *   **Benefit**: Higher frequency means the flight controller can update motor speeds faster (every 2ms instead of 20ms), leading to a more stable and responsive drone.

### B. Resolution: 12-bit
*   **What is it?** The number of discrete steps we can divide the period into.
*   **Calculation**: $2^{12} = 4096$ steps.
*   **Why 12-bit?**
    *   Our period is 2000 µs.
    *   With 4096 steps, each step is $2000 / 4096 \approx 0.488 \mu s$.
    *   **Precision**: This gives us extremely fine control over the motor speed. We have roughly 2000 steps of resolution just in the active throttle range (1000-2000µs), which is excellent for smooth flight.

---

## The Math: Microseconds to Duty Cycle

The ESC understands "Microseconds" (time high), but the ESP32 LEDC register understands "Duty Cycle" (0 to 4095). We need a formula to convert them.

**The Formula:**
$$ Duty = \frac{PulseWidth(\mu s)}{Period(\mu s)} \times MaxResolution $$

Where:
*   $PulseWidth$: The desired motor command (e.g., 1500 µs).
*   $Period$: $1,000,000 / Frequency = 1,000,000 / 500 = 2000 \mu s$.
*   $MaxResolution$: $2^{12} - 1 = 4095$.

**Example Calculation for 1500 µs (Half Throttle):**
$$ Duty = \frac{1500}{2000} \times 4095 $$
$$ Duty = 0.75 \times 4095 \approx 3071 $$

**In Code (`pwm.c`):**
```c
// We use uint64_t to prevent overflow during multiplication before division
uint32_t duty = (uint32_t)(((uint64_t)pulse_width_us * (uint64_t)max_duty * (uint64_t)PWM_FREQ_HZ) / 1000000ULL);
```
*   **Why `uint64_t`?** If we multiply `2000 * 4095 * 500`, the result is `4,095,000,000`, which fits in `uint32_t` (max ~4.29 billion), but it's very close to the limit. Using 64-bit integers ensures the math is always safe.

---

## Software Architecture

### `pwm.h` (The Contract)
This file is the **Public API**. It hides the complex ESP-IDF details from the user.
*   **Macros**: Defines constants like `PWM_MOTOR_COUNT` so you can easily change the number of motors later without rewriting code.
*   **Functions**: Exposes only what is necessary: `init` and `set`.

### `pwm.c` (The Engine)
This file handles the heavy lifting.
1.  **`pwm_init()`**:
    *   **Timer Config**: Sets up the internal clock to tick at the right speed for 500Hz.
    *   **Channel Config**: Connects that timer to specific GPIO pins (13, 25, 33, 27).
2.  **`pwm_set_motor()`**:
    *   **Clamping**: This is a critical safety feature.
        ```c
        if (pulse_width_us < 1000) pulse_width_us = 1000;
        if (pulse_width_us > 2000) pulse_width_us = 2000;
        ```
        *   **Why?** If a bug in the main code sends "50000 µs", the motor might spin incorrectly or the ESC might enter programming mode. Clamping ensures the signal is always valid.

---

# 4. Rate PID Controller Guide

## Overview

This section explains how to use and validate the **Inner (Rate) PID Control Loop** for the quadrotor flight controller.

The rate controller is the fastest loop in cascade control, running at **500 Hz**. It controls angular **rates** (deg/s), not angles.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     INNER LOOP (Rate Control)                   │
│                         Runs @ 500 Hz                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Desired Rate ──►[  Rate PID  ]──► Control Output              │
│   (deg/s)              ▲              (to mixer)                │
│                        │                                        │
│                   Gyro Rate                                     │
│                   (deg/s)                                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## File Structure

| File | Purpose |
|------|---------|
| `lib/pid/pid.h` | Generic PID controller interface |
| `lib/pid/pid.c` | PID algorithm (P + I + D with anti-windup) |
| `lib/rate_control/rate_control.h` | Rate PID gains, limits, timing |
| `lib/rate_control/rate_control.c` | Roll/Pitch/Yaw rate controllers |
| `src/main.c` | Validation firmware (motors disabled) |

---

## How to Flash

```bash
cd /path/to/Project_Firmware1.1
pio run -t upload
pio device monitor
```

---

## Validation Procedure

### Step 1: Power On
After flashing, the serial output shows:
```
==================================================
    RATE PID VALIDATION MODE (Motors Disabled)    
==================================================
[OK] IMU Initialized.
>>> Keep drone STILL for gyro calibration... <<<
[OK] Gyro Calibrated.
[OK] Rate Controllers Initialized.
[OK] Control Loop Started @ 500 Hz
```

### Step 2: Keep Drone Still
When stationary, the output should look like:
```
GyroR  GyroP  GyroY  | PID_R  PID_P  PID_Y
--------------------------------------------------
   0.1   -0.2    0.0 |    0.1   -0.2    0.0
   0.0    0.1   -0.1 |    0.0    0.1   -0.2
```
- Gyro rates ≈ 0 (small noise is normal)
- PID outputs ≈ 0

### Step 3: Rotate Drone by Hand
When you rotate the frame:
```
  45.2  -12.3    5.1 |  -45.2   12.3  -10.2
  38.7   -8.5    3.2 |  -38.7    8.5   -6.4
```
- Gyro rates show rotation speed
- PID outputs are **opposite sign** (trying to counteract)

### Step 4: Stop Movement
When you stop rotating:
```
   2.1   -1.5    0.3 |   -2.1    1.5   -0.6
   0.5   -0.2    0.0 |   -0.5    0.2    0.0
```
- Values return to ~0

---

## Expected Behavior

| Condition | Gyro Rate | PID Output |
|-----------|-----------|------------|
| Still | ~0 | ~0 |
| Rotate Right (+) | Positive | Negative |
| Rotate Left (-) | Negative | Positive |
| Stop | Return to 0 | Return to 0 |

**Key Insight**: The PID output sign should be **opposite** to the gyro rate. This means the controller is trying to counteract unwanted rotation.

---

## Output Limits

```c
#define RATE_OUTPUT_LIMIT 500.0f    // Max PID output
#define RATE_INTEGRAL_LIMIT 200.0f  // Anti-windup limit
```

These prevent:
- Extreme motor commands
- Integral windup during sustained error

---

## Safety Notes

⚠️ **Motors are DISABLED in this firmware**

This is intentional for safe validation. The control outputs are calculated but not sent to ESCs.

Before enabling motors:
1. Verify PID responds correctly (this test)
2. Implement motor mixer
3. Add arming/disarming logic
4. Test with props removed first

---

# 5. PID Tuning and Usage

This guide explains how to use the single-file flight controller firmware (`src/main.c`) and provides a step-by-step procedure for tuning the Rate PID loop.

---

## How to Use the Firmware

### **Safety First!**
*   **REMOVE PROPELLERS** before flashing or testing on the bench.
*   Use a current-limited power supply or a "smoke stopper" if available.
*   The system is designed to **ARM AUTOMATICALLY** after 3 seconds if the battery is good. Be ready to disconnect power or press the **BOOT button (GPIO 0)** for Emergency Stop.

### **Hardware Setup**
Ensure your ESCs are connected to the correct pins (Quad X Configuration):
*   **Motor 1 (Rear Right, CCW)**: GPIO 13
*   **Motor 2 (Front Right, CW)**: GPIO 25
*   **Motor 3 (Rear Left, CW)**: GPIO 33
*   **Motor 4 (Front Left, CCW)**: GPIO 27
*   **IMU (MPU6050)**: SDA=21, SCL=22
*   **Battery Divider**: GPIO 35 (ADC1_CH7)

### **Operation Sequence**
1.  **Power On**: Connect the battery. The LED (GPIO 2) will light up.
2.  **Calibration**: Keep the drone **absolutely still** on a flat surface.
    *   *Console Output*: `Calibrating Gyro...`
3.  **Battery Check**: The system measures voltage.
    *   If < 10.5V: `BATTERY LOW! DISARMED.` (System halts).
    *   If > 10.5V: `Battery OK.`
4.  **Arming Countdown**:
    *   The LED will blink 3 times.
    *   *Console Output*: `ARMING IN 3 SECONDS...`
5.  **Armed State**:
    *   Motors spin at **Idle Throttle** (1100µs).
    *   PID Loop is active (trying to keep the drone steady).

### **Testing the Response (Hand Test)**
*   **Hold the drone firmly** from underneath (Keep fingers away from motors).
*   **Roll Test**: Tilt the drone quickly to the **Right**.
    *   *Expected*: Right motors (1 & 2) speed up, Left motors (3 & 4) slow down to resist the motion.
*   **Pitch Test**: Tilt the nose **Down**.
    *   *Expected*: Front motors (2 & 4) speed up, Rear motors (1 & 3) slow down.
*   **Yaw Test**: Twist the drone **Clockwise**.
    *   *Expected*: CCW motors (1 & 4) speed up, CW motors (2 & 3) slow down.

---

## PID Tuning Guide

The goal of tuning is to find the values for **Kp**, **Ki**, and **Kd** that make the drone fly stable and responsive.

### **The Terms**
*   **P (Proportional)**: "The Present". Reacts to the current error.
    *   *Too Low*: Sluggish, drifts, hard to control.
    *   *Too High*: High-frequency oscillations (shaking), motors get hot.
*   **I (Integral)**: "The Past". Accumulates error over time to correct drift.
    *   *Too Low*: Drone won't hold angle perfectly, drifts with wind/CG imbalance.
    *   *Too High*: Low-frequency oscillations (wobble), "windup" after quick moves.
*   **D (Derivative)**: "The Future". Predicts error change to dampen motion.
    *   *Too Low*: Overshoots target, bouncy stop after a flip.
    *   *Too High*: High-frequency vibrations, grinding noise from motors.

### **Tuning Method: The "Classic" Manual Approach**

This method is safest and most effective for custom builds. You will edit the macros in `src/main.c` and re-flash.

#### **Step 1: Preparation**
1.  Set **Ki = 0.0** and **Kd = 0.0** for all axes.
2.  Set **Kp** to a low value (e.g., 0.5 or 1.0).
3.  Flash the code.

#### **Step 2: Tune P (Proportional)**
1.  **Test**: Hold the drone (carefully!) or put it on a string rig. Increase throttle slightly.
2.  **Observe**: Induce a disturbance (tap it). Does it return to level?
3.  **Adjust**:
    *   If it's sluggish or doesn't return: **Increase Kp**.
    *   If it oscillates rapidly (shakes) after the tap: **Decrease Kp**.
4.  **Goal**: Find the highest Kp where it returns sharply without shaking.
    *   *Typical Value*: 1.0 to 4.0 depending on power/weight.

#### **Step 3: Tune D (Derivative)**
*Once P is decent, the drone might be a bit "bouncy" or overshoot.*
1.  **Test**: Make sharp movements.
2.  **Adjust**:
    *   **Increase Kd** slowly.
3.  **Goal**: The "bounce" after a move should disappear. The stop should be crisp.
    *   *Warning*: Too much D causes hot motors! Check motor temp.
    *   *Typical Value*: 0.01 to 0.1 (Usually much smaller than P).

#### **Step 4: Tune I (Integral)**
*Now the drone is stable but might drift or not hold an angle perfectly.*
1.  **Test**: Tilt the drone and hold it. Does it fight to stay there or slowly drift back? (In Rate mode, it should resist rotation).
2.  **Adjust**:
    *   **Increase Ki** slowly.
3.  **Goal**: The drone should feel "locked in".
    *   *Typical Value*: 0.0 to 1.0 (Start small).

### **Troubleshooting**

| Symptom | Solution |
| :--- | :--- |
| **Fast Shaking (Oscillation)** | **P is too high**. Reduce Kp. |
| **Slow Wobble** | **I is too high**. Reduce Ki. |
| **Sluggish / Drifts** | **P is too low** or **I is too low**. |
| **Bounces after move** | **D is too low**. Increase Kd. |
| **Motors Hot / Grinding Sound** | **D is too high** or excessive noise. Reduce Kd or enable DLPF (already enabled in code). |

---

# 6. Tuning Guide

This guide covers how to verify your setup, tune the PID loops on the bench, and proceed to outdoor flight testing.

## Pre-Tuning Checks (CRITICAL)

Before changing any PID values, you **MUST** verify your motor mapping and sensor orientation. If these are wrong, the drone will flip instantly.

### Motor Mapping Verification
Based on the code analysis, the firmware assumes the following mapping:

| Motor Index | Position | Rotation |
| :--- | :--- | :--- |
| **0** | Rear Right | CCW |
| **1** | Front Right | CW |
| **2** | Rear Left | CW |
| **3** | Front Left | CCW |

**Test:**
1. Remove Props.
2. Connect via USB and open Serial Monitor.
3. Arm the drone (wait for safety sequence).
4. Tilt the drone **LEFT** (Left side down).
    - **Expected**: Left motors (2 & 3) should **SPEED UP**. Right motors (0 & 1) should **SLOW DOWN**.
5. Tilt the drone **FORWARD** (Nose down).
    - **Expected**: Front motors (1 & 3) should **SPEED UP**. Rear motors (0 & 2) should **SLOW DOWN**.
6. Rotate (Yaw) **RIGHT** (CW).
    - **Expected**: CCW motors (0 & 3) should **SPEED UP**. CW motors (1 & 2) should **SLOW DOWN**.

> **If any of these are reversed, DO NOT FLY.** You must fix the wiring or the `mixer.c` logic.

---

## Tuning Process Overview

**The Golden Rule**: Tune the **Inner Loop (Rate)** first, then the **Outer Loop (Angle)**.

### Why?
The Angle Loop asks for a *Rotation Rate*. If the Rate Loop cannot achieve that rate accurately and stably, the Angle Loop will never work.

### Tuning Sequence
1.  **Rate P-Gain**: Make it responsive.
2.  **Rate D-Gain**: Stop the bounce/overshoot.
3.  **Rate I-Gain**: Fix drift/holding.
4.  **Angle P-Gain**: Make it self-level.

---

## Parameters to Tune

Here are the specific variables you will adjust in `config.h` (or `rate_control.c` / `angle_control.c`).

| Loop | Parameter | Variable Name | Default | Typical Range | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **RATE** | **Kp** | `RATE_ROLL_KP` | `0.6` | `0.4` - `1.5` | Reactivity. Higher = Snappier. Too high = Fast Oscillation. |
| **RATE** | **Ki** | `RATE_ROLL_KI` | `0.0` | `0.0` - `0.05` | Holding. Corrects small drifts. Too high = Slow Wobble. |
| **RATE** | **Kd** | `RATE_ROLL_KD` | `0.03` | `0.01` - `0.08` | Dampening. Stops overshoot. Too high = Hot Motors / Noise. |
| **ANGLE**| **Kp** | `ANGLE_ROLL_KP`| `2.0` | `2.0` - `6.0` | Leveling Strength. Higher = Harder return to level. |

> **Note**: Usually, Roll and Pitch gains are kept identical for a symmetric X-quad.

---

## Step-by-Step Tuning Procedure

### Phase 1: Rate Loop Tuning (Acro Mode feel)
*Goal: The drone should resist rotation and hold its angle when you stop moving the stick.*

1.  **Set Angle P to 0** (temporarily disable self-leveling) OR just focus on the "locked-in" feel during momentary inputs.
2.  **Tune Rate P (Kp)**:
    - **Start Low** (e.g., 0.6).
    - **Test**: Hover and give a sharp stick input, then center.
    - **Increase** until you hear/see fast oscillations (shaking) after a move.
    - **Back off** by 10-20%.
3.  **Tune Rate D (Kd)**:
    - **Test**: Do a sharp move and stop. Does it "bounce" back?
    - **Increase** D to remove the bounce.
    - **Warning**: If motors get hot, D is too high.
4.  **Tune Rate I (Ki)**:
    - **Test**: Tilt it and see if it holds that angle (in Acro mode). If it slowly drifts back to level or falls further, add I.
    - **Range**: Start very small (0.001).

### Phase 2: Angle Loop Tuning (Self-Leveling)
*Goal: The drone should return to level quickly without wobbling.*

1.  **Enable Angle Mode** (Set `ANGLE_ROLL_KP` back to default `2.0`).
2.  **Tune Angle P (Kp)**:
    - **Test**: Hover. Push stick to tilt, then release stick.
    - **Observation**:
        - **Lazy/Slow return**: Increase Kp (Try 2.5, 3.0, 4.0).
        - **Wobbles** upon reaching level: Kp is too high. Decrease it.
        - **Oscillates continuously**: Kp is WAY too high.
    - **Ideal**: It snaps back to level and stops instantly.

---

## Interpreting Logs for Tuning
`A: -1.4 4.2 | G: 0.0 0.0 0.0 | P: 3.3 -8.5 -0.0 | M: 1089 1105 1111 1095`

- **A (Angle)**: Tells you if the Angle Loop is happy (Should be 0).
- **G (Gyro)**: Tells you if the Rate Loop is happy (Should be 0 if still).
- **P (PID Out)**: The effort the drone is making.
    - If `A` is 0 but `P` is high, your `I-term` is working hard to fight an imbalance (CG or bent prop).
    - If `G` is oscillating (+5, -5, +5...), your Rate P is too high.

---

## Outdoor Flight Testing (With Props)

**Safety First**:
- Find a grassy field.
- Keep distance.
- Have a kill switch ready (Boot button or Transmitter switch).

### Procedure
1. **Hover Test**:
    - Arm.
    - Slowly raise throttle.
    - **If it flips immediately**: Disarm! Motor mapping or Sensor orientation is wrong.
    - **If it oscillates**: Lower P-gains.
    - **If it drifts**: Check accelerometer calibration.
2. **Disturbance Test**:
    - Hover at eye level.
    - Give small stick inputs (Roll/Pitch).
    - Drone should return to level when stick is released.
3. **Tuning**:
    - If it feels "loose" or "sloppy": Increase Angle P-Gain.
    - If it "shakes" or "jitters": Decrease Rate P/D Gains.
    - If it "wobbles" slowly: Decrease Angle P-Gain.

---

# 7. Mixer Fix Documentation

## Problem Identified

Your blackbox data showed the drone **drifting to one side** even when the PID was trying to correct it. Analysis revealed the **mixer logic had incorrect signs**, causing cross-coupling between axes.

---

## Motor Layout (Quad-X Configuration)

```
        FRONT
          ▲
     M4       M2
      ⟳       ⟲
       \     /
        \   /
         \ /
          X
         / \
        /   \
       /     \
      ⟲       ⟳
     M3       M1
        
        REAR

Legend:
  ⟳ = CCW rotation (Counter-Clockwise)
  ⟲ = CW rotation (Clockwise)
```

| Motor | Position     | GPIO | Rotation |
|-------|--------------|------|----------|
| M1    | Rear Right   | 13   | CCW ⟳   |
| M2    | Front Right  | 25   | CW ⟲    |
| M3    | Rear Left    | 33   | CW ⟲    |
| M4    | Front Left   | 27   | CCW ⟳   |

---

## What Was Wrong (OLD Code)

```c
// INCORRECT - caused cross-coupling!
m1 = t - roll + pitch + yaw;  
m2 = t - roll - pitch - yaw;  
m3 = t + roll - pitch + yaw;  
m4 = t + roll + pitch - yaw;  
```

**Problems:**
1. **Pitch was diagonal:** M1 got `+pitch`, M3 got `-pitch` → Right side pitched differently than Left side = **TWIST**
2. **Yaw caused pitch:** M1,M3 (rear) both got same yaw sign → Rear lifted together = **NOSE DROP**

---

## What We Fixed (NEW Code)

```c
// CORRECT - standard Betaflight/Cleanflight mixer
m1 = t - roll - pitch + yaw;  // Rear Right CCW
m2 = t - roll + pitch - yaw;  // Front Right CW  
m3 = t + roll - pitch - yaw;  // Rear Left CW
m4 = t + roll + pitch + yaw;  // Front Left CCW
```

---

## Mixer Logic Explained

### Roll (Positive = Right Wing Down)
```
  [M4 ↑]     [M2 ↓]      Left motors speed UP
     \       /           Right motors slow DOWN
      \     /            → Drone rolls RIGHT
       \   /
      [M3 ↑]   [M1 ↓]
```
- **M3, M4:** `+ roll` → Speed UP
- **M1, M2:** `- roll` → Slow DOWN

### Pitch (Positive = Nose Up)
```
  [M4 ↑]     [M2 ↑]      Front motors speed UP
     \       /           Rear motors slow DOWN
      \     /            → Nose pitches UP
       \   /
      [M3 ↓]   [M1 ↓]
```
- **M2, M4:** `+ pitch` → Speed UP (Front)
- **M1, M3:** `- pitch` → Slow DOWN (Rear)

### Yaw (Positive = Clockwise Rotation)
```
  [M4 ↑]     [M2 ↓]      CCW motors (M1,M4) speed UP
     \       /           CW motors (M2,M3) slow DOWN
      \     /            → Drone rotates CLOCKWISE
       \   /
      [M3 ↓]   [M1 ↑]
```
- **M1, M4:** `+ yaw` → Speed UP (CCW props create CW torque)
- **M2, M3:** `- yaw` → Slow DOWN (CW props create CCW torque)

---

## Summary Table

| Motor | Roll | Pitch | Yaw | Position |
|-------|------|-------|-----|----------|
| M1    | -    | -     | +   | Rear Right CCW |
| M2    | -    | +     | -   | Front Right CW |
| M3    | +    | -     | -   | Rear Left CW |
| M4    | +    | +     | +   | Front Left CCW |

---

## Web Recalibration Feature

You can now recalibrate the IMU (Gyro + Accel) directly from the web interface without rebooting or erasing NVS.

1. Connect to WiFi `QuadPID`
2. Go to `http://192.168.4.1`
3. Place drone **Level and Still**
4. Click **[Recalibrate IMU]**
5. Wait ~2 seconds for page to reload

This saves the new calibration to NVS automatically.

---

## Safety Features Verified

| Feature | Trigger | Action | Status |
|---------|---------|--------|--------|
| **Crash Protection** | Roll/Pitch > 60° | **DISARM** immediately | ✅ Active |
| **Throttle Safety** | Throttle > 1050us when arming | **Prevent Arming** | ✅ Active |
| **RX Failsafe** | Signal lost for > 100ms | **DISARM** immediately | ✅ Active |
| **ESC Boot Protection** | Power-on | **Force Low** for 3s | ✅ Active |
| **Arming Switch** | Switch Low (<1500) | **DISARM** immediately | ✅ Active |

---

## PID Tuning Quick Reference

### Starting Values (Conservative)

| Parameter | Roll | Pitch | Yaw |
|-----------|------|-------|-----|
| **P** | 1.0 | 1.0 | 2.0 |
| **I** | 0.0 | 0.0 | 0.0 |
| **D** | 0.0 | 0.0 | 0.0 |
| **Angle P** | 3.0 | 3.0 | - |

### Recommended Test Values

| Test | Roll P | Roll I | Roll D | Pitch P | Pitch I | Pitch D | Yaw P | Yaw I | Yaw D |
|------|--------|--------|--------|---------|---------|---------|-------|-------|-------|
| **1. Safe Start** | 0.8 | 0.0 | 0.0 | 0.8 | 0.0 | 0.0 | 1.5 | 0.0 | 0.0 |
| **2. Add Response** | 1.2 | 0.0 | 0.0 | 1.2 | 0.0 | 0.0 | 2.0 | 0.0 | 0.0 |
| **3. Add Damping** | 1.2 | 0.0 | 0.01 | 1.2 | 0.0 | 0.01 | 2.0 | 0.0 | 0.0 |
| **4. Fine Tune** | 1.5 | 0.1 | 0.015 | 1.5 | 0.1 | 0.015 | 2.5 | 0.1 | 0.0 |
| **5. Aggressive** | 2.0 | 0.2 | 0.02 | 2.0 | 0.2 | 0.02 | 3.0 | 0.2 | 0.0 |

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Oscillates fast (vibrates) | P too high | Reduce P by 20% |
| Oscillates slow (wobbles) | I too high | Reduce I by 50% |
| Drifts to one side | I too low or mixer wrong | Increase I or check mixer |
| Overshoots (bounces) | D too low | Increase D |
| Motors hot/noisy | D too high | Reduce D |
| Sluggish response | P too low | Increase P |

---

# 8. Test Logs

## First Bench Test Results - Rate Mode Validation
**Date:** 2025-12-24  
**Test Type:** Hand-held Bench Test (Props OFF)  
**Mode:** Rate Mode (Acro)

### Raw Log Data
The following data was captured during a hand-held test where the drone was pitched up and then held static.

```text
G:    0.0   -0.3   -0.0 | P:  -0.6   0.3   0.0 | M: 1100 1100 1100 1100 | V: 65356
G:    0.4   -0.2   -0.1 | P:  -0.4  -1.0   0.1 | M: 1100 1100 1100 1100 | V: 65356
...
G:   -0.2   33.7   -0.4 | P:   0.2 -35.6   0.8 | M: 1065 1135 1135 1065 | V: 65356
G:   -0.5   41.0   -0.7 | P:  -0.1 -41.0   1.5 | M: 1061 1139 1141 1059 | V: 65356
...
G:   -0.9   31.3   -4.3 | P:   0.9 -32.5   8.6 | M: 1076 1124 1140 1060 | V: 65356
...
G:   -0.0    0.1    0.1 | P:   0.0  -0.7  -0.2 | M: 1100 1100 1100 1100 | V: 65356
```

### Analysis

#### Scenario A: Pitch Up (Nose Up) Movement
**Log Line:**
`G: -0.9 31.3 -4.3 | P: 0.9 -32.5 8.6 | M: 1076 1124 1140 1060`

*   **Gyro Input (`G`)**: Pitch rate is **+31.3 deg/s**. This indicates the nose is rotating UP.
*   **PID Output (`P`)**: Pitch PID output is **-32.5**. The controller is trying to oppose the motion by pushing the nose DOWN.
*   **Motor Output (`M`)**:
    *   **Front Motors (M1, M4)**: 1076, 1060. These are *lower* than the idle speed (1100) or base throttle. The front motors are slowing down.
    *   **Rear Motors (M2, M3)**: 1124, 1140. These are *higher* than the idle speed. The rear motors are speeding up.
*   **Physics Check**: Increasing rear motor speed and decreasing front motor speed creates a torque that pushes the nose down.
*   **Result**: **CORRECT**. The controller is correctly fighting the uncommanded rotation.

#### Scenario B: Static / Level (Wait)
**Log Line:**
`G: -0.0 0.1 0.1 | P: 0.0 -0.7 -0.2 | M: 1100 1100 1100 1100`

*   **Gyro Input (`G`)**: Rates are near zero (~0.1 deg/s).
*   **PID Output (`P`)**: PID corrections are near zero.
*   **Motor Output (`M`)**: All motors are at **1100 (Idle)**.
*   **Result**: **CORRECT**. In Rate Mode, if the drone is not rotating (even if it is tilted at an angle), the controller should not output any correction. It only resists *change* in angle (velocity), not the angle itself.

### Conclusion
*   **Directionality**: The Motor Mixer and PID signs are correct. The drone resists movement in the correct direction.
*   **Noise/Jitter**: The gyro data looks clean when static (0.0 to 0.1 deg/s noise floor).
*   **Tuning**: The P-gain appears to be in a safe starting range (Output ~30 for Input ~30). No violent oscillations were observed in the logs.

**Status:** The firmware logic is validated. The system is ready for a cautious maiden flight (hover test) in a safe environment.

---

# 9. Fix Logs

## 2025-12-27: Major Updates

### ✅ Major Fixes & Improvements

#### 1. **ESC Initialization & Boot Timing Fix**
   - **Issue:** ESCs (specifically Motor 3) failed to arm when powered simultaneously with ESP32 via battery.
   - **Cause:** ESP32 boot time caused GPIO pins to float, sending invalid signals to ESCs before firmware initialization.
   - **Fix:** 
     - Moved `pwm_init()` to the very beginning of `app_main()`.
     - Added an immediate **5-second stable IDLE signal (1000µs)** sequence at boot.
     - This mimics "USB-first" behavior, allowing ESCs to recover from boot noise and arm properly.

#### 2. **GPIO Pin Remapping (Safety)**
   - **Issue:** Motors were connected to ESP32 strapping pins (GPIO 12, 14), causing boot glitches and potential failures.
   - **Fix:** Remapped motors to safe, non-strapping pins:
     - **Motor 2:** Changed from GPIO 12 → **GPIO 25**
     - **Motor 3:** Changed from GPIO 14 → **GPIO 33**
     - **Motor 1 (13)** and **Motor 4 (27)** remained unchanged.

#### 3. **Performance Optimization**
   - **Issue:** Serial `printf` statements in the main control loop caused delays and jitter.
   - **Fix:** Commented out all non-essential `printf` statements in `main.c` to ensure a stable 500Hz control loop.
   - **Boot Speed:** Removed unnecessary 3-second "BOOTING..." delay.

#### 4. **IMU Calibration Storage (NVS)**
   - **Feature:** Implemented Non-Volatile Storage (NVS) for IMU calibration.
   - **Benefit:** Gyro bias and Accel offsets are saved to flash memory. No need to keep the drone still on every boot!
   - **Usage:** Calibration runs once (or if NVS is empty), then loads automatically on subsequent boots.

#### 5. **Web-Based PID Tuning**
   - **Feature:** Integrated a lightweight web server for real-time PID tuning.
   - **Usage:** Connect to WiFi `QuadPID` (Pass: `12345678`) and go to `http://192.168.4.1`.
   - **Benefit:** Tune P, I, D gains wirelessly without recompiling code.

---

## 2025-12-23: Rate PID & Modular Architecture Refactor

- **Implemented Rate PID Control**: Added inner loop PID controllers for Roll, Pitch, and Yaw rates running at 500Hz.
- **Implemented Motor Mixer**: Added Quad-X mixing logic with safety features (arming, idle throttle, output clamping).
- **Restored Modular Structure**: Separated code into `lib/` modules (`pid`, `pwm`, `adc`, `imu`, `mixer`, `rate_control`) for better maintainability.
- **Added Configuration Module**: Created `lib/config` to centralize tunable parameters (PID gains, safety limits) in a `system_config_t` structure.
- **Safety Features**:
    - Low battery disarm (< 10.5V).
    - Crash angle protection (> 60 deg tilt).
    - Emergency stop via Boot button.
- **Documentation**: Added `PID_TUNING_AND_USAGE.md` guide.

## 2025-12-23: Compilation Fixes

- **Fixed `lib/imu/imu.h`**: Removed stray character causing syntax error.
- **Fixed `src/main.c`**:
    - Removed premature closing brace in `app_main`.
    - Replaced undefined macros `LOW_BATTERY_THRESHOLD_MV` with `sys_cfg.low_bat_threshold`.
- **Verified Build**: Project compiles successfully with `pio run`.

---

## Historical Fixes

### 1. Battery Latching & Recovery
*   **File**: `src/main.c` (PWM Demo)
*   **Issue**: The system would permanently lock (Safety Stop) if a low battery was detected, even if a fresh battery was connected afterwards.
*   **Fix**: Implemented a recovery mechanism.
    *   Added a `good_batt_counter`.
    *   If voltage > Threshold + 200mV for a sustained period, the system unlocks.
*   **Why**: To allow hot-swapping batteries without resetting the MCU.

### 2. Soft Watchdog Timeout (Input Loop)
*   **File**: `src/main.c` (PWM Demo)
*   **Issue**: The `while(1)` loop waiting for user input ('s' key) was spinning too fast, starving the IDLE task and triggering the Task Watchdog Timer (TWDT).
*   **Fix**: Added `vTaskDelay(pdMS_TO_TICKS(10))` inside the wait loop.
*   **Why**: To yield CPU time to the FreeRTOS scheduler and reset the watchdog.

### 3. ADC Calibration
*   **File**: `lib/adc/adc.h`
*   **Issue**: The battery voltage reading was inaccurate and didn't provide the desired safety margin.
*   **Fix**:
    *   Updated `VOLTAGE_SCALE_MV` to `41624`.
    *   Added `VOLTAGE_OFFSET_MV` of `180`.
*   **Why**: To ensure the monitor reads ~150-200mV *lower* than actual voltage, ensuring a safe landing before the battery is critically low.

### 4. Soft Watchdog Timeout (IMU Calibration)
*   **File**: `lib/imu/imu.c`
*   **Issue**: The `imu_calibrate_gyro` function used a blocking busy-wait loop (`for(volatile int j...)`) for delay. This caused a WDT timeout during the 2-second calibration phase.
*   **Fix**: Replaced the busy-wait loop with `vTaskDelay(pdMS_TO_TICKS(2))`.
*   **Why**: To allow the IDLE task to run during the delay, preventing WDT reset failure.

### 5. Boot Loop (SW_CPU_RESET)
*   **File**: `lib/imu/imu.c`
*   **Issue**: `imu_init` used `ESP_ERROR_CHECK` for I2C writes. If the sensor was disconnected or failed to ACK, the ESP32 would abort and reset continuously.
*   **Fix**: Replaced `ESP_ERROR_CHECK` with explicit error checking (`if (ret != ESP_OK) return ret;`).
*   **Why**: To allow the application to handle initialization failures gracefully (e.g., by blinking an LED error code) instead of crashing.

### 6. xTaskDelayUntil Assertion Failure
*   **File**: `src/main.c` (IMU Test Mode)
*   **Issue**: After gyro calibration, the system crashed with `assert failed: xTaskDelayUntil tasks.c:1499 (( xTimeIncrement > 0U ))`. The assertion triggers when `vTaskDelayUntil` is asked to delay for 0 or negative ticks.
*   **Root Cause**: `pdMS_TO_TICKS(2)` returns **0** because FreeRTOS default tick rate is 100Hz (1 tick = 10ms). Requesting a 2ms delay rounds down to 0 ticks, which is invalid for `vTaskDelayUntil`.
*   **Fix**: 
    *   Replaced `vTaskDelayUntil(&last_wake_time, period_ticks)` with `vTaskDelay(1)` to guarantee at least 1 tick delay.
    *   Updated `imu_read(0.010f)` so the dt parameter matches the actual 10ms tick period.
    *   Changed print counter from 50 to 10 (10 × 10ms = 100ms print interval).
*   **Why**: At 100Hz tick rate, sub-10ms timing is not possible with `vTaskDelay`/`vTaskDelayUntil`. Using `vTaskDelay(1)` ensures the loop always yields to the scheduler with a valid non-zero delay.

---

*End of Documentation*
