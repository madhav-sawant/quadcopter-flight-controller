# Rate PID Tuning & Usage Guide

This guide explains how to use the single-file flight controller firmware (`src/main.c`) and provides a step-by-step procedure for tuning the Rate PID loop.

---

## 1. How to Use the Firmware

### **Safety First!**
*   **REMOVE PROPELLERS** before flashing or testing on the bench.
*   Use a current-limited power supply or a "smoke stopper" if available.
*   The system is designed to **ARM AUTOMATICALLY** after 3 seconds if the battery is good. Be ready to disconnect power or press the **BOOT button (GPIO 0)** for Emergency Stop.

### **Hardware Setup**
Ensure your ESCs are connected to the correct pins (Quad X Configuration):
*   **Motor 1 (Front Right, CCW)**: GPIO 12
*   **Motor 2 (Rear Right, CW)**: GPIO 13
*   **Motor 3 (Rear Left, CCW)**: GPIO 14
*   **Motor 4 (Front Left, CW)**: GPIO 27
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
    *   *Expected*: Front motors (1 & 4) speed up, Rear motors (2 & 3) slow down.
*   **Yaw Test**: Twist the drone **Clockwise**.
    *   *Expected*: CCW motors (1 & 3) speed up, CW motors (2 & 4) slow down.

---

## 2. PID Tuning Guide

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

### **Where to Change Values**
Look for this section in `src/main.c`:

```c
/* ========================================================================== */
/*                             RATE CONTROL MODULE                            */
/* ========================================================================== */

#define RATE_ROLL_KP 1.0f  // <--- Change this
#define RATE_ROLL_KI 0.0f
#define RATE_ROLL_KD 0.02f
```

Change the values, save, and run `pio run -t upload`.
