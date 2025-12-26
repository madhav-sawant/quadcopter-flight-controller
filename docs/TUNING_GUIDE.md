# Quadrotor Tuning Guide

This guide covers how to verify your setup, tune the PID loops on the bench, and proceed to outdoor flight testing.

## 1. Pre-Tuning Checks (CRITICAL)

Before changing any PID values, you **MUST** verify your motor mapping and sensor orientation. If these are wrong, the drone will flip instantly.

### A. Motor Mapping Verification
Based on the code analysis, the firmware assumes the following mapping:

| Motor Index | Position | Rotation |
| :--- | :--- | :--- |
| **0** | Front Right | CCW |
| **1** | Rear Right | CW |
| **2** | Rear Left | CCW |
| **3** | Front Left | CW |

**Test:**
1. Remove Props.
2. Connect via USB and open Serial Monitor.
3. Arm the drone (wait for safety sequence).
4. Tilt the drone **LEFT** (Left side down).
    - **Expected**: Left motors (2 & 3) should **SPEED UP**. Right motors (0 & 1) should **SLOW DOWN**.
5. Tilt the drone **FORWARD** (Nose down).
    - **Expected**: Front motors (0 & 3) should **SPEED UP**. Rear motors (1 & 2) should **SLOW DOWN**.
6. Rotate (Yaw) **RIGHT** (CW).
    - **Expected**: CCW motors (0 & 2) should **SPEED UP**. CW motors (1 & 3) should **SLOW DOWN**.

> **If any of these are reversed, DO NOT FLY.** You must fix the wiring or the `mixer.c` logic.

---

## 2. Tuning Process Overview

**The Golden Rule**: Tune the **Inner Loop (Rate)** first, then the **Outer Loop (Angle)**.

### Why?
The Angle Loop asks for a *Rotation Rate*. If the Rate Loop cannot achieve that rate accurately and stably, the Angle Loop will never work.

### Tuning Sequence
1.  **Rate P-Gain**: Make it responsive.
2.  **Rate D-Gain**: Stop the bounce/overshoot.
3.  **Rate I-Gain**: Fix drift/holding.
4.  **Angle P-Gain**: Make it self-level.

---

## 3. Parameters to Tune

Here are the specific variables you will adjust in `config.h` (or `rate_control.c` / `angle_control.c`).

| Loop | Parameter | Variable Name | Default | Typical Range | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **RATE** | **Kp** | `RATE_ROLL_KP` | `0.6` | `0.4` - `1.5` | Reactivity. Higher = Snappier. Too high = Fast Oscillation. |
| **RATE** | **Ki** | `RATE_ROLL_KI` | `0.0` | `0.0` - `0.05` | Holding. Corrects small drifts. Too high = Slow Wobble. |
| **RATE** | **Kd** | `RATE_ROLL_KD` | `0.03` | `0.01` - `0.08` | Dampening. Stops overshoot. Too high = Hot Motors / Noise. |
| **ANGLE**| **Kp** | `ANGLE_ROLL_KP`| `2.0` | `2.0` - `6.0` | Leveling Strength. Higher = Harder return to level. |

> **Note**: Usually, Roll and Pitch gains are kept identical for a symmetric X-quad.

---

## 4. Step-by-Step Tuning Procedure

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

## 5. Interpreting Logs for Tuning
`A: -1.4 4.2 | G: 0.0 0.0 0.0 | P: 3.3 -8.5 -0.0 | M: 1089 1105 1111 1095`

- **A (Angle)**: Tells you if the Angle Loop is happy (Should be 0).
- **G (Gyro)**: Tells you if the Rate Loop is happy (Should be 0 if still).
- **P (PID Out)**: The effort the drone is making.
    - If `A` is 0 but `P` is high, your `I-term` is working hard to fight an imbalance (CG or bent prop).
    - If `G` is oscillating (+5, -5, +5...), your Rate P is too high.

---

## 6. Outdoor Flight Testing (With Props)

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
