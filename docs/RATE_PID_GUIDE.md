# Rate PID Controller - Usage Guide

## Overview

This document explains how to use and validate the **Inner (Rate) PID Control Loop** for the quadrotor flight controller.

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

## Tuning the PID Gains

Gains are defined in `lib/rate_control/rate_control.h`:

```c
// Roll Rate PID
#define RATE_ROLL_KP 1.0f   // Proportional
#define RATE_ROLL_KI 0.0f   // Integral (add after P+D tuned)
#define RATE_ROLL_KD 0.02f  // Derivative

// Pitch Rate PID (same as roll for symmetric frame)
#define RATE_PITCH_KP 1.0f
#define RATE_PITCH_KI 0.0f
#define RATE_PITCH_KD 0.02f

// Yaw Rate PID (lower gains, yaw is slower)
#define RATE_YAW_KP 2.0f
#define RATE_YAW_KI 0.0f
#define RATE_YAW_KD 0.0f
```

### Tuning Steps

1. **Start with P only** (Ki=0, Kd=0)
   - Increase Kp until response is snappy but not oscillating

2. **Add D for damping**
   - Small Kd reduces overshoot
   - Too much Kd = noise amplification

3. **Add I to eliminate steady-state error**
   - Only if needed (usually small)
   - Watch for integral windup

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

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| PID output always 0 | Gyro not calibrated | Keep still during calibration |
| Noisy output | Vibration / bad mounting | Check IMU mounting, add damping |
| Output same sign as gyro | Wrong sign in PID | Check error calculation |
| Output saturates (500/-500) | Gains too high | Reduce Kp |
| Slow response | Gains too low | Increase Kp carefully |

---

## Next Steps

After successful validation:

1. ✅ Rate (Inner) Loop - **DONE**
2. ⬜ Motor Mixer (PID → individual motor PWM)
3. ⬜ Angle (Outer) Loop (desired angle → desired rate)
4. ⬜ RC Input Integration
5. ⬜ Arming/Safety Logic
