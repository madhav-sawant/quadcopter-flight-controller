# PID Tuning Log - ESP32 Quadcopter

**Date:** December 28, 2024  
**Duration:** ~3.5 hours  
**Platform:** F450 Frame + 1400KV Motors + 8045 Props  
**Flight Controller:** ESP32-based custom firmware

---

## Tuning Methodology

### Method Used: **Manual Empirical Tuning with Blackbox Analysis**

This tuning approach is also known as:
- **Iterative/Empirical Tuning**
- **Trial-and-Error Method with Data Logging**
- **Blackbox-Assisted Manual Tuning**

Unlike automated methods (Ziegler-Nichols, Cohen-Coon, Auto-tune), this approach involves:
1. Making small adjustments to PID gains
2. Flying the aircraft and logging telemetry (blackbox data)
3. Analyzing the logged data to understand system behavior
4. Identifying issues from data patterns
5. Adjusting gains based on analysis
6. Repeating until stable

### Why This Method?

| Method | Pros | Cons |
|--------|------|------|
| **Manual Empirical** ✓ | Safe, gradual, learns system behavior | Time-consuming |
| Ziegler-Nichols | Mathematical, systematic | Requires oscillation to instability (dangerous for drones) |
| Auto-tune (Betaflight) | Automated | Requires specific firmware support |

For a custom flight controller without auto-tune, **manual empirical tuning with blackbox analysis** is the safest and most educational approach.

---

## Control Architecture

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                   CASCADED PID CONTROL                  │
                    └─────────────────────────────────────────────────────────┘
                    
  Stick Input    ┌──────────────┐   Rate      ┌──────────────┐   Motor
  (Angle Cmd) ──►│  ANGLE PID   │──Setpoint──►│  RATE PID    │──Output──► Motors
                 │ (Outer Loop) │             │ (Inner Loop) │
                 └──────────────┘             └──────────────┘
                        ▲                            ▲
                        │                            │
                   Roll/Pitch               Gyroscope (deg/s)
                   (degrees)                
```

**Outer Loop (Angle PID):** Converts desired angle to desired rotation rate  
**Inner Loop (Rate PID):** Converts desired rotation rate to motor commands

---

## Tuning Session Timeline

### Phase 1: Initial State Analysis

**Problem Observed:**  
- Drone exhibited violent oscillations immediately on arming
- Motors would spike even before liftoff
- Uncontrollable behavior

**Initial PID Values (too aggressive):**
| Parameter | Value |
|-----------|-------|
| Rate Roll/Pitch P | 0.25 |
| Rate Roll/Pitch I | 0.01 |
| Rate Roll/Pitch D | 0.30 |
| Angle Roll/Pitch P | 2.0 |
| Angle Roll/Pitch I | 0.5 |
| Rate Yaw P | 3.0 |

**Blackbox Analysis Findings:**
- `pid_roll` values showing large values (50-100+) even on ground
- Motors showing differential output before takeoff
- I-term was accumulating while drone sat on ground (windup!)

---

### Phase 2: I-Term Windup Fix

**Root Cause Identified:**  
The integral term was accumulating error while the drone was on the ground with props spinning. When throttle was increased, this accumulated I-term caused an immediate violent "kick."

**Code Changes Made:**

1. **Modified `pid_freeze_integral()` in `lib/pid/pid.c`:**
```c
void pid_freeze_integral(pid_controller_t *pid, bool freeze) {
  pid->integral_frozen = freeze;
  if (freeze) {
    pid->integral = 0.0f;  // RESET integral when frozen
  }
}
```
*Previously, this function only stopped accumulation but didn't reset the accumulated value.*

2. **Created `angle_control_freeze_integral()` in `lib/angle_control/angle_control.c`:**
```c
void angle_control_freeze_integral(bool freeze) {
  pid_freeze_integral(&pid_roll_angle, freeze);
  pid_freeze_integral(&pid_pitch_angle, freeze);
}
```
*The angle loop also needed integral freezing, not just the rate loop.*

3. **Updated `src/main.c` to freeze both loops:**
```c
bool freeze_integral = (throttle < 1200);  // Increased from 1150
rate_control_freeze_integral(freeze_integral);
angle_control_freeze_integral(freeze_integral);  // NEW
```

**Result:** ✅ Ground idle PID values now stay near 0

---

### Phase 3: Aggressive Gain Reduction

**Problem After Windup Fix:**  
Drone still exhibited violent oscillations and motor saturation immediately after takeoff.

**Analysis:**  
The original PID gains were tuned for a different motor/prop combination and were too aggressive for the 1400KV + 8045 setup.

**Changes Made:**

| Parameter | Before | After | Reason |
|-----------|--------|-------|--------|
| Rate Roll P | 0.25 | **0.08** | Reduce proportional response to errors |
| Rate Pitch P | 0.25 | **0.08** | Same as roll for symmetry |
| Rate Roll D | 0.30 | **0.12** | Reduce derivative response |
| Rate Pitch D | 0.30 | **0.12** | Same as roll |
| Angle Roll P | 2.0 | **1.0** | Gentler angle corrections |
| Angle Pitch P | 2.0 | **1.0** | Same as roll |
| Angle Roll I | 0.5 | **0.10** | Very low I to start |
| Angle Pitch I | 0.5 | **0.10** | Same as roll |
| Rate Yaw P | 3.0 | **2.0** | Smoother yaw |

**Result:** ✅ Drone can now lift off without violent oscillation

---

### Phase 4: Drift Correction

**Problem:**  
Drone would lift to ~1.5 meters but drift to one side.

**Blackbox Analysis:**
- Initial roll/pitch offset of ~1° (accelerometer calibration issue)
- Angle I-term too low to correct for steady-state drift

**Changes Made:**

| Parameter | Before | After | Reason |
|-----------|--------|-------|--------|
| Angle Roll I | 0.10 | **0.25** | Allow I-term to build up drift correction |
| Angle Pitch I | 0.10 | **0.25** | Same as roll |

**Result:** ✅ Initial attitude much improved (near 0° offset)

---

### Phase 5: Oscillation Damping (Final Stage)

**Problem:**  
Drone would hover for a few seconds, then oscillations would build up over time.

**Analysis:**  
The Rate D term was still too high, causing "derivative kick" - the controller was overreacting to rate changes and oscillating.

**Recommended Final Adjustment (not yet tested):**

| Parameter | Current | Recommended |
|-----------|---------|-------------|
| Rate Roll D | 0.12 | **0.08** |
| Rate Pitch D | 0.12 | **0.08** |

---

## Final PID Values (As of End of Session)

### Committed to Codebase

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Rate Roll P** | 0.08 | Conservative for stability |
| **Rate Roll I** | 0.01 | Minimal - most correction from outer loop |
| **Rate Roll D** | 0.08 | Reduced to dampen oscillations |
| **Rate Pitch P** | 0.08 | Same as roll |
| **Rate Pitch I** | 0.01 | Same as roll |
| **Rate Pitch D** | 0.08 | Same as roll |
| **Rate Yaw P** | 2.0 | Smooth yaw response |
| **Rate Yaw I** | 0.02 | Unchanged |
| **Rate Yaw D** | 0.15 | Unchanged |
| **Angle Roll P** | 1.0 | Gentle self-leveling |
| **Angle Roll I** | 0.25 | Steady-state drift correction |
| **Angle Roll D** | 0.0 | Not used |
| **Angle Pitch P** | 1.0 | Same as roll |
| **Angle Pitch I** | 0.25 | Same as roll |
| **Angle Pitch D** | 0.0 | Not used |

---

## Issues Fixed Summary

| Issue | Symptom | Root Cause | Solution |
|-------|---------|------------|----------|
| **I-term Windup** | Violent jerk on arming | Integral accumulating on ground | Reset integral when throttle low |
| **Motor Saturation** | Motors hitting limits | Rate P/D too high | Reduced Rate P from 0.25→0.08, D from 0.30→0.08 |
| **Aggressive Response** | Too "twitchy" | Angle P too high | Reduced Angle P from 2.0→1.0 |
| **Attitude Drift** | Drone tilts to one side | Low Angle I, accel offset | Increased Angle I from 0.10→0.25 |
| **Oscillation Buildup** | Wobbles increase over time | Rate D too high | Reduced Rate D from 0.12→0.08 |

---

## Tuning Progress Achieved

```
Progress: ████████████████░░░░ ~80% Complete

✅ I-term windup fixed
✅ Smooth liftoff achieved
✅ Initial attitude stable
✅ Holds hover for several seconds
⏳ Oscillation damping (one more adjustment needed)
❌ Extended stable hover
```

---

## Next Steps (When Resuming)

1. **Test with Rate D = 0.08** (already set in codebase defaults)
2. **Observe oscillation behavior**
3. If still oscillating:
   - Reduce Rate P further (try 0.06)
   - Or reduce Angle P (try 0.8)
4. If stable but sluggish:
   - Gradually increase Rate P (try 0.10)
   - Increase Angle P (try 1.2)

---

## Key Learnings

### PID Tuning Rules of Thumb

1. **Start LOW, increase gradually** - It's safer to have a "lazy" drone than an oscillating one
2. **Rate loop first, then Angle loop** - Inner loop must be stable before outer loop
3. **P causes oscillation** - If oscillating, reduce P
4. **D dampens oscillation** - But too much D causes high-frequency jitter
5. **I fixes drift** - But causes windup if not managed
6. **Always freeze I-term on ground** - Prevents windup

### Blackbox Analysis Tips

| Pattern | Meaning |
|---------|---------|
| PID output swinging ±100+ | Gains too high |
| Gyro rates alternating sign rapidly | Oscillation |
| Motors hitting 1100 or saturating | System fighting hard |
| Steady angle offset | Needs more I-term or accel calibration |
| Gradually increasing oscillation | D-term too high or P-term too high |

---

## Hardware Configuration Reference

| Component | Specification |
|-----------|---------------|
| Frame | F450 |
| Motors | 1400KV Brushless |
| Props | 8045 (8 inch, 4.5 pitch) |
| Battery | 3S LiPo |
| Flight Controller | ESP32 (custom) |
| IMU | MPU6050 (with DLPF) |
| Control Loop Rate | 250 Hz |
| Blackbox Log Rate | 50 Hz |

---

## Files Modified During Tuning

| File | Changes |
|------|---------|
| `lib/config/config.c` | Default PID values updated |
| `lib/pid/pid.c` | I-term reset on freeze |
| `lib/angle_control/angle_control.c` | Added freeze function |
| `lib/angle_control/angle_control.h` | Declared freeze function |
| `src/main.c` | Call angle freeze, threshold 1200 |

---

## Conclusion

This tuning session successfully brought the quadcopter from **completely uncontrollable** to **~80% stable hover**. The key breakthrough was identifying and fixing the I-term windup issue, followed by systematic reduction of overly aggressive gains.

The **manual empirical tuning method** with blackbox analysis is time-consuming but provides deep understanding of the control system behavior. Each iteration provided valuable data that informed the next adjustment.

**Estimated remaining work:** 1-2 more test flights to achieve fully stable hover.

---

*Document created: December 28, 2024*  
*Last updated: December 28, 2024*
