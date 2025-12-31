# Rate Mode Tuning Journey: F450 + 1400KV

This document details all the improvements made to achieve stable Rate Mode flight on the ESP32 Quadcopter Firmware.

---

## System Specifications
- **Frame**: F450 (450mm)
- **Motors**: 1400KV brushless
- **Props**: 8045 (high mechanical gain)
- **Battery**: 3S LiPo (12.6V full)
- **Control Loop**: 250Hz

---

## Problem 1: PID Output Limit Too High

### Original Issue:
- `rate_output_limit = 400`
- Throttle range during tuning: 1100 - 1400 = **300us**
- PID could command ±400, but only 300 was available
- Result: **Motor saturation** (hitting min/max constantly)

### Fix:
```c
// Before
sys_cfg.rate_output_limit = 400.0f;

// After
sys_cfg.rate_output_limit = 110.0f;
```

### Reasoning:
- With 300us range, PID should only use ~half for corrections
- 110us allows control while leaving headroom for all 3 axes
- Formula: `Limit ≈ Range / 3` = `300 / 3 ≈ 100-120`

---

## Problem 2: Integral Windup Risk

### Original Issue:
- `rate_integral_limit = 100`
- Output limit = 110
- I-term could consume **90%** of available authority!
- Result: Ground windup, aggressive takeoff behavior

### Fix:
```c
// Before
sys_cfg.rate_integral_limit = 100.0f;

// After
sys_cfg.rate_integral_limit = 50.0f;
```

### Reasoning:
- I-term should handle steady-state errors (weight imbalance, prop thrust differences)
- It should NOT be the primary correction source
- Rule: `I_limit ≤ Output_limit / 2`
- 50 / 110 = ~45% max I-term authority

---

## Problem 3: D-Term Phase Lag

### Original Issue:
- `D_TERM_LPF_ALPHA = 0.15`
- At 250Hz loop, this gives cutoff ≈ **7Hz**
- D-term was reacting too slowly to stop oscillations
- Ironically, the "filter" was causing more oscillation by introducing lag

### Fix:
```c
// Before (lib/pid/pid.c)
#define D_TERM_LPF_ALPHA 0.15f  // ~7Hz cutoff - TOO SLOW

// After
#define D_TERM_LPF_ALPHA 0.42f  // ~30Hz cutoff - PROPER
```

### Cutoff Frequency Formula:
```
f_cutoff = (alpha * loop_rate) / (2 * π * (1 - alpha))
```

| Alpha | Cutoff @ 250Hz | Use Case |
|-------|---------------|----------|
| 0.15 | ~7 Hz | Too slow, adds lag |
| 0.30 | ~17 Hz | Very smooth, some lag |
| **0.42** | **~30 Hz** | **Balanced** |
| 0.50 | ~40 Hz | Industry standard |
| 0.70 | ~93 Hz | Aggressive, noisy |

---

## Problem 4: Pitch Sign Inversion

### Original Issue:
- Standard Quad-X mixer formula caused **pitch flip on takeoff**
- Motor wiring was reversed relative to standard convention

### Fix (lib/mixer/mixer.c):
```c
// HARDWARE ADAPTATION:
// Pitch sign is INVERTED relative to standard Quad-X
int32_t m1 = t - roll + pitch - yaw; // Rear Right
int32_t m2 = t - roll - pitch + yaw; // Front Right
int32_t m3 = t + roll + pitch + yaw; // Rear Left
int32_t m4 = t + roll - pitch - yaw; // Front Left
```

### Reasoning:
- Instead of re-wiring motors, we inverted the pitch term in software
- `+pitch` on rear motors (M1, M3) instead of standard `-pitch`

---

## Problem 5: Battery Cutoff During Flight

### Original Issue:
- Critical battery check triggered mid-flight
- Voltage sag during throttle punch caused false disarms

### Fix (src/main.c):
```c
// DISABLED: Battery auto-disarm removed for testing
// WARNING: Monitor battery voltage manually!
// static int crit_bat_counter = 0;
// if (system_armed && debug_vbat <= 9000) { ...
```

### Status:
- Low battery **LED warning** still active (blinks below 10.5V)
- Auto-disarm **disabled** for testing
- Pilot must monitor voltage manually

---

## Final Rate PID Values (Proven Stable)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Roll P | 0.40 | Conservative for high-KV |
| Roll I | 0.20 | Corrects steady drift |
| Roll D | 0.03 | Light damping |
| Pitch P | 0.40 | Symmetric with roll |
| Pitch I | 0.20 | Symmetric with roll |
| Pitch D | 0.03 | Symmetric with roll |
| Yaw P | 2.50 | Higher (yaw is weaker) |
| Yaw I | 2.50 | Compensates for prop torque |
| Yaw D | 0.00 | Usually not needed |
| Output Limit | 110 | Matches throttle range |
| I-term Limit | 50 | Prevents windup |
| D-term Alpha | 0.42 | ~30Hz filter cutoff |

---

## Flight Test Results

### Before Fixes:
- ❌ Flip on takeoff (pitch inversion)
- ❌ Oscillations (D-term lag)
- ❌ Random disarms (battery cutoff)
- ❌ Aggressive ground behavior (I-term windup)

### After Fixes:
- ✅ **Clean liftoff**
- ✅ **Stable hover** (no oscillations)
- ✅ **Smooth control response**
- ✅ **No unexpected disarms**

---

## Key Lessons Learned

1. **PID limits must match your hardware constraints** (throttle range)
2. **D-term filter must be fast enough** to actually dampen oscillations
3. **I-term is powerful** - limit it to prevent takeover
4. **Test without props first** - verify motor response to tilt
5. **Blackbox is essential** - data-driven tuning beats guessing

---

*Document created: 2024-12-31*
*System: ESP32 Quadcopter Firmware*
