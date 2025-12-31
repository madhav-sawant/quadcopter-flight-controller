# System Configuration Review & Recommendations

This document highlights findings from a deep-dive code review of the current firmware configuration relative to the **F450 Frame + 1400KV Motors** setup.

## 1. Filter Tuning (CRITICAL)

### D-Term Low Pass Filter (`pid.c`)
*   **Current Value**: `D_TERM_LPF_ALPHA 0.15`
*   **Calculated Cutoff**: **~7 Hz** (at 250Hz loop)
*   **Issue**: This is **extremely low**. A 7Hz cutoff introduces significant phase lag. The D-term relies on reacting *instantly* to stop movements. With this lag, the D-term might actually *add* to oscillations rather than dampening them properly.
*   **Recommendation**: Increase Alpha to **0.40**.
    *   `Alpha 0.40` @ 250Hz ~= **27 Hz** Cutoff (Much better, still smooth).
    *   `Alpha 0.50` @ 250Hz ~= **40 Hz** Cutoff (Standard).

### Gyro Low Pass Filter (`imu.c`)
*   **Current Value**: `GYRO_LPF_ALPHA 0.40`
*   **Calculated Cutoff**: **~26 Hz**
*   **Status**: This is conservative but acceptable for a large vibrant frame like F450. It effectively removes motor noise but makes the flight feel slightly "smooth/detached".
*   **Recommendation**: Keep for now. If the drone feels too "slushy" or disconnected, increase to `0.70` (~60Hz).

---

## 2. Control Limits

### Throttle & Mixer
*   **Tune Limit**: 1400us (`main.c`)
*   **Mixer Limit**: 1500us (`mixer.h`)
*   **Status**: **Safe**. The mixer allows slightly more overhead (1500) than the pilot can command (1400), giving the PID controller 100us of headroom to make corrections even at full stick.

### Rates (Stick Sensitivity)
*   **Max Rate**: 100 deg/s (`MAX_RATE_DPS` in `main.c`)
*   **Status**: **Very Slow**. A full stick deflection will rotate the drone very slowly.
*   **Recommendation**: Perfect for initial stable hover testing. For actual flying, this will need to be increased to ~300-400 deg/s later.

---

## 3. PID Configuration (`config.c`)

### Output Limit
*   **Current**: 110.0f (We just changed this from 400).
*   **Status**: **Excellent**. Matches the limited throttle range (1100-1400).

### I-Term Limit
*   **Current**: 100.0f
*   **Status**: **High Risk**.
*   **Risk**: The I-term limit (100) is almost equal to the total Output limit (110). If the drone sits unlevel on the ground for a few seconds, the I-term can saturate 90% of the available power.
*   **Recommendation**: Reduce `rate_integral_limit` to **50.0f**. This allows the I-term to fix drifts but prevents it from "taking over" the entire control authority.

---

## Summary of Proposed Fixes

1.  **`lib/pid/pid.c`**: Change `D_TERM_LPF_ALPHA` from `0.15` to **0.42** (~30Hz).
2.  **`lib/config/config.c`**: Change `rate_integral_limit` from `100.0f` to **50.0f**.

Do you want me to apply these fixes?
