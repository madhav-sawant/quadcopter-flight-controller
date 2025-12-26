# First Bench Test Results - Rate Mode Validation
**Date:** 2025-12-24
**Test Type:** Hand-held Bench Test (Props OFF)
**Mode:** Rate Mode (Acro)

## 1. Raw Log Data
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

## 2. Analysis

### Scenario A: Pitch Up (Nose Up) Movement
**Log Line:**
`G: -0.9 31.3 -4.3 | P: 0.9 -32.5 8.6 | M: 1076 1124 1140 1060`

*   **Gyro Input (`G`)**: Pitch rate is **+31.3 deg/s**. This indicates the nose is rotating UP.
*   **PID Output (`P`)**: Pitch PID output is **-32.5**. The controller is trying to oppose the motion by pushing the nose DOWN.
*   **Motor Output (`M`)**:
    *   **Front Motors (M1, M4)**: 1076, 1060. These are *lower* than the idle speed (1100) or base throttle. The front motors are slowing down.
    *   **Rear Motors (M2, M3)**: 1124, 1140. These are *higher* than the idle speed. The rear motors are speeding up.
*   **Physics Check**: Increasing rear motor speed and decreasing front motor speed creates a torque that pushes the nose down.
*   **Result**: **CORRECT**. The controller is correctly fighting the uncommanded rotation.

### Scenario B: Static / Level (Wait)
**Log Line:**
`G: -0.0 0.1 0.1 | P: 0.0 -0.7 -0.2 | M: 1100 1100 1100 1100`

*   **Gyro Input (`G`)**: Rates are near zero (~0.1 deg/s).
*   **PID Output (`P`)**: PID corrections are near zero.
*   **Motor Output (`M`)**: All motors are at **1100 (Idle)**.
*   **Result**: **CORRECT**. In Rate Mode, if the drone is not rotating (even if it is tilted at an angle), the controller should not output any correction. It only resists *change* in angle (velocity), not the angle itself.

## 3. Conclusion
*   **Directionality**: The Motor Mixer and PID signs are correct. The drone resists movement in the correct direction.
*   **Noise/Jitter**: The gyro data looks clean when static (0.0 to 0.1 deg/s noise floor).
*   **Tuning**: The P-gain appears to be in a safe starting range (Output ~30 for Input ~30). No violent oscillations were observed in the logs.

**Status:** The firmware logic is validated. The system is ready for a cautious maiden flight (hover test) in a safe environment.
