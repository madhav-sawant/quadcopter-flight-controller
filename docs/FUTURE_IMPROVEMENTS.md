# Future Improvements & Logic Ideas

This document stores architectural ideas and logic to be implemented after the core PID tuning is complete.

---

## 1. Active Angle Limiting & Recovery System

**Goal:** Prevent the drone from ever reaching the 60° crash cutoff angle by actively limiting and recovering attitude before it gets there.

### The Logic
1.  **Command Limit (45°)**:
    *   The flight controller should ignore any stick input that requests an angle greater than 45 degrees.
    *   `constrain(desired_angle, -45, 45)`

2.  **Recovery Zone (45° - 60°)**:
    *   If external forces (wind, momentum) push the drone past 45° (e.g., into the 45-59° range):
    *   **Action:** The controller must override normal PID behavior to aggressively correct the angle back to within the safe zone (<45°).
    *   This is a "Soft Limit" that acts before the "Hard Limit" (Crash Cutoff).

3.  **Crash Cutoff (60°)**:
    *   Existing logic: If angle > 60°, disarm motors immediately.
    *   With the new logic, this becomes a fail-safe that should rarely be triggered.

### Implementation Strategy (Draft)
```c
// Pseudo-code for future implementation in angle_control.c

#define MAX_SAFE_ANGLE 45.0f
#define CRASH_ANGLE 60.0f

void angle_safety_check(float current_roll, float current_pitch) {
    // 1. Clamp Desired Setpoint
    // Ensure pilot cannot ask for > 45 degrees
    
    // 2. Active Recovery
    if (abs(current_roll) > MAX_SAFE_ANGLE && abs(current_roll) < CRASH_ANGLE) {
        // We are in the danger zone!
        // Force a strong corrective setpoint regardless of stick input
        // e.g., set_roll_rate = -MAX_RATE * sign(current_roll);
    }
}
```

---

## 2. Throttle Scaling / Tilt Compensation (Future)
*   As the drone tilts, it loses vertical lift component.
*   Future logic: Automatically increase throttle slightly as tilt angle increases to maintain altitude ( `throttle / cos(tilt_angle)` ).

---

*Document created to preserve logic ideas for post-tuning implementation.*
