# RC Channel Mapping & Rates Check

## 1. Physical Channel Order
*   **Standard FlySky/IBUS PPM Order**: AETR or TAER?
*   **Code Implementation** (`main.c` lines 155-158):
    *   Channel 0 -> `rx_roll` (Aileron)
    *   Channel 1 -> `rx_pitch` (Elevator)
    *   Channel 2 -> `rx_thr` (Throttle)
    *   Channel 3 -> `rx_yaw` (Rudder)
*   **Status**: This assumes **AETR** (Aileron, Elevator, Throttle, Rudder) mapping.
    *   *Verify*: If your transmitter is set to TAER (Throttle first), your Roll stick will control Throttle!
    *   *Check*: On the bench (NO PROPS), raise throttle. If only one motor pair spins or pitch changes, your mapping is wrong.

## 2. Low Rate Sensitivity
*   **Code**: `MAX_RATE_DPS = 100.0f`
*   **Calculation**:
    *   Full stick = 500us deviation.
    *   Target Rate = (500 / 500) * 100 = **100 deg/s**.
*   **Effect**:
    *   To rotate 90 degrees, you must hold full stick for almost 1 full second.
    *   **Pros**: Extremely safe for first hover. Prevents panic flips.
    *   **Cons**: Drone will feel "heavy" and sluggish. You might struggle to fight strong wind or drift.
*   **Recommendation**: Keep 100dps for the very first "does it fly?" test. Once stable, increase to 300dps immediately.

## 3. Deadband
*   **Value**: `RC_DEADBAND_US 20`
*   **Status**: Standard. 20us ignores small stick jitters (1480-1520 is considered center).

## Summary
The RC Logic is sound, provided your Transmitter uses **AETR** channel order.
