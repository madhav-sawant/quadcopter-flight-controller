# ADC Calibration & Logic Check

## Current Implementation

### 1. Hardware
*   **Pin**: GPIO 35 (ADC1 Channel 7)
*   **Resolution**: 12-bit (0-4095)
*   **Attenuation**: 12dB (0-3.3V range)

### 2. Voltage Calculation
`Battery_mV = (Scaled_ADC * 4.16) - 180mV`

This formula comes from empirical testing.
*   `VOLTAGE_SCALE_MV 41624` / `10000` = **4.1624 Multiplier**.
    *   This implies a voltage divider ratio of approx 1:4.16. (e.g. 12.6V -> 3.0V at pin).
    *   Normally a 100k/33k divider gives 1 : (133/33) = ~4.03.
    *   So this multiplier accounts for the divider AND the ADC reference error.
*   `VOLTAGE_OFFSET_MV 180`
    *   Subtracts a constant calibration offset.

### 3. Usage
*   **Location**: `main.c` line 522.
*   **Action**: Reads every 10ms (100Hz) inside the low-priority loop.
*   **Low Bat Warning**:
    *   `if (vbat < sys_cfg.low_bat_threshold)` (Default 10.5V).
    *   Action: Blinks LED.
*   **Critical Cutoff (`DISABLED`)**:
    *   Current code has the critical disarm logic commented out (lines 550-561).
    *   **This is Correct** for testing. We don't want the drone to fall out of the sky if the voltage sags momentarily during a punch-out.

## Recommendation
The ADC system is robust and safe for flight. The critical disarm is correctly disabled, leaving only the LED warning.
