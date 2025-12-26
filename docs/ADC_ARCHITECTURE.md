# ADC Battery Monitoring Architecture

This document explains the architecture of the battery monitoring system integrated into the flight controller.

## 1. Overview

The system monitors the LiPo battery voltage to prevent over-discharge, which can damage the battery and cause a crash.

*   **Threshold**: 9.9V (Critical Low)
*   **Action**: Safety Stop (Motors to Min Throttle, LED ON)
*   **Hardware**: ESP32 ADC1 Channel 7 (GPIO 35 usually, but depends on board).

## 2. Hardware Setup (Voltage Divider)

The ESP32 ADC can only measure up to ~3.3V (with attenuation). A 3S LiPo battery is ~12.6V. A voltage divider is used to scale the voltage down.

```mermaid
graph TD
    Battery[Battery + (12.6V)] --> R1[Resistor R1]
    R1 --> Node[ADC Pin]
    Node --> R2[Resistor R2]
    R2 --> GND[Ground]
```

*   **Formula**: $V_{adc} = V_{batt} \times \frac{R2}{R1 + R2}$
*   **Scaling Factor**: The code uses a scaling factor (`VOLTAGE_SCALE_MV` / `VOLTAGE_SCALE_DIV`) to convert the ADC reading back to the actual battery voltage.

## 3. Software Logic

### `adc.c` (Driver)
*   **`adc_init()`**: Configures ADC width (12-bit) and attenuation (11dB) to measure up to ~3.3V.
*   **`adc_read_battery_voltg()`**:
    1.  Reads raw ADC value (0-4095).
    2.  Converts to pin voltage (mV).
    3.  Applies scaling factor to get Battery Voltage (mV).

### `main.c` (Integration)
The monitoring logic is embedded in the main control loop to ensure safety at all times.

#### Safety Logic Flow
1.  **Read**: Get current voltage.
2.  **Debounce**:
    *   Voltage fluctuates due to motor load.
    *   We only trigger if voltage is low for **10 consecutive checks**.
    *   This prevents false alarms during momentary voltage sags.
3.  **Trigger**:
    *   Set `safety_stop = true`.
    *   Turn ON Status LED (Pin 2).
    *   **Force Motors to 1000µs** (Stop).
    *   Lock system (prevent further ramp-up).

## 4. Integration with PWM Ramp

The ramp-up sequence in `main.c` is a loop that increases motor speed.
*   **Problem**: If we just used `delay()`, we wouldn't check the battery for 30 seconds.
*   **Solution**: Inside the ramp loop, we call `check_battery_status()` every step.
    ```c
    for (int step = 0; step < total_steps; step++) {
        if (check_battery_status()) {
            break; // Stop immediately!
        }
        // ... increase speed ...
    }
    ```

This ensures that if the battery dies mid-flight (or mid-test), the motors cut off immediately.

## 5. Design Choice: Why Millivolts (mV)?

You will notice we use `uint16_t` to store voltage in **millivolts** (e.g., `11400` for 11.4V) rather than using `float` (e.g., `11.4`) or integer Volts (`11`).

### Reasons:
1.  **Precision**:
    *   Integer Volts (`11`) is too imprecise. We need to know the difference between `11.4V` (Charged) and `9.9V` (Empty).
    *   Millivolts gives us 3 decimal places of precision without using decimals.
2.  **Performance (Speed)**:
    *   Microcontrollers like the ESP32 can process Integers (`int`, `uint32_t`) much faster than Floating Point numbers (`float`, `double`).
    *   Floating point math requires complex CPU instructions or software emulation, which is slower.
3.  **Code Size**:
    *   Using `float` often pulls in large software libraries, increasing the firmware size.
    *   Integer math is native and lightweight.
4.  **Exactness**:
    *   Floats can have rounding errors (e.g., `11.4` might be stored as `11.4000001`). Integers are exact.

## 6. Recent Changes & Calibration

### Calibration
To ensure safety, we intentionally calibrated the ADC to read slightly **lower** than the actual voltage.
*   **Offset**: We subtract `180mV` (`VOLTAGE_OFFSET_MV`) from the reading.
*   **Goal**: Ensure the flight controller "thinks" the battery is lower than it is. This provides a safety margin of ~150-200mV.
    *   *Actual*: 11.40V
    *   *Reading*: ~11.22V
    *   *Result*: We land earlier, protecting the battery.

### Recovery Logic
We added a "Latching with Recovery" mechanism:
*   **Latch**: If voltage drops < 9.9V, the system locks (Safety Stop).
*   **Recovery**: If a fresh battery is connected (> 10.1V) and stays good for a short time, the system automatically unlocks.
*   **Why?** This prevents the drone from re-arming if the voltage sags momentarily under load, but allows you to swap batteries without rebooting.
