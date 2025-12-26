# Firmware Fix Log

## 2025-12-23: Rate PID & Modular Architecture Refactor
- **Implemented Rate PID Control**: Added inner loop PID controllers for Roll, Pitch, and Yaw rates running at 500Hz.
- **Implemented Motor Mixer**: Added Quad-X mixing logic with safety features (arming, idle throttle, output clamping).
- **Restored Modular Structure**: Separated code into `lib/` modules (`pid`, `pwm`, `adc`, `imu`, `mixer`, `rate_control`) for better maintainability.
- **Added Configuration Module**: Created `lib/config` to centralize tunable parameters (PID gains, safety limits) in a `system_config_t` structure.
- **Safety Features**:
    - Low battery disarm (< 10.5V).
    - Crash angle protection (> 60 deg tilt).
    - Emergency stop via Boot button.
- **Documentation**: Added `PID_TUNING_AND_USAGE.md` guide.

## 2025-12-23: Compilation Fixes
- **Fixed `lib/imu/imu.h`**: Removed stray character causing syntax error.
- **Fixed `src/main.c`**:
    - Removed premature closing brace in `app_main`.
    - Replaced undefined macros `LOW_BATTERY_THRESHOLD_MV` with `sys_cfg.low_bat_threshold`.
- **Verified Build**: Project compiles successfully with `pio run`.

This document tracks the critical fixes and improvements made to the quadcopter firmware.

## 1. Battery Latching & Recovery
*   **File**: `src/main.c` (PWM Demo)
*   **Issue**: The system would permanently lock (Safety Stop) if a low battery was detected, even if a fresh battery was connected afterwards.
*   **Fix**: Implemented a recovery mechanism.
    *   Added a `good_batt_counter`.
    *   If voltage > Threshold + 200mV for a sustained period, the system unlocks.
*   **Why**: To allow hot-swapping batteries without resetting the MCU.

## 2. Soft Watchdog Timeout (Input Loop)
*   **File**: `src/main.c` (PWM Demo)
*   **Issue**: The `while(1)` loop waiting for user input ('s' key) was spinning too fast, starving the IDLE task and triggering the Task Watchdog Timer (TWDT).
*   **Fix**: Added `vTaskDelay(pdMS_TO_TICKS(10))` inside the wait loop.
*   **Why**: To yield CPU time to the FreeRTOS scheduler and reset the watchdog.

## 3. ADC Calibration
*   **File**: `lib/adc/adc.h`
*   **Issue**: The battery voltage reading was inaccurate and didn't provide the desired safety margin.
*   **Fix**:
    *   Updated `VOLTAGE_SCALE_MV` to `41624`.
    *   Added `VOLTAGE_OFFSET_MV` of `180`.
*   **Why**: To ensure the monitor reads ~150-200mV *lower* than actual voltage, ensuring a safe landing before the battery is critically low.

## 4. Soft Watchdog Timeout (IMU Calibration)
*   **File**: `lib/imu/imu.c`
*   **Issue**: The `imu_calibrate_gyro` function used a blocking busy-wait loop (`for(volatile int j...)`) for delay. This caused a WDT timeout during the 2-second calibration phase.
*   **Fix**: Replaced the busy-wait loop with `vTaskDelay(pdMS_TO_TICKS(2))`.
*   **Why**: To allow the IDLE task to run during the delay, preventing WDT reset failure.

## 5. Boot Loop (SW_CPU_RESET)
*   **File**: `lib/imu/imu.c`
*   **Issue**: `imu_init` used `ESP_ERROR_CHECK` for I2C writes. If the sensor was disconnected or failed to ACK, the ESP32 would abort and reset continuously.
*   **Fix**: Replaced `ESP_ERROR_CHECK` with explicit error checking (`if (ret != ESP_OK) return ret;`).
*   **Why**: To allow the application to handle initialization failures gracefully (e.g., by blinking an LED error code) instead of crashing.

## 6. xTaskDelayUntil Assertion Failure
*   **File**: `src/main.c` (IMU Test Mode)
*   **Issue**: After gyro calibration, the system crashed with `assert failed: xTaskDelayUntil tasks.c:1499 (( xTimeIncrement > 0U ))`. The assertion triggers when `vTaskDelayUntil` is asked to delay for 0 or negative ticks.
*   **Root Cause**: `pdMS_TO_TICKS(2)` returns **0** because FreeRTOS default tick rate is 100Hz (1 tick = 10ms). Requesting a 2ms delay rounds down to 0 ticks, which is invalid for `vTaskDelayUntil`.
*   **Fix**: 
    *   Replaced `vTaskDelayUntil(&last_wake_time, period_ticks)` with `vTaskDelay(1)` to guarantee at least 1 tick delay.
    *   Updated `imu_read(0.010f)` so the dt parameter matches the actual 10ms tick period.
    *   Changed print counter from 50 to 10 (10 × 10ms = 100ms print interval).
*   **Why**: At 100Hz tick rate, sub-10ms timing is not possible with `vTaskDelay`/`vTaskDelayUntil`. Using `vTaskDelay(1)` ensures the loop always yields to the scheduler with a valid non-zero delay.
