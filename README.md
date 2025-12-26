# Quadcopter Flight Controller Firmware (ESP32)

This project is a custom flight controller firmware for an ESP32-based quadcopter. It uses the ESP-IDF framework (via PlatformIO) and features a modular architecture with register-level MPU6050 configuration.

## Current Status: IMU Test Mode
The firmware is currently configured to **isolate and test the MPU6050 IMU**.
- **Enabled**: MPU6050 Driver, Complementary Filter, Serial Logging.
- **Disabled**: PWM (Motors), ADC (Battery), PID Control Loop.

## Hardware Setup
1.  **ESP32**: Standard Dev Module.
2.  **MPU6050**: Connect via I2C.
    *   **VCC** -> 3.3V
    *   **GND** -> GND
    *   **SDA** -> GPIO 21
    *   **SCL** -> GPIO 22
3.  **Status LED**: GPIO 2 (Built-in LED on most boards).

## How to Run

### 1. Build and Upload
Connect your ESP32 via USB and run the following command in the terminal:
```bash
pio run -t upload
```

### 2. Monitor Output
Open the serial monitor to view the sensor data:
```bash
pio device monitor
```
*   **Baud Rate**: 115200 (Default)

### 3. Calibration
*   When the board resets, the LED (GPIO 2) will turn **ON**.
*   **Keep the board perfectly still** on a flat surface for about 2-3 seconds.
*   The LED will turn **OFF** once calibration is complete.

### 4. Verify Data
You should see output similar to this every 100ms:
```text
R:   0.50 | P:  -1.20 | Ax:  0.01 Ay:  0.02 Az:  0.98 | Gx:   0.1 Gy:  -0.2 Gz:   0.0
```
*   **R (Roll)**: Angle in degrees (Tilt Left/Right).
*   **P (Pitch)**: Angle in degrees (Tilt Forward/Back).
*   **Ax/Ay/Az**: Accelerometer data in g (Az should be ~1.00 when flat).
*   **Gx/Gy/Gz**: Gyroscope data in deg/s (Should be near 0.0 when still).

## Troubleshooting
*   **"IMU Init Failed! Halting."**:
    *   Check your wiring (SDA/SCL swapped?).
    *   Check if MPU6050 has power.
    *   The LED will blink rapidly if initialization fails.
