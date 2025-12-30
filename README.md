# ESP32 Quadcopter Flight Controller Firmware

A custom flight controller firmware for ESP32-based quadcopters, built from scratch using the ESP-IDF framework. Features cascaded PID control, real-time IMU processing, blackbox data logging, and web-based tuning interface.

## ✨ Features

- **Cascaded PID Control**: Dual-loop control with outer angle controller and inner rate controller running at 500Hz
- **MPU6050 IMU Integration**: Gyroscope and accelerometer fusion using complementary filter
- **PPM Receiver Support**: 8-channel PPM input with failsafe handling
- **Blackbox Logging**: In-memory flight data recorder with CSV export via web interface
- **Web-Based Tuning**: Live PID parameter adjustment without reflashing
- **Quad-X Mixer**: Standard quadcopter motor mixing for X-configuration
- **Safety Features**: 
  - Arm/disarm via RC switch
  - Emergency stop on boot button
  - Throttle limiting for safe tuning
  - Recovery mode for extreme angles (>45°)
- **NVS Persistence**: PID gains and IMU calibration saved to flash

## 🛠️ Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Microcontroller | ESP32 DevKitC |
| IMU | MPU6050 (I2C) |
| ESCs | 4x PWM ESCs (OneShot125 compatible) |
| Receiver | PPM sum output compatible |
| Frame | F450 or similar Quad-X |
| Motors | Brushless BLDC (1400kv with 8045 props) |
| Battery | 3S/4S LiPo |

### Pin Configuration

| Function | GPIO |
|----------|------|
| Motor 1 (Rear Right) | 25 |
| Motor 2 (Front Right) | 26 |
| Motor 3 (Rear Left) | 27 |
| Motor 4 (Front Left) | 32 |
| PPM Receiver | 4 |
| I2C SDA (MPU6050) | 21 |
| I2C SCL (MPU6050) | 22 |
| Status LED | 2 |
| Emergency Button | 0 |

## 📁 Project Structure

```
├── src/
│   └── main.c              # Main entry, control loop (500Hz timer)
├── lib/
│   ├── angle_control/      # Outer loop angle PID controller
│   ├── rate_control/       # Inner loop rate PID controller  
│   ├── pid/                # Generic PID implementation with D-term filtering
│   ├── imu/                # MPU6050 driver, complementary filter, calibration
│   ├── mixer/              # Quad-X motor mixing
│   ├── pwm/                # ESC PWM output (1000-2000µs)
│   ├── rx/                 # PPM receiver input with ISR
│   ├── blackbox/           # Flight data logging (ring buffer)
│   ├── config/             # System configuration, NVS storage
│   ├── adc/                # Battery voltage monitoring
│   └── webserver/          # WiFi AP + HTTP server for tuning
├── platformio.ini          # PlatformIO build configuration
└── partitions.csv          # ESP32 partition table
```

## 🚀 Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- USB cable for flashing
- Serial monitor for debugging

### Build & Flash

```bash
# Clone the repository
git clone https://github.com/madhav-sawant/ESP32-Quadcopter-Firmware.git
cd ESP32-Quadcopter-Firmware

# Build the project
pio run

# Flash to ESP32
pio run --target upload

# Monitor serial output
pio device monitor -b 115200
```

### First-Time Setup

1. **Power on** with quad on a level surface
2. **Wait for gyro calibration** (~1 second, keep still)
3. **Connect to WiFi AP**: `Drone_AP` (password: `12345678`)
4. **Open browser**: Navigate to `192.168.4.1`
5. **Calibrate accelerometer** via web interface (keep level)
6. **Verify receiver** connection and channel mapping

## ⚙️ PID Tuning

### Current Default Values

| Controller | P | I | D |
|------------|---|---|---|
| Rate Roll/Pitch | 0.8 | 0.15 | 0.008 |
| Rate Yaw | 1.5 | 0.3 | 0.0 |
| Angle Roll/Pitch | 3.5 | 0.05 | 0.0 |

### Tuning Workflow

1. **Set throttle limit** to 1400 in config for safe testing
2. **Start with rate PIDs**: Tune P first, then D, finally I
3. **Move to angle PIDs**: Adjust P for response, I for drift
4. **Use blackbox data**: Export CSV and analyze with Python/Excel
5. **Iterate**: Each session, adjust one parameter at a time

## 📊 Blackbox Data

The blackbox records at 500Hz with the following fields:

| Field | Description |
|-------|-------------|
| timestamp_ms | Milliseconds since boot |
| roll/pitch/yaw | Current angles (degrees) |
| gyro_x/y/z | Angular rates (deg/s) |
| roll/pitch/yaw_setpoint | Target angles |
| throttle | Throttle input (µs) |
| motor1-4 | Motor outputs (µs) |
| pid_roll/pitch/yaw | PID outputs |
| vbat | Battery voltage (mV) |

Access via: `http://192.168.4.1/blackbox/download`

## 🔧 Control Architecture

```
        Angle Mode (Default)
        ┌──────────────────────────────────────────────────────────┐
        │                                                          │
RC Input ──► Angle Controller ──► Rate Controller ──► Mixer ──► Motors
  (deg)       (P, I, D)           (P, I, D)         (Quad-X)
               ▲                    ▲
               │                    │
               └── IMU Angle ◄──────┴── IMU Gyro
                   (Complementary Filter)
```

- **Control Loop Rate**: 500Hz (2ms)
- **IMU Sample Rate**: 500Hz
- **PWM Update Rate**: 500Hz (supports OneShot)

## ⚠️ Safety Notes

- **ALWAYS remove propellers** during bench testing
- Use **throttle limiter** (~1400µs) during initial tuning
- Test in a **large open area** away from people
- Verify **motor spin directions** before first flight
- Check **center of gravity** is centered
- Ensure **failsafe** returns throttle to minimum on signal loss

## 📜 License

This project is for educational and personal use.

## 🙏 Acknowledgments

- ESP-IDF framework by Espressif
- MPU6050 datasheets and community resources
- Various open-source flight controller projects for inspiration
