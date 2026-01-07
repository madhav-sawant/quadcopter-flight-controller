# ESP32 Quadcopter Flight Controller

A custom flight controller firmware for ESP32-based quadcopters. This was my final year engineering project - built completely from scratch to understand how drones actually fly. No external flight controller libraries, just bare metal PID control.

## About This Project

I started this because I wanted to understand what's happening inside commercial flight controllers like Betaflight (which has 200,000+ lines of code). This codebase is around 3,000 lines - simple enough to read and understand, but complete enough to actually fly.

**What it does:**
- Cascaded PID control (angle loop + rate loop) running at 500Hz
- MPU6050 IMU with complementary filter for attitude estimation
- PPM receiver input (8 channels)
- Blackbox logging to RAM with CSV export
- Web interface for live PID tuning
- All settings saved to flash memory

## Hardware Setup

I used the following components:

| Part | What I Used |
|------|-------------|
| Controller | ESP32 DevKitC |
| IMU | MPU6050 |
| ESCs | 30A PWM (generic) |
| Frame | F450 clone |
| Motors | 1400KV brushless |
| Props | 8045 |
| Battery | 3S 2200mAh LiPo |
| Receiver | FlySky FS-iA6B (PPM mode) |

### Wiring

| Function | ESP32 Pin |
|----------|-----------|
| Motor 1 (Rear Right) | GPIO 25 |
| Motor 2 (Front Right) | GPIO 26 |
| Motor 3 (Rear Left) | GPIO 27 |
| Motor 4 (Front Left) | GPIO 32 |
| Receiver PPM | GPIO 4 |
| MPU6050 SDA | GPIO 21 |
| MPU6050 SCL | GPIO 22 |
| LED | GPIO 2 |
| Emergency Stop | GPIO 0 (Boot button) |

## Project Structure

```
Firmware/
├── src/
│   └── main.c              # Main control loop (runs at 500Hz)
├── lib/
│   ├── adc/                # Battery voltage reading
│   ├── blackbox/           # Flight data logging
│   ├── config/             # PID storage (NVS)
│   ├── imu/                # MPU6050 driver + sensor fusion
│   ├── mixer/              # Motor mixing for X-quad
│   ├── pid/                # PID controller
│   ├── pwm/                # ESC output
│   ├── rate_control/       # Rate PID wrapper
│   ├── rx/                 # PPM decoder
│   ├── telemetry/          # nRF24L01 transmitter
│   └── webserver/          # WiFi tuning interface
├── ground_station/         # ESP8266 receiver (separate project)
│   ├── ground_station.ino
│   └── README.md
├── analysis/               # Control system analysis (Python)
│   └── paper/              # LaTeX files for the paper
├── docs/
│   ├── Firmware_Documentation.md
│   └── BLACKBOX_EXPLAINED.md
└── platformio.ini
```


## How to Build

I used PlatformIO. Install VS Code + PlatformIO extension, then:

```bash
# Build
pio run

# Upload to ESP32
pio run --target upload

# Serial monitor
pio device monitor -b 115200
```

## First Time Setup

1. Place the quad on a flat surface
2. Power on and wait 1-2 seconds (gyro calibration runs automatically)
3. Connect to WiFi: `Drone_AP` (password: `12345678`)
4. Open `192.168.4.1` in browser
5. Click "Calibrate Accelerometer" (keep it level)
6. Check that receiver channels are working in the live view

## PID Values

These worked for my F450 build (1400KV motors, 8045 props, ~880g AUW):

| Controller | P | I | D |
|------------|---|---|---|
| Rate (Roll/Pitch) | 0.8 | 0.15 | 0.008 |
| Rate (Yaw) | 1.5 | 0.3 | 0.0 |
| Angle (Roll/Pitch) | 3.5 | 0.05 | 0.0 |

Your values will probably be different. Start with Rate P, then tune D, then I. Use the blackbox data to see what's happening.

## Control Architecture

```
Stick Input → Angle PID → Rate PID → Mixer → Motors
     ↑            ↑           ↑
     └────────────┴───────────┘
              IMU Feedback
```

The angle loop runs the outer control (stick → desired angle → rate setpoint) and the rate loop runs the inner control (rate setpoint → motor commands). Both run at 500Hz.

## Blackbox

The blackbox records flight data to RAM at ~80Hz. After landing, download the CSV from `http://192.168.4.1/blackbox/download`. I used Python scripts in `analysis/` to plot the data.

Fields recorded: timestamp, angles, gyro rates, setpoints, motor outputs, PID terms, battery voltage

## Safety Notes

- **Remove props during bench testing** - seriously, I broke 3 props learning this
- Keep throttle limit at 1400 until you're confident in the tune
- The boot button (GPIO 0) is emergency stop - press it to cut motors
- If angle goes over 60° or gyro rate over 500°/s, it auto-disarms

## What I Learned

Building this taught me a lot about:
- Real-time embedded systems (task timing, ISRs, watchdogs)
- Control theory in practice (PID tuning is harder than textbooks make it seem)
- Sensor fusion (why accelerometers are noisy on quads)
- How vibrations mess everything up

See `docs/Firmware_Documentation.md` for detailed code explanations.

## Acknowledgments

- Prof. Amol Sutar for guidance
- ESP-IDF documentation
- Various open source flight controller projects for inspiration
- Lots of YouTube videos on PID tuning

---

*This was made for my B.E. project at Finolex Academy of Management and Technology, Ratnagiri (2024-25).*
