---
description: Flight test plan for PID tuning validation and stability verification
---

# 🚁 Flight Test Plan - December 30, 2025

## 📋 Pre-Flight Checklist

### Hardware Check
- [ ] Battery fully charged (3S, >12.0V)
- [ ] Props inspected (no damage, balanced)
- [ ] All screws tight (motors, arms, FC mount)
- [ ] Battery secured at center
- [ ] Transmitter charged, bound

### Software Prep
// turbo
1. Source ESP-IDF environment
   ```bash
   source ~/esp/esp-idf/export.sh
   ```
// turbo
2. Build firmware
   ```bash
   cd ~/Videos/Project_Firmware1.1-154c313ca87655c3f5d4cd465e96744cc6202e90
   idf.py build
   ```

3. Flash to ESP32
   ```bash
   idf.py -p /dev/ttyUSB0 flash
   ```

4. Connect to QuadPID WiFi AP on phone/laptop
   - SSID: `QuadPID`
   - Password: `12345678`

---

## 🎯 Test Session 1: IMU Calibration Verification (10 min)

### Goal: Verify trim values are correct

1. **Place quad on known level surface** (use a level tool)

2. **Power on and observe serial output:**
   ```
   idf.py -p /dev/ttyUSB0 monitor
   ```

3. **Check these values:**
   - `IMU: Angles initialized - Roll: X, Pitch: Y`
   - Both should be **near 0°** (within ±0.5°)

4. **If NOT near zero:**
   - Edit `lib/imu/imu.c` line 57-58:
   - Adjust `PITCH_TRIM_DEG` and `ROLL_TRIM_DEG`
   - Rebuild and flash

5. **Expected Output:**
   ```
   IMU: Angles initialized - Roll: 0.1 (trim: 0.3), Pitch: 0.2 (trim: 2.0)
   ```

---

## 🎯 Test Session 2: Motor Balance Check (5 min)

### Goal: Verify all motors get equal PWM at hover

1. **Arm the quad (throttle low, switch high)**

2. **Hold at hover throttle (~1300-1400 μs)**

3. **Check serial output - Motor values:**
   ```
   M: 1285 1280 1278 1283
   ```
   - All 4 motors should be within **±10 μs**

4. **If M3 still higher:**
   - Increase `PITCH_TRIM_DEG` by +0.3°
   - Rebuild and flash

---

## 🎯 Test Session 3: Hover Test (15 min)

### Goal: Achieve stable hover within 30cm radius

1. **Find safe test area** (indoors or low wind)

2. **Arm and gently lift off to 0.5m**

3. **Release sticks - observe behavior:**
   
   | Observation | Action |
   |------------|--------|
   | Drifts forward | Increase `PITCH_TRIM_DEG` +0.3° |
   | Drifts backward | Decrease `PITCH_TRIM_DEG` -0.3° |
   | Drifts left | Increase `ROLL_TRIM_DEG` +0.2° |
   | Drifts right | Decrease `ROLL_TRIM_DEG` -0.2° |
   | Oscillates | Decrease `roll_kd` / `pitch_kd` |

4. **Good hover = stays within 30cm for 5+ seconds**

---

## 🎯 Test Session 4: Self-Leveling Test (10 min)

### Goal: Verify 0.5s settling time

1. **Hover at 1m height**

2. **Give quick stick input (5° tilt)**

3. **Release and count:**
   - Should return to level in **< 1 second**
   - No oscillation

4. **If too slow:**
   - Increase `angle_roll_kp` / `angle_pitch_kp` (try 4.0)
   
5. **If overshoots:**
   - Decrease `angle_roll_kp` / `angle_pitch_kp` (try 3.0)

---

## 📊 Data Collection

### After Each Test Flight:

1. **Disarm** (preserves blackbox data)

2. **Connect to QuadPID WiFi**

3. **Download blackbox:**
   - Open browser: `http://192.168.4.1/blackbox`
   - Save as `session10_flight1.csv`

4. **Create new folder:**
   ```bash
   mkdir -p ~/Videos/Project_Firmware1.1.../bb/10/
   mv ~/Downloads/blackbox.csv ~/Videos/Project_Firmware1.1.../bb/10/
   ```

---

## 📈 Post-Flight Analysis

// turbo
1. **Run analysis script:**
   ```bash
   cd ~/Videos/Project_Firmware1.1.../bb/analysis
   python3 flight_data_analysis.py
   ```

2. **Open report:**
   ```bash
   xdg-open comprehensive_analysis.html
   ```

3. **Check key metrics:**
   - Motor balance (M1-M4 within ±10 μs)
   - Angle error settling time (< 0.5s)
   - Gyro oscillation (std dev < 15°/s)

---

## 🎯 Success Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Hover radius | < 30 cm | ⬜ |
| Self-leveling time | < 0.5 sec | ⬜ |
| Motor balance | ±10 μs | ⬜ |
| Oscillation | None visible | ⬜ |
| Drift | Near zero | ⬜ |

---

## 📝 Notes

### Current PID Values:
```
Rate P: 0.04, I: 0.02, D: 0.008
Angle P: 3.5, I: 0.8
PITCH_TRIM: 2.0°
ROLL_TRIM: 0.3°
```

### Changes Made Today:
- Fixed IMU trim application (inside complementary filter)
- Expanded blackbox with setpoints, errors, accel data
- Increased logging rate to 83Hz
- Verified theoretical settling time = 0.48s

### Tomorrow's Focus:
1. Validate trim fixes motor balance
2. Confirm 0.5s self-leveling
3. Fine-tune if needed

---

**Good luck! 🚁✨**
