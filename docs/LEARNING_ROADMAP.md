# Learning Roadmap: Understanding Your Flight Controller

This document outlines everything you need to learn to fully understand the ESP32 quadcopter flight controller system. You mentioned having intermediate C knowledge (up to pointers) - that's a great foundation. This roadmap builds on that.

---

## 1. C Language Deep Dive

### What You Need to Master

#### Structs and Typedef
Your code uses structs extensively (e.g., `imu_data_t`, `blackbox_entry_t`).

```c
typedef struct {
    float roll_kp;
    float roll_ki;
} system_config_t;

extern system_config_t sys_cfg;  // Global config
```

**Learn:**
- How `typedef` creates type aliases
- Struct packing with `__attribute__((packed))`
- Accessing struct members via pointers (`imu->roll_deg`)

#### Function Pointers
Used in ESP-IDF for callbacks (e.g., HTTP handlers, WiFi events).

```c
static esp_err_t get_handler(httpd_req_t *req);  // Function
httpd_uri_t uri = { .handler = get_handler };    // Pointer to function
```

**Learn:**
- Function pointer syntax
- Callback patterns

#### Static Variables
Used throughout for state preservation.

```c
static float i_roll = 0.0f;  // Persists across function calls
```

**Learn:**
- Difference between `static` local vs `static` global
- Why we use `static` for module-internal state

#### Volatile Keyword
Critical for embedded systems.

```c
static volatile bool control_loop_flag = false;  // Modified by ISR
```

**Learn:**
- Why `volatile` prevents compiler optimization
- When to use it (ISRs, hardware registers)

---

## 2. Embedded Systems Fundamentals

### Memory Model
- **RAM vs Flash**: Variables in RAM, code in Flash
- **Stack vs Heap**: Local variables vs `malloc()`
- **NVS (Non-Volatile Storage)**: Persistent settings (like EEPROM)

### GPIO (General Purpose I/O)
```c
gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
gpio_set_level(LED_PIN, 1);  // Turn ON
```

**Learn:**
- Input/Output modes
- Pull-up/Pull-down resistors
- Why we use `gpio_reset_pin()` first

### PWM (Pulse Width Modulation)
ESCs are controlled via PWM signals (1000-2000 μs).

```c
pwm_set_motor(0, 1500);  // 50% throttle
```

**Learn:**
- Duty cycle concept
- How ESCs interpret PWM signals
- LEDC peripheral on ESP32

### I2C Communication
MPU6050 IMU communicates over I2C bus.

**Learn:**
- Master-Slave architecture
- SDA/SCL lines
- Register read/write operations
- Why we use GPIO 21/22 for I2C

### Interrupts and ISRs
PPM receiver uses interrupt-driven decoding.

```c
static void IRAM_ATTR ppm_isr_handler(void* arg) {
    // Called on every PPM pulse edge
}
```

**Learn:**
- What `IRAM_ATTR` means (code in fast RAM)
- ISR constraints (no blocking, no printf)
- Why `volatile` is needed for ISR-shared variables

---

## 3. FreeRTOS (Real-Time Operating System)

ESP-IDF is built on FreeRTOS. Your flight controller uses it extensively.

### Tasks
```c
xTaskCreatePinnedToCore(
    control_loop_task,  // Function
    "control_loop",     // Name
    4096,               // Stack size
    NULL,               // Parameters
    24,                 // Priority (higher = more important)
    NULL,               // Handle
    1                   // Core (0 or 1)
);
```

**Learn:**
- Task creation and priorities
- Why control loop runs on Core 1 (isolated from WiFi)
- Stack size considerations

### Timing
```c
vTaskDelay(pdMS_TO_TICKS(100));  // Sleep for 100ms
```

**Learn:**
- Why `vTaskDelay` is preferred over busy-wait
- Tick rate (default 100Hz = 10ms tick)
- Why we use `esp_timer_get_time()` for microsecond precision

### Semaphores and Mutexes
```c
static SemaphoreHandle_t buffer_mutex;
xSemaphoreTake(buffer_mutex, portMAX_DELAY);
// Critical section
xSemaphoreGive(buffer_mutex);
```

**Learn:**
- Race conditions and why we need locks
- Mutex vs Binary Semaphore
- Deadlock prevention

### Watchdog Timer
```c
esp_task_wdt_add(NULL);   // Register task
esp_task_wdt_reset();     // Feed the watchdog
```

**Learn:**
- Why watchdogs exist (detect frozen code)
- How to properly feed the watchdog

---

## 4. Control Theory (PID)

### The PID Algorithm
```
Output = Kp*Error + Ki*∫Error + Kd*dError/dt
```

**Your system has two loops:**

1. **Angle Loop (Outer)**: Stick → Target Angle → Angle Error → Rate Setpoint
2. **Rate Loop (Inner)**: Rate Setpoint → Rate Error → Motor Commands

### What to Learn

#### Proportional (P)
- Larger P = faster response, but can oscillate
- Error × Kp = Output

#### Integral (I)
- Accumulates error over time
- Fixes steady-state error (drift)
- **Anti-windup**: Clamp integral to prevent runaway

```c
if (i_roll > max_integral) i_roll = max_integral;
```

#### Derivative (D)
- Responds to rate of change of error
- Dampens oscillations
- Sensitive to noise (why rate loop D is small)

### Tuning Process
1. Set I and D to 0
2. Increase P until oscillation
3. Back off P by ~30%
4. Add D to dampen oscillation
5. Add I to fix drift

**Resources:**
- [PID Without a PhD](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf)
- YouTube: "PID Control - A Brief Introduction"

---

## 5. Sensor Fusion (IMU)

### MPU6050 Sensors
- **Gyroscope**: Measures angular rate (deg/s) - fast but drifts
- **Accelerometer**: Measures gravity vector (g) - slow but accurate

### Complementary Filter
```c
angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle;
```

**Learn:**
- Why gyro alone drifts over time
- Why accelerometer is noisy on a vibrating quad
- How complementary filter combines both
- Alpha value trade-off (0.98 = trust gyro more)

### Calibration
- **Gyro**: Zero out bias (must be stationary)
- **Accelerometer**: Level calibration (accounts for mounting angle)

---

## 6. Quadcopter Physics

### Motor Mixing (X Configuration)
```
      Front
   M4     M2
     \   /
      \ /
       X
      / \
     /   \
   M3     M1
      Rear
```

**Motor directions:**
- M1, M4: CW (clockwise)
- M2, M3: CCW (counter-clockwise)

**Mixing equations:**
```c
motor[0] = throttle - roll - pitch - yaw;  // Rear Right
motor[1] = throttle + roll - pitch + yaw;  // Front Right
motor[2] = throttle - roll + pitch + yaw;  // Rear Left
motor[3] = throttle + roll + pitch - yaw;  // Front Left
```

**Learn:**
- Why opposite motors spin in opposite directions (torque cancellation)
- How roll/pitch/yaw commands affect individual motors
- Why yaw control uses differential thrust

---

## 7. ESP-IDF Framework

### Key APIs Used in Your Project

| Module | Purpose | Key Functions |
|--------|---------|---------------|
| GPIO | Pin control | `gpio_set_direction()`, `gpio_set_level()` |
| LEDC | PWM output | `ledc_set_duty()`, `ledc_update_duty()` |
| I2C | IMU communication | `i2c_master_write_read_device()` |
| NVS | Persistent storage | `nvs_set_blob()`, `nvs_get_blob()` |
| WiFi | Access Point | `esp_wifi_start()` |
| HTTP | Web server | `httpd_start()`, `httpd_register_uri_handler()` |

### Documentation
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

---

## 8. Debugging Techniques

### Serial Monitor
```bash
pio device monitor -b 115200
```

### Printf Debugging
```c
printf("Roll=%.2f Pitch=%.2f\n", imu->roll_deg, imu->pitch_deg);
```

### Blackbox Analysis
1. Download CSV from `192.168.4.1/blackbox`
2. Plot in Python/Excel
3. Look for oscillations, drift, saturation

### Common Issues
- **Oscillation**: P too high or D too low
- **Drift**: I-term needed or accelerometer not calibrated
- **Motor spin on boot**: ESC initialization timing issue

---

## 9. Realistic 4-5 Week Study Plan

**Your time budget:** 4-5 weeks × 5-6 hrs/day = **~150 hours total**

### Priority-Based Approach
Not everything is equally important. Focus on what directly explains YOUR code.

---

### Week 1: Foundations (35 hrs)
**Goal:** Understand the building blocks

| Day | Topic | Hours | What to Do |
|-----|-------|-------|------------|
| 1-2 | C: Structs, typedef, extern | 10 | Read `config.h`, `config.c` - trace how `sys_cfg` works |
| 3 | C: Static variables | 5 | Read `angle_control.c` - why are `i_roll`, `i_pitch` static? |
| 4 | C: Function pointers | 5 | Read `webserver.c` lines 303-334 - HTTP handlers |
| 5-6 | GPIO + PWM basics | 10 | Read `pwm.c`, try modifying LED blink timing |
| 7 | I2C basics | 5 | Read `imu.c` register read/write functions |

**Checkpoint:** You should understand how config is stored and how motors get PWM signals.

---

### Week 2: Control Theory + IMU (35 hrs)
**Goal:** Understand the "brains" of the system

| Day | Topic | Hours | What to Do |
|-----|-------|-------|------------|
| 1-2 | PID Theory | 10 | Watch [Brian Douglas PID videos](https://www.youtube.com/watch?v=wkfEZmsQqiA), then read `pid.c` |
| 3 | Rate Control | 5 | Read `rate_control.c` - how it uses `pid.c` |
| 4 | Angle Control | 5 | Read `angle_control.c` - the outer loop you just refactored |
| 5 | Complementary Filter | 5 | Read `imu.c` sensor fusion section (search for "complementary") |
| 6-7 | Motor Mixing | 10 | Read `mixer.c` - draw out the X-quad and trace each motor |

**Checkpoint:** You should be able to explain how stick input becomes motor output.

---

### Week 3: FreeRTOS + Real-time (30 hrs)
**Goal:** Understand timing and multitasking

| Day | Topic | Hours | What to Do |
|-----|-------|-------|------------|
| 1-2 | FreeRTOS Tasks | 10 | Read `main.c` task creation, understand Core 0 vs Core 1 |
| 3 | Timing | 5 | Understand `esp_timer_get_time()` vs `vTaskDelay()` |
| 4 | ISRs | 5 | Read `rx.c` - PPM interrupt handler |
| 5 | Volatile keyword | 5 | Find all `volatile` in codebase, understand why each is needed |
| 6 | Watchdog | 5 | Search for `wdt` in `main.c`, understand crash protection |

**Checkpoint:** You should understand why control loop is on Core 1 and WiFi on Core 0.

---

### Week 4: System Integration (30 hrs)
**Goal:** See how everything connects

| Day | Topic | Hours | What to Do |
|-----|-------|-------|------------|
| 1-2 | Main.c deep read | 10 | Read entire `main.c` line by line, add comments |
| 3 | Data flow tracing | 5 | Draw diagram: IMU → Angle PID → Rate PID → Mixer → Motors |
| 4 | Blackbox analysis | 5 | Download a flight log, plot in Python/Excel |
| 5 | Webserver | 5 | Read `webserver.c`, understand HTTP POST handling |
| 6-7 | Modify & test | 5 | Change one parameter, flash, observe behavior |

**Checkpoint:** You can explain the entire system from sensor to motor.

---

### Week 5 (Optional Buffer): Deep Dives (20 hrs)
| Topic | Hours | Notes |
|-------|-------|-------|
| NVS storage | 5 | How settings persist across reboots |
| Blackbox internals | 5 | Queue-based logging |
| ESP-IDF WiFi | 5 | AP mode setup |
| Advanced PID tuning | 5 | Anti-windup, derivative filtering |

---

### Daily Routine Suggestion
```
Hour 1-2: Read theory/documentation
Hour 3-4: Read YOUR code with that knowledge
Hour 5:   Modify something small and test
Hour 6:   Notes/review (optional, prevents burnout)
```

---

### Minimum Viable Understanding (~80 hours)
If you're really short on time, focus on these ONLY:

1. **Week 1:** `config.c`, `pwm.c` - 15 hrs
2. **Week 2:** `pid.c`, `rate_control.c`, `angle_control.c` - 20 hrs  
3. **Week 3:** `imu.c` (sensor fusion only), `mixer.c` - 15 hrs
4. **Week 4:** `main.c` control loop section - 15 hrs
5. **Buffer:** Practice, Q&A - 15 hrs

This gets you **functional understanding** in ~3 weeks.

---

## 10. Code Reading Order

Read your codebase in this order for best understanding:

1. **`lib/config/config.c`** - Simple, shows NVS and struct usage
2. **`lib/pwm/pwm.c`** - GPIO and timer setup for motors
3. **`lib/imu/imu.c`** - I2C, sensor fusion, calibration
4. **`lib/pid/pid.c`** - Core PID algorithm
5. **`lib/rate_control/rate_control.c`** - Wrapper using PID
6. **`lib/angle_control/angle_control.c`** - Outer loop PI
7. **`lib/mixer/mixer.c`** - Motor mixing logic
8. **`lib/rx/rx.c`** - ISR-based PPM decoding
9. **`lib/blackbox/blackbox.c`** - Queue-based logging
10. **`src/main.c`** - Ties everything together

---

## 11. Recommended Resources

### Books
- "Making Embedded Systems" by Elecia White
- "The Definitive Guide to ARM Cortex-M3/M4" (concepts apply)

### Online
- [LearnCpp.com](https://www.learncpp.com/) - Modern C/C++ tutorial
- [FreeRTOS Documentation](https://www.freertos.org/Documentation)
- [Phil's Lab YouTube](https://www.youtube.com/@PhilsLab) - IMU, PID, embedded

### Practice Projects
1. Build a line-following robot (simpler PID)
2. Make a self-balancing stick (1-axis angle control)
3. Implement Betaflight's Blackbox parser in Python

---

*Remember: Understanding comes from doing. Modify small parts of the code, break things, and fix them. That's the best way to learn!*
