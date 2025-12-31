# ESP-IDF Global Library Usage Reference

This document details the standard ESP-IDF (Espressif IoT Development Framework) libraries used in this project. It explains **why** each library is used, **what** it does, and **how** it is implemented in the flight controller.

---

## 1. FreeRTOS (`freertos/`)
**Purpose**: The Real-Time Operating System kernel. Handles multitasking (Wi-Fi vs. Flight Control), critical timing, and inter-core communication.

### Key Functions Used:
*   **`xTaskCreatePinnedToCore`**
    *   **Usage**: Used in `main.c` (Flight Loop) and `blackbox.c` (Logger).
    *   **Implementation**: Manually assigns tasks to specific CPU cores.
        *   **Core 0**: Wi-Fi, Webserver, Blackbox (Background tasks).
        *   **Core 1**: Flight Control Loop, PWM, IMU (Real-time critical).
    *   **Why**: Isolates flight code from Wi-Fi interrupts to prevent jitters.

*   **`vTaskDelay` / `pdMS_TO_TICKS`**
    *   **Usage**: In `main.c` initialization and loops.
    *   **Implementation**: Pauses the current task for a set number of system ticks.
    *   **Why**: Used for simple delays (e.g., waiting for ESCs to boot). NOT used for the main flight loop timing (too imprecise).

*   **`xQueueCreate` / `xQueueSend` / `xQueueReceive`**
    *   **Usage**: In `blackbox.c`.
    *   **Implementation**: Creates a thread-safe FIFO buffer between the Flight Loop (Producer) and Logger Task (Consumer).
    *   **Why**: Allows the Flight Loop to "fire and forget" log data without waiting for slow memory operations.

*   **`portDISABLE_INTERRUPTS` / `portENABLE_INTERRUPTS`**
    *   **Usage**: In `rx.c`.
    *   **Implementation**: Disables all hardware interrupts briefly.
    *   **Why**: Ensures atomic reading of multi-byte variables (shared between ISR and Main Loop) to prevent data tearing.

---

## 2. ESP Timer (`esp_timer.h`)
**Purpose**: High-resolution hardware timer (microsecond precision).

### Key Functions Used:
*   **`esp_timer_get_time`**
    *   **Usage**: Everywhere (`main.c`, `rx.c`, `pid.c`).
    *   **Implementation**: Returns the number of microseconds since boot.
    *   **Why**:
        *   **Flight Loop**: Determines exactly how long the previous loop took (`dt`) for PID integration.
        *   **RX**: Measures the width of PPM pulses to decode stick positions.
        *   **Blackbox**: Timestamps logs.

---

## 3. Driver: GPIO (`driver/gpio.h`)
**Purpose**: General Purpose Input/Output control.

### Key Functions Used:
*   **`gpio_set_level` / `gpio_get_level`**
    *   **Usage**: LED blinking, motor safety output, button reading.
    *   **Implementation**: Direct register write to set pin voltage High (3.3V) or Low (0V).

*   **`gpio_install_isr_service` / `gpio_isr_handler_add`**
    *   **Usage**: In `rx.c`.
    *   **Implementation**: Attaches a C-function (`rx_isr_handler`) to a hardware pin interrupt.
    *   **Why**: Triggers code execution *immediately* when the PPM signal changes state, allowing sub-microsecond pulse measurement.

---

## 4. Driver: I2C (`driver/i2c.h`)
**Purpose**: Communication with the MPU6050 Gyroscope/Accelerometer.

### Key Functions Used:
*   **`i2c_master_cmd_begin`**
    *   **Usage**: In `imu.c`.
    *   **Implementation**: Executes a queue of I2C commands (Start, Write, Read, Stop).
    *   **Why**: Relies on ESP32's hardware I2C peripheral to handle the protocol bits, freeing the CPU.

*   **`i2c_cmd_link_create` (Legacy Driver)**
    *   **Usage**: Building command chains.
    *   **Why**: We use the older ESP-IDF I2C driver (Standard in Arduino/ESP-IDF v4.x) because it is robust and widely documented.

---

## 5. Driver: LEDC (`driver/ledc.h`)
**Purpose**: LED Control peripheral, purposed here for **PWM Motor Control**.

### Key Functions Used:
*   **`ledc_timer_config` / `ledc_channel_config`**
    *   **Usage**: In `pwm.c`.
    *   **Implementation**: Sets up a hardware timer running at 250Hz.
    *   **Why**: Generates precise pulses without using CPU cycles. The hardware toggles the pins automatically.

*   **`ledc_set_duty` / `ledc_update_duty`**
    *   **Usage**: In `pwm.c`.
    *   **Implementation**: Updates the "high time" of the pulse based on the PID output.

---

## 6. Driver: ADC OneShot (`esp_adc/adc_oneshot.h`)
**Purpose**: Analog-to-Digital Converter for Battery Voltage.

### Key Functions Used:
*   **`adc_oneshot_read`**
    *   **Usage**: In `adc.c`.
    *   **Implementation**: Triggers a single conversion on the requested channel.
    *   **Why**: "OneShot" mode is simpler than Continuous DMA mode and sufficient for battery monitoring (which changes slowly).

---

## 7. Configuration: NVS Flash (`nvs_flash.h`)
**Purpose**: Non-Volatile Storage (like EEPROM). Saves settings even when power is off.

### Key Functions Used:
*   **`nvs_open` / `nvs_commit` / `nvs_close`**
    *   **Usage**: In `config.c` (PIDs) and `imu.c` (Calibration).
    *   **Implementation**: Manages a key-value storage partition in the ESP32's flash memory.
    *   **Why**: Stores PID gains, Accelerometer Trim, and Gyro Bias so you don't have to re-tune/re-calibrate every flight.

---

## 8. Wi-Fi & Networking (`esp_wifi.h`, `esp_netif.h`)
**Purpose**: Creates the "QuadPID" Hotspot.

### Key Functions Used:
*   **`esp_wifi_set_mode(WIFI_MODE_AP)`**
    *   **Usage**: In `webserver.c`.
    *   **Implementation**: Configures the radio to act as a Router (Access Point).
    *   **Why**: Allows the user to connect their phone/laptop directly to the drone in the field.

---

## 9. HTTP Server (`esp_http_server.h`)
**Purpose**: Serves the Tuning Dashboard.

### Key Functions Used:
*   **`httpd_start` / `httpd_register_uri_handler`**
    *   **Usage**: In `webserver.c`.
    *   **Implementation**: Starts a lightweight web server task that listens on Port 80.
    *   **Why**: Handles incoming GET/POST requests and routes them to our C-functions (e.g., `save_handler` for saving PIDs).

---

## 10. Task Watchdog (`esp_task_wdt.h`)
**Purpose**: Safety mechanism. Reboots system if it freezes.

### Key Functions Used:
*   **`esp_task_wdt_add` / `esp_task_wdt_reset`**
    *   **Usage**: In `main.c` (Control Loop).
    *   **Implementation**: The Main Loop must "pet the dog" (`reset`) every cycle. If it gets stuck (infinite loop, deadlock), the timer expires and hard-resets the CPU.
    *   **Why**: Prevents a frozen drone from staying armed and dangerous.
