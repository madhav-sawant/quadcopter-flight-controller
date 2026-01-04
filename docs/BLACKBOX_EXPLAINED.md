# Blackbox Logging System: The Flight Recorder

## 1. Why Do We Use It? (The Problem)
In drone development, things happen too fast for the human eye. 
*   **The "Blink" Problem:** A PID oscillation happens at 10-50Hz. You blink, and you miss it.
*   **The "Invisible" Problem:** You can't "see" what the Gyro is feeling or what the motors are being told to do. You only see the result (a crash or shake).
*   **The "Evidence" Problem:** When a drone crashes, you need to know *why* to fix it. Without data, you are just guessing (e.g., "Maybe P was too high?").

**The Blackbox solves this by recording the internal state of the Flight Controller 100 times per second.**

## 2. What Does It Do?
It acts exactly like the Flight Data Recorder on an airplane. It saves a **CSV (Excel-compatible)** file to the flash memory every time you arm the drone.

### Data Recorded (Every 10ms / 100Hz):
1.  **Time:** `time_ms` (Exact timeline of events)
2.  **Sensors:** `gyro_x, gyro_y, gyro_z` (What the drone *felt*)
3.  **Orientation:** `angle_roll, angle_pitch` (Where the drone *thought* it was)
4.  **Inputs:** `rc_roll, rc_pitch, rc_thr, rc_yaw` (What *you* told it to do)
5.  **Outputs:** `m1, m2, m3, m4` (What it told the *motors* to do)
6.  **PID Debug:** `pid_p, pid_i, pid_d` (Internal calculator decisions)
7.  **System:** `vbat, loop_time` (Battery health and CPU load)

## 3. How We Implemented It (Technical Deep Dive)

### A. Storage Technology: LittleFS
We use the ESP32's internal 4MB Flash memory.
*   **File System:** We use **LittleFS**, a fail-safe file system designed for microcontrollers. It survives sudden power loss (crashes) without corrupting the file system.
*   **Path:** Files are saved as `/blackbox.csv`, `/blackbox (1).csv`, etc.

### B. The Challenge: Latency
Writing to Flash memory is **SLOW**. It can take 10-20ms to write a block of data.
*   **Risk:** If we write to flash directly inside the main Flight Loop (`main.c`), the drone would pause for 20ms to save data. This would cause it to crash!

### C. The Solution: The Double-Buffer System
We separated the **Logging** from the **Flying**.

1.  **The Fast Path (Flight Loop - Core 1):**
    *   The `blackbox_update()` function is called.
    *   It does **NOT** write to disk.
    *   It formats the data into a string and pushes it into a **RAM Buffer**.
    *   This takes microseconds (fast!).

2.  **The Slow Path (Worker Task - Core 0):**
    *   A separate background task (`blackbox_task`) wakes up periodically.
    *   It looks at the RAM Buffer.
    *   If there is data, it writes it to the LittleFS file on Flash.
    *   This happens in the background, so the Flight Loop is never blocked.

## 4. Summary
We built a high-speed, non-blocking logger that allows us to reverse-engineer any crash. By using **LittleFS** and **RAM Buffering**, we ensure data integrity without sacrificing flight performance.

---

# Appendix: Research Paper Architecture Definition
*(Use the following text for your academic paper)*

## **IV. End-to-End Flight Data Acquisition and Analysis System**

To facilitate the rigorous post-flight analysis of high-frequency UAV dynamics, a custom "Blackbox" data acquisition architecture was implemented, spanning onboard real-time logging and wireless off-board extraction. The onboard system addresses the critical challenge of non-deterministic flash memory latency (10-20ms) by utilizing a dual-core asynchronous buffering strategy: the primary control core (Core 1) serializes the full state vector—comprising sensor fusion outputs ($ \omega, a, \theta, \phi $), control inputs, and actuator commands—into a high-speed circular RAM buffer at 100Hz, ensuring zero impact on the 250Hz stability loop. A decoupled, low-priority task on the auxiliary core (Core 0) asynchronously commits this buffered data to non-volatile storage using the fail-safe LittleFS file system. Upon mission completion, the system transitions to a "Ground Mode" state, activating an embedded HTTP server that exposes the stored datasets via a RESTful API. This allows for the automated, wireless extraction of flight logs to a ground station for subsequent time-domain analysis, enabling precise identification of mechanical phase lags, sensor noise, and control loop performance without physical data port access.

### **Figure 1: Full System Data Flow**
```mermaid
graph TD
    %% Global Styling for Black and White
    classDef bw fill:#FFF,stroke:#000,stroke-width:1px,color:#000;
    linkStyle default stroke:#000,stroke-width:1px,fill:none,color:#000;

    %% Onboard System
    subgraph ONBOARD["Onboard System (Dual-Core ESP32)"]
        direction TB
        A[Sensor Fusion]:::bw -- "State Vector" --> B[Control Loop]:::bw
        B -- "PWM Signals" --> C[Motors]:::bw
        B -. "Fast Push" .-> D[RAM Buffer]:::bw
        D -- "Fetch Data" --> E[Logger Task]:::bw
        E -- "Write Block" --> F[(Flash Storage)]:::bw
    end
    
    %% Invisible Spacer to force layout separation
    F ~~~ SPACER[ ]:::bw ~~~ H

    %% Ground Station
    subgraph GROUND["Ground Station Analysis"]
        direction TB
        H[Webserver]:::bw
        P[Python Script]:::bw
        G[Analysis Tool]:::bw
        
        H -. "Wi-Fi Download" .-> P
        P -- "Parse CSV" --> G
    end

    %% Connection (Cross-System)
    F -. "Read Log File" .-> H

    %% Apply Black and White Class
    class A,B,C,D,E,F,H,P,G,SPACER bw
    style SPACER width:0px,height:0px,stroke:none,fill:none
```
