# ESP32 Hexapod Robot Controller

This repository contains the firmware for an 18-DOF (Degree of Freedom) hexapod walking robot powered by the ESP32. It handles inverse kinematics, leg coordination, balance leveling, and multiple remote control interfaces in real-time.

There are two versions of this project available in this repository:
- **`main` branch**: The optimized classic Arduino version using a modular `.ino` structure.
- **`feature/freertos-native-architecture` branch**: A native FreeRTOS version structured into clean C++ header and source files (`.h`/`.cpp`) using advanced RTOS queues and event groups.

---

## Features

- **Real-Time Inverse Kinematics (IK)**: Smooth analytical leg angle calculation running at 50Hz.
- **Davenport Auto-Leveling**: Dynamic body leveling using an MPU6050 IMU and PID correction.
- **Smooth Gait Generation**: Support for Tripod, Ripple, and Wave gaits using cycloid trajectories.
- **Multiple Control Interfaces**: Control the robot using a Web App (WebSockets), BLE, or an NRF24L01+ physical remote.
- **Wireless Updates (OTA)**: Upload firmware updates wirelessly over WiFi.
- **Safety Watchdog**: Built-in hardware watchdog timer (TWDT) and battery protection to shut down servos if power gets too low.

---

## Hardware Connection Diagram

Below is the wiring schematic showing the connection between the ESP32, PCA9685 servo drivers, MPU6050 sensor, and wireless modules:

![Hexapod Wiring Diagram](hexapod_wiring_diagram.jpg)

---

## Project Structure

Depending on the branch you are on, the files are structured as follows:

### Classic Arduino (Main Branch)
The code in the `Hexapod Code` folder uses standard Arduino IDE compilation:
- `hexapod_esp32_v3.ino`: Setup, main loop, and global states.
- `hexapod_tasks.ino`: Core 0 (sensors/comm) and Core 1 (kinematics) task loops.
- `hexapod_drivers.ino`: PCA9685 I2C servo drivers.
- `hexapod_ik.ino` & `hexapod_gait.ino`: Kinematics, cycloid steps, and gait planning.
- `hexapod_wifi.ino` & `hexapod_comm.ino`: WebSockets, WiFi, BLE, and NRF24 handlers.
- `hexapod_config.ino`: NVS parameter storage with CRC32 flash protection.
- `hexapod_battery.ino` & `hexapod_watchdog.ino`: Power management and safety monitors.
- `hexapod_telemetry.ino`: Real-time fast/slow data telemetry.

### Native FreeRTOS (FreeRTOS Branch)
The code in the `Hexapod_FreeRTOS` folder is rewritten into modular C++ classes:
- `Hexapod_FreeRTOS.ino`: Entry point initializing tasks, queues, and event groups.
- `robot_config.h` / `.cpp`: Settings lifecycle and NVS flash writes.
- `servo_drivers.h` / `.cpp`: PCA9685 I2C writes with auto-increment optimization.
- `kinematics.h` / `.cpp`: Gait engine and IK calculations.
- `communication.h` / `.cpp`: Dynamic WebSocket client tracking, BLE, and NRF24.
- `battery_monitor.h` / `.cpp`: Oversampled ADC reading and low-voltage auto-shutdown.
- `watchdog_safety.h` / `.cpp`: Task WDT feed, comm timeout, and recovery cooldown.

---

## Getting Started

### Prerequisites
To compile the code, you will need the following libraries installed in your Arduino IDE:
- **ArduinoJson** (by Benoit Blanchon)
- **WebSockets** (by Markus Sattler)
- **RF24** (by TMRh20 - optional, for physical remote control)

### Uploading the Code
1. Open the main `.ino` sketch folder in Arduino IDE.
2. Under **Tools**, configure the ESP32 partition scheme to **Minimal SPIFFS (1.9MB APP)**.
3. Select your ESP32 Dev Module port.
4. Click **Upload**.

### Controlling the Robot
- On startup, the robot creates a WiFi Access Point named `Hexapod-Setup` (password: `12345678`).
- Connect to it and open your WebSocket client (or custom controller) at `ws://192.168.4.1:81`.
- Send walking commands as JSON packets:
  ```json
  {
    "cmd": "motion",
    "x": 50,
    "y": 0,
    "yaw": 10
  }
  ```

---

## License
Licensed under the MIT License. See [LICENSE](LICENSE) for details.

Developed & maintained by **Ali Eren Safi**.
