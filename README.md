# ESP32 Hexapod Robot Controller

![Hexapod Concept Banner](hexapod_concept.jpg)

This repository contains the firmware for an 18-DOF (Degree of Freedom) hexapod walking robot powered by the ESP32. It handles inverse kinematics, leg coordination, balance leveling, and multiple remote control interfaces in real-time.

There are two versions of this project available in this repository:
- **`main` branch**: The optimized classic Arduino version using a modular `.ino` structure.
- **`feature/freertos-native-architecture` branch** (This branch): A native FreeRTOS version structured into clean C++ header and source files (`.h`/`.cpp`) using advanced RTOS queues and event groups.

---

## Features

- **Real-Time Inverse Kinematics (IK)**: Smooth analytical leg angle calculation running at 50Hz.
- **Davenport Auto-Leveling**: Dynamic body leveling using an MPU6050 IMU and PID correction.
- **Smooth Gait Generation**: Support for Tripod, Ripple, and Wave gaits using cycloid trajectories.
- **Multiple Control Interfaces**: Control the robot using a Web App (WebSockets), BLE, or an NRF24L01+ physical remote.
- **Wireless Updates (OTA)**: Upload firmware updates wirelessly over WiFi.
- **Safety Watchdog**: Built-in hardware watchdog timer (TWDT) and battery protection to shut down servos if power gets too low.

---

## Kinematics & Coordinate System

Below is the layout of the leg coordinates, joint angles ($\alpha, \beta, \gamma$), and leg numbering configurations:

![Hexapod Kinematics Diagram](hexapod_kinematics_concept.jpg)

---

## FreeRTOS Architecture (This Branch)

This branch has been redesigned from the ground up to utilize ESP32 dual-core FreeRTOS task coordination. Instead of polling global variables, it uses thread-safe communication primitives:

- **Core 0 Task (`taskSensorComm` - 100Hz)**: Handles IMU readings (MPU6050 complementary filter at 50Hz), battery oversampling, WebSockets/WiFi client loops, and watchdog polling.
- **Core 1 Task (`taskKinematics` - 50Hz)**: Calculates inverse kinematics, gait planning, and drives the PCA9685 servo controllers.
- **OTATask (Core 0)**: Runs in the background with lower priority for handling WiFi OTA updates.

### Primitives Used:
- **Command Queue (`xQueueCmd`)**: WebSockets, NRF24, and BLE tasks send control packets to this queue. The Kinematics task reads from it asynchronously, removing the need for a global commands mutex.
- **Phase Reset Event Group (`xEventGroupPhaseReset`)**: Handles leg switch contact events using lightweight thread-safe bit flags.
- **Task Notifications**: When NVS parameters change, the communication task sends a direct notification to the Kinematics task (`xTaskNotifyGive`) to trigger a configuration snapshot rebuild.
- **I2C Mutex (`wireMutex`)**: Prevents bus conflicts between Core 0 (IMU) and Core 1 (PCA9685) transactions.

---

## Hardware Connection Diagram

Below is the wiring schematic showing the connection between the ESP32, PCA9685 servo drivers, MPU6050 sensor, and wireless modules:

![Hexapod Wiring Diagram](hexapod_wiring_diagram.jpg)

---

## Project Structure

On this branch, the files are structured into clean C++ header and source files:
- `Hexapod_FreeRTOS.ino`: Initializes hardware, queues, event groups, and schedules the core tasks.
- `robot_config.h` / `.cpp`: Settings lifecycle, Preferences, and NVS flash writes with CRC32 dirty-flag protection.
- `servo_drivers.h` / `.cpp`: PCA9685 I2C driver utilizing auto-increment write optimizations.
- `kinematics.h` / `.cpp`: Gait scheduler, cycloid calculations, and analytical IK solving.
- `imu_leveling.h` / `.cpp`: MPU6050 interface, complementary filter, and auto-leveling PID regulator.
- `communication.h` / `.cpp`: WebSockets server connection, BLE characteristics, and NRF24 interfaces.
- `battery_monitor.h` / `.cpp`: Low-voltage battery monitoring with emergency torque-cutoff.
- `watchdog_safety.h` / `.cpp`: Task Watchdog (TWDT) registrations, feeding, and timeout handling.
- `telemetry.h` / `.cpp`: Custom `snprintf` telemetries for high-speed Processing GUI updates.

---

## Getting Started

### Prerequisites
To compile the code, you will need the following libraries installed in your Arduino IDE:
- **ArduinoJson** (by Benoit Blanchon)
- **WebSockets** (by Markus Sattler)
- **RF24** (by TMRh20 - optional, for physical remote control)

### Uploading the Code
1. Open `Hexapod_FreeRTOS.ino` inside the `Hexapod_FreeRTOS` folder in Arduino IDE.
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
