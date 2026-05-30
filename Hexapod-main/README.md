# Hexapod Robot Control System v3.1

[![ESP32](https://img.shields.io/badge/ESP32-DevKit-blue)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange)](https://platformio.org/)
[![Arduino](https://img.shields.io/badge/Arduino-IDE-green)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> A professional-grade, real-time control system for 6-legged robots powered by ESP32

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Architecture](#hardware-architecture)
- [Software Architecture](#software-architecture)
- [Installation](#installation)
- [Usage](#usage)
- [API Reference](#api-reference)
- [Performance](#performance)
- [Contributing](#contributing)
- [License](#license)

##  Overview

This project provides a complete ESP32-based control system for hexapod (6-legged) robots. It features real-time kinematics, multiple gait patterns, WiFi/WebSocket communication, battery management, and a modular architecture designed for both research and educational purposes.

### Key Highlights

-  **Real-time kinematics** at 50Hz on dual-core ESP32
-  **WebSocket control** via WiFi with JSON protocol
-  **Intelligent battery management** with safety shutdown
-  **3 gait patterns**: Tripod, Ripple, Wave
-  **Multiple control interfaces**: Web GUI, BLE, NRF24
-  **OTA updates** for remote firmware upgrades
-  **Live telemetry** with 10Hz IMU and status data

##  Features

### Motion Control
- **Inverse Kinematics (IK)**: Analytical solution for 3DOF leg chains
- **Auto-leveling**: PID-based body orientation compensation
- **Gait Generation**: Cycloid trajectory with configurable parameters
- **Smooth transitions**: Phase-synchronized leg coordination

### Communication
- **Primary**: WebSocket over WiFi (port 81)
- **Secondary**: BLE (optional, mobile apps)
- **Legacy**: NRF24 2.4GHz (optional, physical controllers)
- **Protocol**: JSON-based command/telemetry

### Safety Systems
- Watchdog timer (5s timeout)
- Communication timeout protection
- Battery voltage monitoring (3-tier protection)
- Servo position limits (software + hardware)

### Configuration
- Persistent storage in ESP32 NVS
- Runtime parameter adjustment
- Factory reset capability

##  Hardware Architecture

### Core Components

| Component | Specification | Interface |
|-----------|--------------|-----------|
| **MCU** | ESP32-WROOM-32 | - |
| **Clock** | 240 MHz Dual Core | - |
| **Flash** | 4MB (Minimal SPIFFS) | - |
| **RAM** | 520KB SRAM | - |

### Peripheral Devices

#### Servo Control: PCA9685 PWM Driver
```
I2C Address: 0x40 (Right), 0x41 (Left)
PWM Frequency: 50 Hz
Resolution: 12-bit (4096 steps)
Servo Range: 500μs - 2500μs (0-180°)
```

#### Sensors
| Sensor | Purpose | Interface | Address/Pin |
|--------|---------|-----------|-------------|
| **MPU6050** | 6-axis IMU | I2C | 0x68 |
| **ADC** | Battery monitoring | GPIO36 | ADC1_CH0 |
| **Switches** | Leg contact detection | GPIO | 16,17,18,19,21,22 |

#### Wireless Modules
| Module | Protocol | Use Case |
|--------|----------|----------|
| **ESP32 WiFi** | 802.11 b/g/n | Primary control, WebSocket |
| **NRF24L01+** | 2.4GHz RF | Physical controllers (optional) |
| **ESP32 BLE** | Bluetooth 4.2 | Mobile apps (optional) |

### Pinout

```cpp
// I2C Bus
#define I2C_SDA         21
#define I2C_SCL         22

// NRF24 (Optional)
#define NRF_CE          5
#define NRF_CSN         17

// Limit Switches
#define SWITCH_PINS     {16, 17, 18, 19, 21, 22}

// Servo Output Enable
#define OE_PIN          4

// Battery ADC
#define BATT_ADC_PIN    36
```

##  Software Architecture

### Project Structure

```
hexapod_esp32_v3/
├── src/
│   ├── hexapod_esp32_v3.ino    # Main entry point
│   ├── hexapod_config.ino      # NVS configuration
│   ├── hexapod_wifi.ino        # WiFi + WebSocket server
│   ├── hexapod_comm.ino        # BLE/NRF24 communication
│   ├── hexapod_telemetry.ino   # Telemetry system
│   ├── hexapod_battery.ino     # Battery management
│   ├── hexapod_watchdog.ino    # Watchdog & fault recovery
│   ├── hexapod_tasks.ino       # FreeRTOS tasks
│   ├── hexapod_imu.ino         # IMU sensor & filtering
│   ├── hexapod_gait.ino        # Gait algorithms
│   ├── hexapod_ik.ino          # Inverse kinematics
│   ├── hexapod_drivers.ino     # Hardware drivers
│   ├── hexapod_ota.ino         # OTA updates
│   └── hexapod_future.ino      # Future features stub
├── docs/
│   ├── api_reference.md
│   ├── hardware_setup.md
│   └── protocol_spec.md
├── platformio.ini
└── README.md
```

### Core Data Structures

#### RobotSettings
```cpp
struct RobotSettings {
    // Mechanical
    float coxaLen = 25.0f;        // Coxa length (mm)
    float femurLen = 50.0f;       // Femur length (mm)
    float tibiaLen = 70.0f;       // Tibia length (mm)
    float stanceRadius = 110.0f;  // Stance radius (mm)
    float stanceHeight = -80.0f;  // Body height (mm)
    
    // Gait
    float stepHeight = 30.0f;     // Step height (mm)
    float stepLength = 40.0f;     // Step length (mm)
    float gaitSpeed = 50.0f;      // Speed (0-100%)
    float swingRatio = 0.35f;     // Swing phase ratio
    GaitType gaitType = TRIPOD;   // Gait pattern
    
    // PID
    float kp = 1.2f, ki = 0.0f, kd = 0.5f;
    float levelingLimit = 15.0f;  // Max correction (mm)
    
    // IMU
    float compAlpha = 0.98f;      // Complementary filter
    bool levelingEnabled = true;
    
    // Battery
    float battWarnVolt = 7.0f;    // Warning threshold
    float battCritVolt = 6.4f;    // Critical threshold
    float battCutoffVolt = 6.0f;  // Shutdown threshold
    
    // Telemetry
    uint16_t telemetryRateMs = 100;  // Fast telemetry interval
    uint16_t commTimeoutMs = 5000;   // Command timeout
};
```

### FreeRTOS Task Distribution

| Task | Core | Frequency | Priority | Responsibilities |
|------|------|-----------|----------|------------------|
| **taskSensorComm** | 0 | 100Hz | High | IMU, WiFi, Telemetry, Battery, Watchdog |
| **taskKinematics** | 1 | 50Hz | High | Gait, IK, PID, Servo output |

### Communication Protocol

#### WebSocket Endpoint
```
ws://[ESP32_IP]:81
```

#### Command Format
```json
// Motion Control
{
    "cmd": "motion",
    "x": 50,      // Forward/Back [-100, 100]
    "y": 0,       // Left/Right [-100, 100]
    "yaw": 30     // Rotation [-100, 100]
}

// Gait Selection
{
    "cmd": "gait",
    "type": "tripod"  // "tripod" | "ripple" | "wave"
}

// Parameter Update
{
    "cmd": "param.set",
    "key": "stepHeight",
    "value": 35.0,
    "save": true
}

// System Commands
{
    "cmd": "system",
    "action": "save_nvs"  // save_nvs | load_nvs | reset_defaults
}
```

#### Telemetry Format
```json
// Fast Telemetry (10Hz)
{
    "t": "fast",
    "ts": 12345678,
    "imu": {
        "p": 5.2, "r": -3.1,
        "ax": 0.1, "ay": 0.0, "az": 0.98
    },
    "gait": 0,
    "moving": true,
    "legs": [
        {"ph": 0.25, "sw": true, "fx": 120, "fy": 80, "fz": -80}
    ]
}

// Slow Telemetry (1Hz)
{
    "t": "slow",
    "batt": {"v": 7.8, "pct": 65, "lvl": 3},
    "wifi": {"rssi": -45, "connected": true},
    "sys": {"up": 3600, "heap": 45000}
}
```

##  Installation

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or 2.x)
- [ESP32 Board Support](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
- Required Libraries:
  - `ArduinoJson` by Benoit Blanchon
  - `WebSockets` by Markus Sattler
  - `RF24` by TMRh20 (optional)

### Board Configuration

```
Board: ESP32 Dev Module
CPU Frequency: 240MHz
Flash Mode: QIO
Flash Size: 4MB
Partition Scheme: Minimal SPIFFS (1.9MB APP / 128KB SPIFFS)
Upload Speed: 921600
```

### Wiring Diagram

```
ESP32          PCA9685 (Right)    PCA9685 (Left)     MPU6050
-----          ---------------    --------------     -------
3.3V    ---->  VCC                VCC                VCC
GND     ---->  GND                GND                GND
GPIO21  ---->  SDA                SDA                SDA
GPIO22  ---->  SCL                SCL                SCL
         Addr: 0x40               0x41               0x68

ESP32          NRF24 (Optional)   Battery Monitor
-----          ----------------   ---------------
GPIO5   ---->  CE
GPIO17  ---->  CSN
GPIO36  ---->  ADC (with voltage divider 20k/10k)
```

### Upload

1. Clone this repository
2. Open `hexapod_esp32_v3.ino` in Arduino IDE
3. Select correct board and port
4. Click Upload

##  Usage

### Web Interface

1. Connect to robot's WiFi AP (SSID: `Hexapod-Setup`, Pass: `12345678`)
   - Or configure STA mode with your WiFi credentials
2. Open any WebSocket client and connect to `ws://192.168.4.1:81`
3. Send JSON commands as documented in the API section

### First Run

1. Robot starts in AP mode
2. Connect via WebSocket and send WiFi credentials
3. Robot will switch to STA mode
4. Use assigned IP for future connections

##  Performance

| Metric | Value |
|--------|-------|
| Kinematics Loop | 20ms (50Hz) |
| Sensor Loop | 10ms (100Hz) |
| WebSocket Latency | <10ms (local) |
| Telemetry Bandwidth | ~5 KB/s |
| Free Heap | ~45KB |
| Boot Time | ~2 seconds |


### Development Roadmap

- [x] Core kinematics and gait engine
- [x] WiFi/WebSocket communication
- [x] Battery management
- [x] IMU integration
- [ ] ROS2 integration
- [ ] SLAM support
- [ ] Autonomous navigation
- [ ] Machine learning gait optimization

##  License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

##  Acknowledgments

- [ArduinoJson](https://arduinojson.org/) - JSON library
- [WebSockets](https://github.com/Links2004/arduinoWebSockets) - WebSocket implementation
- [FreeRTOS](https://www.freertos.org/) - Real-time OS
- [Espressif](https://www.espressif.com/) - ESP32 platform

