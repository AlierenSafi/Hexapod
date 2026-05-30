# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [3.2.0] - 2026-05-30

### Performance Optimizations

#### I2C Bus Efficiency
- **PCA9685 Batch Writes**: Implemented `pca9685SetLeg()` function that writes 3 servo channels in a single I2C transaction
  - Reduced I2C transactions per kinematic cycle from 18 to 6
  - I2C overhead reduced from ~1.26ms to ~0.42ms per cycle
  - Added fallback to individual writes for non-consecutive channels

#### IMU Reading Optimization
- **50Hz IMU Sampling**: Reduced IMU read rate from 100Hz to 50Hz
  - `wireMutex` contention reduced by 50%
  - Added `imuSkipCount` counter in `taskSensorComm`
  - Adjusted `dt` compensation for skipped cycles

#### Telemetry Optimization
- **snprintf Fast Telemetry**: Replaced `StaticJsonDocument<768>` with direct `snprintf()`
  - Stack allocation eliminated (768 bytes → 0 bytes)
  - Backward compatible: preserved `cx/fm/tb` angle fields per leg
  - CPU cycle reduction of ~15%

#### ADC Oversampling
- **Removed Blocking Delay**: Eliminated `delayMicroseconds(100)` from ADC sampling loop
  - Sampling time reduced from ~3.2ms to ~0.05ms
  - Reduced `BATT_ADC_SAMPLES` from 32 to 16
  - Moving average filter still provides sufficient smoothing

#### Watchdog Fault Handler
- **5-Second Cooldown**: Added per-fault cooldown to `handleFault()`
  - Prevents I2C bus thrashing from persistent faults
  - Uses `lastHandleMs[8]` array with bit-position indexing
  - Improves system stability during fault conditions

#### KinTask Stack Optimization
- **Static zOff Array**: Moved `float zOff[6]` from loop-local to function-static
  - Eliminates per-iteration stack allocation
  - Memory moved from stack to `.bss` section

#### WebSocket Client Counting
- **Real Client Tracking**: Added `wsClientCount` variable
  - Incremented on `WStype_CONNECTED`
  - Decremented on `WStype_DISCONNECTED`
  - `getClientCount()` now returns actual connected clients

#### NVS Write Protection
- **CRC32 Dirty-Flag**: Added `crc32()` function and `savedConfigCRC` variable
  - Calculates CRC of `RobotSettings` (excluding WiFi credentials)
  - Skips NVS write if configuration unchanged
  - Extends Flash memory lifetime

#### TWDT Initialization Fix
- **Setup Integration**: Added `wdtInit()` call in `setup()`
  - TWDT was previously never initialized (silent bug)
  - All watchdog functionality now active

#### FreeRTOS Compatibility
- **sitDown() vTaskDelay**: Replaced `delay(16)` with FreeRTOS-aware delay
  - Uses `vTaskDelay()` when running in task context
  - Falls back to `delay()` when called from `setup()`

### Technical Details

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| I2C transactions/cycle | 18 | 6 | 66% reduction |
| ADC sampling time | ~3.2ms | ~0.05ms | 64x faster |
| Fast telemetry stack | 768 B | 0 B | Eliminated |
| IMU wireMutex load | 100Hz | 50Hz | 50% reduction |
| TWDT status | Passive | Active | Bug fixed |
| Flash write safety | None | CRC protected | Added |

### Compilation
- **Board**: ESP32 Dev Module
- **Core**: esp32:esp32@3.3.8
- **Flash Usage**: 1,069,347 bytes (81%)
- **RAM Usage**: 55,944 bytes (17%)
- **Result**: Success (0 errors)

## [3.1.0] - 2026-04-27

### Added
- Initial telemetry system with fast/slow/event channels
- Battery management with 4-level FSM
- WebSocket server with JSON command protocol
- IMU integration with complementary filter
- Multi-gait support (Tripod, Ripple, Wave)
- OTA firmware update capability

### Features
- Real-time kinematics at 50Hz
- WiFi AP/STA mode support
- BLE and NRF24 communication options
- Persistent configuration via NVS
- Safety systems (watchdog, timeout, battery)

## [3.0.0] - 2026-04-15

### Initial Release
- ESP32 dual-core FreeRTOS architecture
- PCA9685 servo driver integration
- Basic inverse kinematics solver
- Cycloid trajectory generation
- Phase-synchronized gait engine

---

## Release Notes Format

Each release includes:
- **Performance metrics** before/after comparison
- **Technical details** for engineering review
- **Compilation results** with board settings
- **Breaking changes** (if any)
- **Migration guide** (if required)

## Versioning Strategy

- **MAJOR**: Breaking changes to API or protocol
- **MINOR**: New features, optimizations, non-breaking improvements
- **PATCH**: Bug fixes, documentation updates

## Future Roadmap

- [ ] ROS2 integration
- [ ] SLAM support
- [ ] Autonomous navigation
- [ ] Machine learning gait optimization
- [ ] Web-based configuration UI
