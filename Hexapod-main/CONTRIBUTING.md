# 🤝 Contributing to ESP32 Hexapod

Thank you for taking the time to contribute! This document covers everything you need to know to get started — from setting up your environment to submitting a pull request.

---

## 📋 Table of Contents

- [Code of Conduct](#-code-of-conduct)
- [Before You Start](#-before-you-start)
- [Types of Contributions](#-types-of-contributions)
- [Development Environment Setup](#-development-environment-setup)
- [Project Structure](#-project-structure)
- [Coding Standards](#-coding-standards)
- [FreeRTOS & Dual-Core Rules](#-freertos--dual-core-rules)
- [Commit Message Format](#-commit-message-format)
- [Pull Request Process](#-pull-request-process)
- [Reporting Bugs](#-reporting-bugs)
- [Suggesting Features](#-suggesting-features)
- [File Header Template](#-file-header-template)

---

## 🧭 Code of Conduct

All contributors are expected to:

- Use respectful and constructive language.
- Welcome differing viewpoints and experience levels.
- Direct criticism at code and design decisions, not at people.
- Back technical arguments with reasoning and evidence.

---

## 🔍 Before You Start

Before submitting a contribution, please check:

1. Is there an open **Issue** or **Pull Request** already covering the same topic? If so, comment there first.
2. For large changes (new subsystems, architecture refactors), open an **Issue** first to discuss the approach before writing any code.
3. Make sure your toolchain is set up and the firmware compiles cleanly on your machine.

---

## 🎯 Types of Contributions

| Type | Description |
|---|---|
| 🐛 **Bug Fix** | Fix incorrect behavior in existing firmware modules |
| ✨ **New Feature** | Implement a stub from `hexapod_future.ino` or propose something new |
| 🔧 **Refactor** | Improve code quality without changing behavior |
| 📝 **Documentation** | Improve README, CONTRIBUTING, or inline code comments |
| ⚡ **Performance** | Reduce latency, stack usage, or heap consumption |
| 🧪 **Testing** | Add test sketches or single-module validation tools |
| 🔌 **Hardware Support** | Add support for new sensors, drivers, or communication modules |

---

## 🛠 Development Environment Setup

### Required Tools

| Tool | Version | Notes |
|---|---|---|
| **Arduino IDE** | 2.x (recommended) | Legacy 1.8.x also works |
| **ESP32 Arduino Core** | 2.0.x+ | Install via Board Manager |
| **Git** | Any recent version | For branching and PRs |

### Required Libraries

Install all three via `Tools → Manage Libraries`:

```
ArduinoJson    →  Benoit Blanchon  (v6.x — do NOT use v5 or v7)
WebSockets     →  Markus Sattler
RF24           →  TMRh20
```

### Board Settings

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module |
| Partition Scheme | **Minimal SPIFFS** (1.9MB APP / 128KB SPIFFS) |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO |
| Flash Size | 4MB (32Mb) |
| Upload Speed | 921600 |

> ⚠️ The Partition Scheme is critical. Using `Default` will leave insufficient space for OTA and may cause subtle memory issues at runtime.

### Fork & Clone

```bash
# 1. Fork the repository on GitHub, then:
git clone https://github.com/YOUR_USERNAME/hexapod-esp32.git
cd hexapod-esp32

# 2. Add the upstream remote so you can pull future changes:
git remote add upstream https://github.com/ORIGINAL_OWNER/hexapod-esp32.git
```

### Create a Feature Branch

Always work on a dedicated branch — never commit directly to `main`.

```bash
git checkout -b feature/my-new-feature
# or
git checkout -b fix/battery-fsm-edge-case
```

---

## 🗂 Project Structure

```
hexapod_esp32_v3/
│
├── hexapod_esp32_v3.ino    # Entry point: structs, globals, setup(), loop()
├── hexapod_config.ino      # NVS parameter load / save / reset
├── hexapod_wifi.ino        # WiFi connection manager + WebSocket server (port 81)
├── hexapod_comm.ino        # BLE service + NRF24L01+ receiver + packet parser
├── hexapod_telemetry.ino   # Fast / slow telemetry and event message system
├── hexapod_battery.ino     # ADC reading, moving-average filter, 4-level FSM
├── hexapod_watchdog.ino    # Hardware WDT + per-task FreeRTOS heartbeat monitor
├── hexapod_tasks.ino       # Core 0 SensorTask + Core 1 KinematicsTask loops
├── hexapod_imu.ino         # MPU6050 init + Complementary Filter + PID leveling
├── hexapod_gait.ino        # Gait phase manager + safe sitDown position
├── hexapod_ik.ino          # Analytical IK solver + Cycloid trajectory generator
├── hexapod_drivers.ino     # PCA9685 I2C driver + servo PWM write
├── hexapod_ota.ino         # Arduino OTA WiFi firmware update
└── hexapod_future.ino      # Autonomous navigation infrastructure stubs
```

**Ground rule:** Each `.ino` file owns one logical subsystem. Do not add functions to a file that belongs to a different subsystem — create a new file instead and declare its forward references in `hexapod_esp32_v3.ino`.

---

## 📐 Coding Standards

### General

- **Indentation:** 2 spaces (no tabs).
- **Line length:** Soft limit of 100 characters.
- **Language:** English for all new identifiers, comments, and documentation. Existing Turkish comments may be retained where they provide important context.
- **Naming conventions:**

| Kind | Convention | Example |
|---|---|---|
| Global variable | `camelCase` | `battState`, `imuData` |
| Local variable | `camelCase` | `rawVoltage`, `phaseStep` |
| Constant / `#define` | `UPPER_SNAKE_CASE` | `BATT_ADC_PIN`, `NRF_ADDRESS` |
| Struct / Enum type | `PascalCase` | `RobotSettings`, `GaitType` |
| Enum value | `UPPER_SNAKE_CASE` | `TRIPOD`, `BATT_CRITICAL` |
| Function | `camelCase` | `mpuInit()`, `sitDown()` |

### Memory

- **Prefer stack over heap.** Avoid `new` / `malloc` inside task loops.
- Use `static` for persistent local buffers inside functions.
- Check `ESP.getFreeHeap()` in Serial output when adding new subsystems. Aim to keep free heap above **30 KB** during normal operation.
- Keep `RadioPacket` and similar shared structs `__attribute__((packed))` and verify their sizes with `static_assert`.

### Floating Point

- Use `float` (not `double`) for all kinematic and sensor calculations. The ESP32 FPU handles single precision natively.
- Always suffix literals with `f`: `0.5f`, `180.0f`.

### Magic Numbers

Always name constants. Do not embed raw numeric literals in logic:

```cpp
// ❌ Bad
if (voltage < 6.4f) { ... }

// ✅ Good
if (voltage < cfg.battCritVolt) { ... }
```

### Error Handling

- I2C operations must check return codes and set the appropriate `sysState.faultCode` bit on failure.
- Fatal initialization failures (mutex creation, etc.) must call `esp_restart()` and print a descriptive `[FATAL]` message to Serial beforehand.
- Non-fatal failures (NRF not found, MPU not found) must set the corresponding `Available` flag and continue gracefully.

---

## ⚙️ FreeRTOS & Dual-Core Rules

This is the most important section for firmware contributions. Violating these rules causes intermittent crashes that are extremely difficult to debug.

### Mutex Ownership

| Mutex | Protects | Written By | Read By |
|---|---|---|---|
| `configMutex` | `cfg` (RobotSettings) | Core 0 (WebSocket cmd handler) | Core 1 (KinTask) |
| `cmdMutex` | `ctrlPkt`, `lastSrc` | Core 0 (packet parser) | Core 1 (KinTask) |
| `imuMutex` | `imuData` | Core 0 (SensorTask) | Core 1 (KinTask) |
| `wireMutex` | I2C bus (`Wire`) | Both cores | Both cores |

**Always take the narrowest lock possible and release it immediately.** Never hold two mutexes simultaneously — this is a deadlock waiting to happen.

```cpp
// ✅ Correct pattern — copy out what you need, then release
if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
  float h = cfg.stanceHeight;
  xSemaphoreGive(configMutex);
  // use h here, outside the lock
}
```

### Cross-Core Signalling

Use `volatile bool` flags for single-bit signals from Core 0 to Core 1:

```cpp
volatile bool configChanged   = false;  // Core 0 sets → Core 1 reads & clears
volatile bool phaseResetReq[6] = {};    // Core 0 sets → Core 1 reads & clears
```

Do **not** use `volatile` as a substitute for a mutex when sharing multi-byte structures.

### Task Stack Sizes

| Task | Current Stack | Minimum Safe Headroom |
|---|---|---|
| `SensorTask` | 10 240 bytes | 512 bytes |
| `KinTask` | 20 480 bytes | 512 bytes |

If your change adds significant local variables inside a task loop, measure the remaining headroom with `uxTaskGetStackHighWaterMark()` and report the result in your PR description.

### Blocking Calls

- **Never call `delay()`** inside a FreeRTOS task. Use `vTaskDelay(pdMS_TO_TICKS(N))`.
- **Never call `Wire` functions** without first acquiring `wireMutex`.
- Keep task loop bodies deterministic. Avoid dynamic memory allocation inside task loops.

---

## 📝 Commit Message Format

This project follows a simplified [Conventional Commits](https://www.conventionalcommits.org/) style:

```
<type>(<scope>): <short summary>

[optional body]

[optional footer]
```

### Types

| Type | When to Use |
|---|---|
| `feat` | A new feature or capability |
| `fix` | A bug fix |
| `refactor` | Code restructuring without behavior change |
| `perf` | Performance improvement |
| `docs` | Documentation only |
| `chore` | Build config, tooling, or dependency updates |
| `test` | Adding or improving test sketches |

### Scopes

Use the filename without the `hexapod_` prefix and `.ino` extension:

```
feat(ik): add reachability guard before IK solve
fix(battery): clamp ADC reading to valid 12-bit range
refactor(gait): extract phase advance into helper function
docs(readme): add OTA update instructions
perf(tasks): reduce KinTask stack usage by 2 KB
```

### Rules

- Use **imperative mood** in the summary: "add", "fix", "remove" — not "added", "fixes", "removed".
- Keep the summary line **under 72 characters**.
- Reference related issues in the footer: `Closes #42` or `Refs #17`.

---

## 🔃 Pull Request Process

1. **Sync with upstream** before opening a PR:
   ```bash
   git fetch upstream
   git rebase upstream/main
   ```

2. **Verify the build** compiles without warnings using the board settings listed above.

3. **Test on hardware** if at all possible. State in the PR description which modules were exercised and how.

4. **Open the PR** against the `main` branch and fill in the following template:

   ```markdown
   ## Summary
   Brief description of what this PR does and why.

   ## Changes
   - hexapod_ik.ino: added reachability guard
   - hexapod_config.ino: new NVS key `ik_strict`

   ## Testing
   - Tested on hardware: yes / no
   - Modules exercised: IK, Config, KinTask
   - KinTask stack high-water mark: 1 024 bytes remaining

   ## Related Issues
   Closes #42
   ```

5. A PR will be merged once it:
   - Compiles cleanly with zero warnings.
   - Does not regress any existing functionality.
   - Has at least one reviewer approval.
   - Has all review comments resolved.

---

## 🐛 Reporting Bugs

Open a GitHub Issue and include:

- **Firmware version** — shown in Serial Monitor on boot, e.g. `v3.1.0`.
- **Hardware configuration** — which optional modules are connected (NRF24, MPU6050, BLE enabled)?
- **Steps to reproduce** — exact sequence of commands or events.
- **Expected behavior** — what should have happened.
- **Actual behavior** — what actually happened, including any Serial Monitor output.
- **Panic / stack trace** — if the device crashed, paste the full output from Serial Monitor.

Helpful extras: logic analyzer captures, oscilloscope traces, or a short video of the physical robot behavior.

---

## 💡 Suggesting Features

Open a GitHub Issue with the label `enhancement` and include:

- **Problem statement** — what limitation or gap does this address?
- **Proposed solution** — how would you implement it? Which files would change?
- **Alternatives considered** — other approaches you evaluated and why you ruled them out.
- **Impact on existing behavior** — will this affect task timing, memory usage, or the wire protocol?

Features that map to existing stubs in `hexapod_future.ino` are especially welcome and will be prioritized.

---

## 📄 File Header Template

All new `.ino` files must begin with this header block:

```cpp
// ════════════════════════════════════════════════════════════════
//  hexapod_<module>.ino  ·  <One-line description>
//
//  Responsibilities:
//    · <bullet 1>
//    · <bullet 2>
//
//  Runs on:  Core <0|1>  at  <frequency> Hz
//  Mutexes:  <list mutexes taken, or "none">
// ════════════════════════════════════════════════════════════════
```

---

## 🙏 Thank You

Every contribution — no matter how small — makes this project better. Whether you fix a typo, sharpen a comment, or implement a whole new gait algorithm, it is genuinely appreciated.

Happy hacking! 🦾
