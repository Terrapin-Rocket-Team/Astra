---
title: Home
hide: footer
---

# Astra Documentation

Astra is the Terrapin Rocket Team's modular flight software library for embedded systems. It provides sensor abstraction, state estimation, logging, and testing utilities so you can focus on mission logic instead of plumbing.

---

## What Astra Gives You

- **Drop-in sensor stack**: IMU, GPS, barometer, magnetometer, voltage, and HITL sensors
- **State estimation**: Orientation (Mahony AHRS) + position/velocity (Kalman filter)
- **Unified logging**: CSV telemetry (`DataLogger`) and event logs (`EventLogger`)
- **Multi-platform**: STM32, Teensy, ESP32, plus native SITL builds
- **Testing & simulation**: HITL parser, SerialMessageRouter, SITL example

---

## Quick Start

1. Read [Installation](user-guide/installation.md)
2. Follow [Basic Usage](user-guide/basic-use.md)
3. Dive deeper via the User Guide and Interfaces sections

---

## Core Architecture

**Astra System**
- `Astra` orchestrates sensors, state updates, and telemetry logging
- `AstraConfig` wires sensors, log sinks, status LEDs, and HITL mode

**Sensors**
- Sensors are `DataReporter`s (auto-registered for logging)
- `Astra` updates sensors on their own rates and uses the latest healthy data

**State Estimation**
- `State` is math-only: it consumes vectors and outputs position/velocity/orientation
- `DefaultState` provides a ready-to-use filter stack

**Logging**
- `DataLogger` emits CSV telemetry (`TELEM/` prefix when enabled)
- `EventLogger` emits human-readable logs (`LOG/` prefix when enabled)

**Serial & Simulation**
- `SerialMessageRouter` routes prefixed messages (e.g., `HITL/`, `CMD/`)
- HITL/SITL sensors allow full-system testing without hardware

---

## Where To Go Next

- [Installation](user-guide/installation.md)
- [Basic Usage](user-guide/basic-use.md)
- [Astra System](user-guide/utils/mmfssys.md)
- [Sensors & Interfaces](user-guide/ifaces/sensor.md)
