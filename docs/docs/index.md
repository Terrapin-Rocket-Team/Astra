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
- Sensors are `DataReporter`s; Astra registers the sensors supplied in its configuration
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

## Related Projects

Astra is the reusable bottom layer of the current flight-software stack:

- [Astra-Rocket](https://terrapin-rocket-team.github.io/Astra-Rocket/) adds
  rocket flight stages, flight-oriented defaults, and ARC command handling.
- [Astra-Support](https://github.com/Terrapin-Rocket-Team/Astra-Support) provides
  the shared setup, test, and simulation CLI.
- [SRAD-Avionics](https://github.com/Terrapin-Rocket-Team/SRAD-Avionics) owns the
  board-specific firmware and end-to-end integration procedure.
- [Airbrake](https://github.com/Terrapin-Rocket-Team/Airbrake) contains the
  Astra-Rocket-based airbrake application and its closed-loop simulation.
- Start with the canonical
  [install-to-HITL workflow](https://github.com/Terrapin-Rocket-Team/SRAD-Avionics/blob/main/docs/software-stack.md)
  when setting up or handing off the complete stack.

Keep Astra API details here. Do not copy them into downstream documentation;
downstream guides should link back to the relevant Astra page.
