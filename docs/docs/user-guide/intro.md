---
title: User Manual - Introduction
---
# The Complete Astra User's Manual

!!! info
    These docs are organized in the order shown in the sidebar. Later pages may reference concepts covered earlier, though each should be reasonably self-sufficient.

    If you have questions, reach out on Slack or GitHub.

---

This is the official user's manual for TRT's Astra flight software library. It's designed as a comprehensive guide for users of all levels, from beginners to advanced developers. The manual covers everything from installation and setup to advanced features and troubleshooting.

Astra is organized into two main categories: **Utilities** and **Interfaces**.

---

## Utilities

Utilities are parts of the library designed to be used as-is, without requiring you to extend or modify them. Astra provides the following utilities:

### **[Astra System](utils/mmfssys.md)**
The main system coordinator that manages sensors, state, logging, and BlinkBuzz. Configure it with `AstraConfig` and call `init()` and `update()` to run your telemetry system.

### **[Data Logger & Event Logger](utils/logger.md)**
Two separate logging systems:
- **DataLogger**: Automatic CSV telemetry recording from all sensors and state
- **EventLogger**: Human-readable timestamped messages with severity levels (INFO, WARNING, ERROR)

### **[BlinkBuzz](utils/blinkbuzz.md)**
An asynchronous utility for creating LED blinks and buzzer patterns, including morse code. Supports both blocking and non-blocking operation.

### **[CircBuffer](utils/circbuf.md)**
A circular buffer implementation for fixed-size FIFO queues, useful for storing recent sensor data or managing log buffers.

### **[Math Libraries](utils/math.md)**
Vector, matrix, and quaternion classes with common operations for state estimation and sensor fusion calculations.

---

## Interfaces

Interfaces are abstract classes or extensible components that you customize for your specific vehicle and mission. Astra provides these interfaces:

### **[State](ifaces/state.md)**
The core state estimation class. Manages sensor fusion and provides unified access to position, velocity, acceleration, orientation, and GPS coordinates. Extend it to add custom behavior like launch detection or stage transitions.

### **[Sensor](ifaces/sensor.md)**
The base sensor interface with type-safe specializations for:

- **[Barometer](ifaces/sensors/baro.md)** - Pressure sensors providing altitude and temperature
- **[IMU](ifaces/sensors/imu.md)** - 9DOF inertial measurement units (accelerometer, gyroscope, magnetometer)
- **[GPS](ifaces/sensors/gps.md)** - Position and velocity from satellite navigation
- **[Encoder](ifaces/sensors/enc.md)** - Motor encoder for rotational tracking
- **[Light Sensor](ifaces/sensors/light.md)** - Ambient light detection

### **[DataReporter](ifaces/data-reporter.md)**
An interface for adding custom data to CSV telemetry logs. All sensors implement this, and you can extend it for your own data sources (e.g., battery voltage, custom calculations).

### **[Filters](ifaces/filters.md)**
A Kalman filter interface that can be passed to `State` for improved state estimation. Implement custom filters for your specific vehicle dynamics.

---

## Platform Support

Astra supports multiple microcontroller platforms and storage backends:

**Supported Microcontrollers:**
- STM32 (various families)
- Teensy (3.x, 4.x)
- ESP32

**Storage Options:**
- SD Cards via SPI
- SD Cards via SDIO (platform-dependent)
- Internal flash memory
- eMMC (platform-dependent)

Platform-specific features are handled automatically based on your PlatformIO configuration.

---

## Design Philosophy

Astra's architecture emphasizes:

1. **Modularity**: Use only the components you need
2. **Type Safety**: Sensor types are checked at compile time while maintaining runtime flexibility
3. **Automatic Logging**: Sensors and state automatically register their data—no manual column management
4. **Vehicle Agnostic**: Not just for rockets—suitable for drones, rovers, or any vehicle needing telemetry
5. **Minimal Boilerplate**: Simple integration with sensible defaults

---

## Getting Started

Ready to integrate Astra? Start with:

1. **[Installation](installation.md)** - Set up your PlatformIO project
2. **[Basic Usage](basic-use.md)** - Create your first Astra system
3. Explore the interface and utility documentation as needed

---
