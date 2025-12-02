---
title: Home
hide: footer
---

# Welcome to Astra Documentation

Welcome to the official documentation for **Astra**—a flexible and robust flight software framework developed by the Terrapin Rocket Team (TRT). Whether you're building a rocket, drone, rover, or any other vehicle requiring reliable telemetry and data logging, Astra provides the foundation you need.

---

## What is Astra?

Astra is a modular flight software library designed for diverse vehicle platforms. Built with adaptability and reliability in mind, it handles sensor integration, state estimation, data logging, and system coordination—letting you focus on your mission-specific logic.

**Key Capabilities:**

- **Multi-Platform Support:** Works on STM32, Teensy, and ESP32 microcontrollers
- **Flexible Storage:** Supports SD cards (SPI/SDIO), internal flash, and eMMC depending on your platform
- **Sensor Abstraction:** Unified interfaces for IMUs, barometers, GPS, encoders, and more
- **Dual Logging System:** Separate data logging (CSV telemetry) and event logging (human-readable status messages)
- **Vehicle Agnostic:** Not just for rockets—use it for any vehicle that needs sensor fusion and logging

---

## Documentation Overview

This documentation is organized to help you get started quickly and dive deeper as needed:

### **User Guide**
- Installation and setup instructions
- Basic usage patterns and examples
- Interface documentation for sensors, state, filters, and data reporters
- Utility documentation for logging, BlinkBuzz, math libraries, and more

### **Maintainer Guide**
- Architecture and design principles
- Contributing guidelines
- Best practices for extending Astra

### **Troubleshooting & FAQ**
- Common issues and solutions
- Platform-specific notes

---

## Features at a Glance

- **Modular Design:** Use only the components you need
- **Type-Safe Sensor System:** Compile-time type checking with runtime flexibility
- **Configurable Logging:** Control logging rates, formats, and destinations
- **Efficient Data Recording:** Optimized for high-frequency telemetry
- **Async Utilities:** Non-blocking LED and buzzer patterns with BlinkBuzz
- **Math Libraries:** Built-in vectors, matrices, and quaternions for state estimation

---

## Ready to Start?

Head to the [Installation](user-guide/installation.md) guide to get Astra set up, then check out [Basic Usage](user-guide/basic-use.md) to learn how to integrate it into your project.

---
