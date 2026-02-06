# Astra
### Terrapin Rocket Team's universal flight software library

Astra is a modular flight software library for embedded systems. It provides sensor abstraction, state estimation, logging, and HITL/SITL testing utilities so you can focus on mission logic instead of plumbing.

## What You Get
- Multi-sensor support (IMU, baro, GPS, mag, voltage, HITL)
- State estimation (Mahony AHRS + Kalman filter)
- Unified logging (CSV telemetry + event logs)
- Multi-platform support (STM32, Teensy, ESP32, native SITL)

## Quick Start (v0.2+)

```cpp
#include <Arduino.h>
#include <Utils/Astra.h>
#include <State/DefaultState.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/HW/GPS/MAX_M10S.h>

using namespace astra;

BMI088 imu;
DPS368 baro;
MAX_M10S gps;
DefaultState state;

AstraConfig config = AstraConfig()
    .with6DoFIMU(&imu)
    .withBaro(&baro)
    .withGPS(&gps)
    .withState(&state);

Astra sys(&config);

void setup() {
    Serial.begin(115200);
    sys.init();
}

void loop() {
    sys.update();
}
```

## Platform Setup

This library is designed for PlatformIO. See the docs for example `platformio.ini` configs and build flags:

- `ENV_TEENSY`
- `ENV_STM`
- `ENV_ESP`
- `NATIVE=1` (SITL)

## Docs

Start here:
- `docs/docs/index.md`
- `docs/docs/user-guide/installation.md`
- `docs/docs/user-guide/basic-use.md`
