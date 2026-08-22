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

This library is designed for PlatformIO. For a new development machine, first
install the shared support CLI and a native C++ compiler by following the
[Astra-Support setup instructions](https://github.com/Terrapin-Rocket-Team/Astra-Support#install-cli).

From the Astra repository root, verify the toolchain and run the maintained
build/test matrix with:

```bash
astra-support doctor --project .
astra-support test --project . --clean --no-progress
```

The test command downloads the required PlatformIO platforms and library
dependencies. The initial run can take several minutes.

See the [installation guide](docs/docs/user-guide/installation.md) for consumer
project examples and these required build flags:

- `ENV_TEENSY`
- `ENV_STM`
- `ENV_ESP`
- `NATIVE=1` (SITL)

## Docs

Start here:
- `docs/docs/index.md`
- `docs/docs/user-guide/installation.md`
- `docs/docs/user-guide/basic-use.md`

## Contributor Checks

The supported handoff check is:

```bash
astra-support test --project . --clean --no-progress
```

It builds the Teensy 4.1, STM32H723, ESP32-S3, and native environments and runs
the native Unity test suites. Hardware upload and HITL tests require the
corresponding device and are not part of this command.
