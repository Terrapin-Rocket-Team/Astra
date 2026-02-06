# SITL (Software‑In‑The‑Loop) Guide

SITL runs Astra natively on your computer and exchanges HITL sensor data over TCP.

---

## Architecture

```
Simulator (Python/MATLAB) → TCP → Native Serial Mock → SerialMessageRouter → HITLParser → HITL Sensors
                                                                                ↓
                                                                              Astra
                                                                                ↓
                                                                           DataLogger
```

---

## Quick Start

### 1) Start the Simulator

```bash
python sitl_simulator.py --sim parabolic
```

### 2) Build + Run Native

```bash
pio run -e native
.pio/build/native/program
```

---

## Minimal SITL Loop (Native)

```cpp
#include <Sensors/HITL/HITL.h>
#include <Communication/SerialMessageRouter.h>
#include <Utils/Astra.h>
#include <State/DefaultState.h>

using namespace astra;

HITLBarometer baro;
HITLAccel accel;
HITLGyro gyro;
HITLMag mag;
HITLGPS gps;

DefaultState state;
AstraConfig config = AstraConfig()
    .withAccel(&accel)
    .withGyro(&gyro)
    .withMag(&mag)
    .withBaro(&baro)
    .withGPS(&gps)
    .withState(&state)
    .withHITL(true);

Astra sys(&config);
SerialMessageRouter router;

void setup() {
    Serial.connectSITL("localhost", 5555);
    sys.init();

    router.withInterface(&Serial)
          .withListener("HITL/", [](const char* msg, const char*, Stream*) {
              double simTime;
              if (HITLParser::parse(msg, simTime)) {
                  sys.update(simTime);
              }
          });
}

void loop() {
    router.update();
}
```

---

## Notes

- `Serial.connectSITL()` is provided by the native Serial mock
- HITL messages use the format described in `src/Sensors/HITL/README.md`
- See `examples/SITL_Example/` for a complete implementation

