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
#include <Utils/Astra.h>
#include <State/DefaultState.h>

using namespace astra;

DefaultState state;
AstraConfig config = AstraConfig()
    .withState(&state);

Astra sys(&config);

void setup() {
    sys.init();
}

void loop() {
    sys.update();
}
```

---

## Notes

- Native Astra runs auto-connect to the configured SITL endpoint and wait until a simulator is available
- Native Astra also owns `HITL/` routing internally; callers just run `sys.update()`
- Call `withHITL()` explicitly on native only if you want the default HITL sensors materialized early so a decorator can wrap them before `init()`
- HITL messages use the format described in `src/Sensors/HITL/README.md`
- See `examples/SITL_Example/` for a complete implementation

