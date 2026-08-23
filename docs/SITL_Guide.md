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

Install Astra-Support as described in its
[README](https://github.com/Terrapin-Rocket-Team/Astra-Support), then run from
this repository:

```bash
astra-support sim list --project .
astra-support sim run --project . --mode sitl --source physics
```

The runner builds the native target when its executable is missing, starts it,
feeds lock-step simulation data, and writes the simulation log. It reuses an
existing executable; pass `--build` after source or configuration changes. The
equivalent compatibility shortcut is:

```bash
astra-support sitl -C . -s physics
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
- Astra-Support is the maintained simulation harness; this repository does not
  contain a standalone `sitl_simulator.py`

