# SITL (Software‑In‑The‑Loop)

SITL runs Astra natively on your PC and connects to a simulator over TCP.

---

## Quick Start

From the Astra repository, use the simulation runner installed by
[Astra-Support](https://github.com/Terrapin-Rocket-Team/Astra-Support):

```bash
astra-support sim list --project .
astra-support sim run --project . --mode sitl --source physics
```

The runner builds the `native` PlatformIO environment when its executable is
missing, starts the flight-software process, supplies simulated sensor packets,
and records the session. It reuses an existing executable; pass `--build` after
source or configuration changes. The compatibility shortcut is:

```bash
astra-support sitl -C . -s physics
```

---

## Notes

- SITL uses the HITL message format over TCP
- The native firmware example is in `examples/SITL_Example`
- Astra-Support is the maintained simulator and process launcher; Astra does not
  contain a standalone `sitl_simulator.py`
- Native Astra connects to the endpoint configured by the runner and waits until
  it is available
- Call `withHITL()` explicitly only when you want to override the default HITL sensors before `init()`

