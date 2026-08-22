# SITL Example

This example is the native Astra flight-software process used for
Software-In-The-Loop testing. It receives simulated sensor data over TCP,
updates Astra's HITL sensors and state estimator, and returns `TELEM/` data.

## Run It

Install
[Astra-Support](https://github.com/Terrapin-Rocket-Team/Astra-Support), then run
from the Astra repository root:

```bash
astra-support sim list --project .
astra-support sim run --project . --mode sitl --source physics
```

The runner builds the `native` PlatformIO environment, launches this example,
feeds it simulation packets, and records the session. The compatibility shortcut
is:

```bash
astra-support sitl -C . -s physics
```

The firmware intentionally calls only `g_sys.update()` in its loop. Astra owns
the `HITL/` listener, parses each packet, and performs the update using simulation
time. Do not add a second HITL listener or call the message router separately.

## Modify the Flight Software

Edit `SITL_Example.cpp` to change the configured state, log sinks, or other Astra
components. Project-specific simulation sources belong in an
`astra_support_sim.py` hook at the consumer project root; see the Astra-Support
README for that interface.

## Troubleshooting

- Run `astra-support doctor --project .` to check the toolchain.
- Use `astra-support sim list --project .` to confirm available sources.
- Check that TCP port 5555 is available if the native process cannot connect.
- Keep the packet format synchronized with `src/Sensors/HITL/README.md` when
  implementing a custom source.
