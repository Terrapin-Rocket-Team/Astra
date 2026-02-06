# SITL (Software‑In‑The‑Loop)

SITL runs Astra natively on your PC and connects to a simulator over TCP.

---

## Quick Start

1. Start the simulator:
   ```bash
   python sitl_simulator.py --sim parabolic
   ```

2. Build for native:
   ```bash
   pio run -e native
   ```

3. Run the program:
   ```bash
   .pio/build/native/program.exe
   ```

---

## Notes

- SITL uses the HITL message format over TCP
- The included example is in `examples/SITL_Example`
- Use `Serial.connectSITL(host, port)` to connect

