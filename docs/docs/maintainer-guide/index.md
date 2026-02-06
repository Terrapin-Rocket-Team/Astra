# Maintainer Guide

This section is for contributors maintaining Astra.

---

## Documentation Workflow

Docs are powered by MkDocs + Material.

```bash
mkdocs serve
```

Edit pages under `docs/docs/`, then verify locally.

---

## Versioning

When you cut a release:

1. Update `library.json` version
2. Update `docs/mkdocs.yml` `repo_version`
3. Tag the release in Git

---

## Adding a Sensor

1. Create a class derived from the appropriate base (`Accel`, `Gyro`, `Barometer`, etc.)
2. Implement `init()` and `read()` (return 0 on success)
3. Register telemetry columns in the constructor
4. Add documentation in `docs/docs/user-guide/ifaces/sensors/`

---

## Logging Integration

- Sensors and State auto‑register via `DataReporter`
- `EventLogger::configure()` must be called to emit logs

---

## Internal APIs

- [SensorManager (Internal)](sensor-manager.md)

---

## Tests

Tests are under `test/` and run with PlatformIO:

```bash
pio test -e native
```

---

## Style Notes

- Avoid dynamic allocation in hot paths
- Prefer `Vector`/`Matrix` over ad‑hoc math
- Keep sensor updates non‑blocking
