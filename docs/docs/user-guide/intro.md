---
title: User Manual - Introduction
---

# The Astra User Manual

This guide is written in the order shown in the sidebar. Each page stands alone, but later sections assume the basics.

If you’re unsure where to start, read [Installation](installation.md) and [Basic Usage](basic-use.md) first.

---

## How Astra Is Organized

Astra is built around three layers:

1. **System**: `Astra` + `AstraConfig` coordinate everything
2. **Interfaces**: `State`, `Sensor`, `DataReporter`, and filters define behavior
3. **Utilities**: Logging, BlinkBuzz, math types, and helpers

---

## High-Level Data Flow

```
Sensors → SensorManager → State → DataLogger
                    ↘ EventLogger (LOG/)
```

- Sensors update at their own rates
- `SensorManager` tracks updates and sensor health
- `State` consumes vectors (gyro/accel/GPS/baro) and estimates pose
- `DataLogger` writes CSV telemetry from all `DataReporter`s

---

## Key Concepts

**DataReporter**
- Any class that registers columns for telemetry logging
- Sensors and State inherit from it automatically

**Sensor Health**
- Base sensor classes track communication failures and stuck readings
- `SensorManager` checks health before using data

**Orientation + Position**
- Orientation is estimated with Mahony AHRS
- Position/velocity are estimated with a linear Kalman filter

---

## Recommended Starting Point

If you want a working system quickly:

1. Use `DefaultState` (built-in filters)
2. Use `AstraConfig` with `with6DoFIMU()`, `withBaro()`, and `withGPS()`
3. Add log sinks and call `Astra::init()`/`Astra::update()`

See [Basic Usage](basic-use.md) for a complete example.

