# Astra System

`Astra` is the high-level orchestrator. It wires sensors, state estimation, logging, and indicators into a single update loop.

---

## Minimal Setup

```cpp
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
    .withState(&state);  // Optional: Astra will use DefaultState if not provided

Astra sys(&config);
```

Call `sys.init()` in `setup()` and `sys.update()` in `loop()`.

---

## AstraConfig Overview

### Required

**`withState(State* state)`**

Provide a `State` (or `DefaultState`). Astra uses it for orientation and position estimation.

---

### Sensor Configuration

Provide sensors directly:

- `withAccel(Accel*)`
- `withGyro(Gyro*)`
- `withMag(Mag*)`
- `withBaro(Barometer*)`
- `withGPS(GPS*)`
- `withMiscSensor(Sensor*)` (logged, not used for state)

Convenience for IMUs:

- `with6DoFIMU(IMU6DoF*)` – extracts accel + gyro
- `with9DoFIMU(IMU9DoF*)` – extracts accel + gyro + mag

The IMU itself is added as a misc sensor for logging.

---

### Logging Sinks

**`withDataLogs(ILogSink** sinks, uint8_t count)`**

Configures telemetry CSV sinks for `DataLogger`. Event logs are configured separately via `EventLogger::configure()`.

**`withEventLogs(ILogSink** sinks, uint8_t count)`**

Configures sinks for `EventLogger` during `Astra::init()`.

---

### Logging Cadence

```cpp
config.withLoggingRate(20);      // Hz
config.withLoggingInterval(50);  // ms
```

`Astra` evaluates logging on a seconds-based clock. Use `withLoggingInterval()` if you want an explicit interval.

---

### BlinkBuzz

- `withBuzzerPin(uint)` – sets global `BUZZER` and registers pin
- `withBBPin(uint)` – registers a pin
- `withBBAsync(bool enable, uint queueSize = 50)`

---

### Status Indicators

Optional diagnostic LEDs/buzzers:

- `withStatusLED(int pin)`
- `withStatusBuzzer(int pin)`
- `withGPSFixLED(int pin)`

Init feedback patterns:

- **Solid ON** = all sensors initialized
- **N blinks** = single sensor failure (1=Accel, 2=Gyro, 3=Mag, 4=Baro, 5=GPS, 6=Misc)
- **Rapid blink** = multiple failures

---

### HITL Mode

**`withHITL(bool enabled)`**

Flags the system as HITL. Use this when you pass simulation time into `update(simTimeSeconds)`.

---

### Ownership Model

- `Astra` stores a pointer to `AstraConfig`. Ensure the config outlives the `Astra` instance.
- `Astra` does **not** own sensors or log sinks. You are responsible for their lifetime.
- If you call `withState()`, you own that `State`.
- If you do **not** call `withState()`, `Astra` creates a `DefaultState` and owns it.
- `SerialMessageRouter` is created and owned by `Astra`.

---

## Astra API

### `int init()`

Initializes BlinkBuzz, logging sinks, sensors, and state.

Returns the number of sensor initialization failures (0 = success).

### `bool update(double timeSeconds = -1)`

Runs one system cycle:

- Updates `SerialMessageRouter`
- Updates BlinkBuzz
- Updates sensors (as each sensor’s update interval allows)
- Updates orientation + Kalman prediction
- Updates measurement corrections (GPS/baro)
- Logs telemetry at the configured logging interval

If `timeSeconds` is `-1`, Astra uses `millis()/1000.0`.

---

## SerialMessageRouter Integration

Astra creates a `SerialMessageRouter` internally and registers:

```
CMD/HEADER
```

Sending `CMD/HEADER` on a serial interface prints the current telemetry header to that stream.

You can access the router to add your own listeners:

```cpp
SerialMessageRouter* router = sys.getMessageRouter();
router->withListener("CMD/", myHandler);
```

Because Astra updates the router internally, you do not need to call `router->update()` yourself.

---

## Diagnostics Helpers

`Astra` exposes flags for debugging:

- `didLog()`
- `didUpdateState()`
- `didPredictState()`

These are updated each cycle by `update()`.  
For per-sensor update events in custom loops or tests, see the Maintainer Guide.
