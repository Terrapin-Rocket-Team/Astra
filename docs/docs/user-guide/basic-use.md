# Basic Usage

This guide shows the current (v0.2+) Astra workflow:

1. Instantiate sensors
2. Create a `State` (usually `DefaultState`) or let Astra create one for you
3. Configure logging sinks
4. Build `AstraConfig` with sensors and state
5. Call `init()` and `update()`

---

## 1. Create Sensors

```cpp title="src/main.cpp"
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
```

If you use a 9-DoF IMU (e.g. BNO055), use `with9DoFIMU()` later.

---

## 2. Create a State

For most projects, start with `DefaultState`:

```cpp
DefaultState state;
```

If you want full control, you can pass your own filters to `State`:

```cpp
#include <Filters/DefaultKalmanFilter.h>
#include <Filters/Mahony.h>

DefaultKalmanFilter kf;
MahonyAHRS ahrs;
State state(&kf, &ahrs);
```

---

## 3. Configure Logging (Optional but Recommended)

Event logs are configured separately from telemetry logs. You can either call `EventLogger::configure()` yourself or pass sinks to `withEventLogs()`.

```cpp
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

UARTLog eventLog(Serial, 115200, true);
ILogSink* eventSinks[] = { &eventLog };

// Option A: configure manually
// EventLogger::configure(eventSinks, 1);
```

Telemetry logs (CSV) are configured via `AstraConfig`:

```cpp
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

FileLogSink telemFile("TELEM.csv", StorageBackend::SD_CARD, true);
ILogSink* telemSinks[] = { &telemFile };
```

---

## 4. Build `AstraConfig`

```cpp
AstraConfig config = AstraConfig()
    .with6DoFIMU(&imu)
    .withBaro(&baro)
    .withGPS(&gps)
    .withState(&state)           // Optional if you want DefaultState automatically
    .withBBPin(LED_BUILTIN)
    .withBuzzerPin(13)
    .withDataLogs(telemSinks, 1)
    .withEventLogs(eventSinks, 1);
```

If you have a separate magnetometer:

```cpp
#include <Sensors/HW/Mag/MMC5603NJ.h>

MMC5603NJ mag;

config.withMag(&mag);
```

---

## 5. Initialize and Update

```cpp
Astra sys(&config);

void setup() {
    Serial.begin(115200);
    int err = sys.init();
    if (err != 0) {
        LOGE("Astra init failed with %d error(s)", err);
    }
}

void loop() {
    sys.update();  // Uses millis() internally
}
```

---

## 6. Access State Data

```cpp
Vector<3> pos = state.getPosition();
Vector<3> vel = state.getVelocity();
Quaternion att = state.getOrientation();
```

---

## Sensor Update Rates

There is no global update rate. Each sensor controls its own rate:

```cpp
imu.setUpdateRate(100);  // 100 Hz
baro.setUpdateRate(20);  // 20 Hz
gps.setUpdateRate(5);    // 5 Hz
```

`Astra::update()` can run as fast as your loop; sensors decide when to read.

---

## Common Patterns

- **IMU-only orientation**: `with6DoFIMU()` + `DefaultState` gives fast attitude estimation.
- **Baro + GPS fusion**: add `withBaro()` and `withGPS()` for position/velocity corrections.
- **Extra sensors for logging**: use `withMiscSensor()` to log custom sensors without feeding State.

---

## Command System (SerialMessageRouter)

`Astra` creates a `SerialMessageRouter` internally and listens for:

```
CMD/HEADER
```

Sending `CMD/HEADER` on a serial interface prints the current telemetry header to that stream.

Add your own commands via `getMessageRouter()`:

```cpp
SerialMessageRouter* router = sys.getMessageRouter();
router->withListener("CMD/", myHandler);
```

For more details, see [SerialMessageRouter](utils/serial-router.md).

---

## Next Steps

- [Astra System](utils/mmfssys.md)
- [Logging](utils/logger.md)
- [State](ifaces/state.md)
- [Sensor Interface](ifaces/sensor.md)
