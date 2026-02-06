# Sensor Interface

`Sensor` is the base class for all hardware sensors in Astra. It inherits from `DataReporter`, so every sensor can publish telemetry automatically.

---

## Core Concepts

- **`begin()`** initializes hardware
- **`update()`** reads fresh data
- **`setUpdateRate(hz)`** controls how often the sensor should update
- **`isHealthy()`** indicates whether the sensor’s data can be trusted

Sensors implement **`init()`** and **`read()`** internally. `begin()` and `update()` call these for you.

---

## Key Methods

```cpp
int begin();
int update(double currentTime = -1);
void setUpdateRate(double hz);
bool isHealthy() const;
bool isInitialized() const;
```

---

## Update Rate

Sensors decide when to update based on `setUpdateRate()` and `shouldUpdate(currentTime)`.  
`SensorManager` calls `update()` only when the interval has elapsed.

```cpp
imu.setUpdateRate(100);  // 100 Hz
baro.setUpdateRate(20);  // 20 Hz
gps.setUpdateRate(5);    // 5 Hz
```

---

## Health Tracking

Base sensor types (Accel/Gyro/Mag/Baro) detect:

- Read failures
- Stuck readings (values not changing)

`isHealthy()` will return `false` if the sensor is unreliable.

---

## Rotatable Sensors

`Accel`, `Gyro`, `Mag`, and `IMU` inherit from `RotatableSensor`, which allows you to set board mounting orientation:

```cpp
imu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
```

---

## Implementing a Custom Sensor

```cpp
#include <Sensors/Sensor.h>

class MySensor : public astra::Sensor {
public:
    MySensor() : Sensor("MySensor") {
        addColumn("%.2f", &value, "value");
    }

    int begin() override { return init(); }
    int update(double currentTime = -1) override { return read(); }

protected:
    int init() override {
        // Hardware init
        return 0;
    }

    int read() override {
        value = 42.0f;
        return 0;
    }

private:
    float value = 0.0f;
};
```

---

## Available Built‑In Sensors

See the specific sensor pages:

- [Accelerometer](sensors/accel.md)
- [Gyroscope](sensors/gyro.md)
- [Magnetometer](sensors/mag.md)
- [Barometer](sensors/baro.md)
- [GPS](sensors/gps.md)
- [Voltage Sensor](sensors/voltage.md)
- [HITL Sensors](sensors/hitl.md)

