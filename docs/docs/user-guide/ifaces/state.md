# State

The `State` class is Astra's central state estimation component. It manages sensors, performs sensor fusion, and provides a unified interface to your vehicle's position, velocity, acceleration, and orientation. You can use it directly or extend it to add custom behavior.

---

## Overview

The `State` class handles:

1. **Sensor Management** – Initializes and updates all registered sensors
2. **Sensor Fusion** – Optionally uses a `Filter` (e.g., Kalman filter) to estimate state from multiple sensors
3. **State Estimation** – Provides position, velocity, acceleration, orientation, and GPS coordinates
4. **Data Logging** – Automatically registers state variables for CSV telemetry

Unlike earlier versions of Astra, there's no required `determineStage()` method—you only extend `State` if you need custom logic.

---

## Constructor

```cpp
State(Sensor **sensors, int numSensors, Filter *filter);
```

**Parameters:**
- `sensors`: Array of pointers to sensor objects (IMU, GPS, Barometer, etc.)
- `numSensors`: Number of sensors in the array
- `filter`: Optional pointer to a `Filter` for sensor fusion (use `nullptr` if not needed)

**Example:**
```cpp
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;
Sensor *sensors[] = {&gps, &imu, &baro};

State vehicleState(sensors, 3, nullptr);
```

---

## Lifecycle Methods

### begin()

```cpp
bool begin();
```

Initializes all sensors. Returns `false` if any sensor fails to initialize. Failed sensors are automatically disabled and logged.

This is called automatically by `Astra::init()` if you're using the main system.

### update()

```cpp
void update(double currentTime = -1);
```

Updates the state with fresh sensor data. If `currentTime` is not provided, it uses `millis() / 1000.0`.

This method:
1. Updates the internal time tracking
2. Polls all enabled sensors via `updateSensors()`
3. Runs sensor fusion if a filter is configured
4. Updates state variables (position, velocity, acceleration, orientation)
5. Updates GPS coordinates and heading

This is called automatically by `Astra::update()` at the configured rate.

---

## State Variables

The `State` class maintains the following estimated quantities:

| Variable | Type | Units | Description |
|----------|------|-------|-------------|
| `position` | `Vector<3>` | meters | XYZ displacement from launch origin |
| `velocity` | `Vector<3>` | m/s | XYZ velocity |
| `acceleration` | `Vector<3>` | m/s² | XYZ acceleration |
| `orientation` | `Quaternion` | - | Vehicle attitude |
| `coordinates` | `Vector<2>` | degrees | GPS latitude and longitude |
| `heading` | `double` | degrees | Direction of travel |

### Accessing State Variables

Use the provided getter methods:

```cpp
Vector<3> pos = vehicleState.getPosition();        // [x, y, z] in meters
Vector<3> vel = vehicleState.getVelocity();        // [vx, vy, vz] in m/s
Vector<3> accel = vehicleState.getAcceleration();  // [ax, ay, az] in m/s²
Quaternion orient = vehicleState.getOrientation(); // Attitude quaternion
Vector<2> coords = vehicleState.getCoordinates();  // [lat, lon] in degrees
double heading = vehicleState.getHeading();        // Heading in degrees
```

These can be accessed from anywhere in your code after `update()` has been called.

---

## Sensor Access

### getSensor()

```cpp
Sensor *getSensor(SensorType type, int sensorNum = 1) const;
```

Retrieves a sensor by type. The `sensorNum` parameter is 1-indexed (1 gets the first sensor of that type, 2 gets the second, etc.).

**Available sensor types:**
- Defined by compile-time hashing of sensor type strings
- Access via the sensor's `getType()` method
- Common types: IMU, GPS, Barometer, Encoder, LightSensor

**Example:**
```cpp
Sensor *rawSensor = vehicleState.getSensor(imu.getType());
IMU *imu = dynamic_cast<IMU*>(rawSensor);
if (imu) {
    Vector<3> accel = imu->getAccelerationGlobal();
}
```

### sensorOK()

```cpp
bool sensorOK(const Sensor *sensor) const;
```

Checks if a sensor is initialized and functioning. Always check this before using sensor data.

**Example:**
```cpp
Sensor *gpsPtr = vehicleState.getSensor(gps.getType());
if (vehicleState.sensorOK(gpsPtr)) {
    GPS *gps = dynamic_cast<GPS*>(gpsPtr);
    Vector<2> coords = gps->getCoordinates();
}
```

### getSensors()

```cpp
Sensor **getSensors() const;
int getNumMaxSensors() const;
```

Get direct access to the sensor array and count. Useful for iterating over all sensors.

---

## Kalman Filter Integration

If you provide a `Filter` object to the constructor, `State` will use it for sensor fusion:

```cpp
MyKalmanFilter filter;
State vehicleState(sensors, 3, &filter);
```

The filter is called during `update()` to estimate state from noisy sensor measurements. See the [Filter interface documentation](filters.md) for implementation details.

If no filter is provided, `State` uses simple sensor data directly without fusion.

---

## Extending State for Custom Logic

While the base `State` class works for many applications, you may want to add vehicle-specific logic. Simply extend it:

### Example: Launch and Landing Detection

```cpp
#ifndef ROCKET_STATE_H
#define ROCKET_STATE_H

#include <State/State.h>
#include "RecordData/Logging/EventLogger.h"

using namespace astra;

class RocketState : public State {
public:
    RocketState(Sensor **sensors, int numSensors, Filter *filter);

    void update(double currentTime = -1) override;

    int getStage() const { return stage; }

private:
    int stage = 0;  // 0=pad, 1=boost, 2=coast, 3=descent, 4=landed
    double launchTime = 0;

    void detectLaunch();
    void detectApogee();
    void detectLanding();
};

#endif
```

```cpp
#include "RocketState.h"

RocketState::RocketState(Sensor **sensors, int numSensors, Filter *filter)
    : State(sensors, numSensors, filter) {}

void RocketState::update(double currentTime) {
    // Call parent to update sensors and state variables
    State::update(currentTime);

    // Add custom stage detection logic
    detectLaunch();
    detectApogee();
    detectLanding();
}

void RocketState::detectLaunch() {
    if (stage == 0 && acceleration.z() > 20) {
        stage = 1;
        launchTime = currentTime;
        LOGI("Launch detected at %.2f m/s²", acceleration.z());

        // Disable sensor bias correction after launch
        for (int i = 0; i < numSensors; i++) {
            sensors[i]->useBiasCorrection(false);
        }
    }
}

void RocketState::detectApogee() {
    if (stage == 1 && velocity.z() < 0) {
        stage = 2;
        LOGI("Apogee at %.1f m AGL", position.z());
    }
}

void RocketState::detectLanding() {
    if ((stage == 2 || stage == 3) &&
        abs(velocity.z()) < 1.0 &&
        position.z() < 5.0) {
        stage = 4;
        LOGI("Landing detected after %.1f seconds", currentTime - launchTime);
    }
}
```

Then use your custom state:

```cpp
RocketState rocketState(sensors, 3, &filter);

AstraConfig config = AstraConfig()
                        .withState(&rocketState)
                        .withUsingSensorBiasCorrection(true);

Astra system(&config);

void loop() {
    system.update();

    // Access custom methods
    int currentStage = rocketState.getStage();
    if (currentStage == 2) {
        // Coasting - deploy drogue?
    }
}
```

### Example: Multi-Vehicle Support

```cpp
class DroneState : public State {
public:
    DroneState(Sensor **sensors, int numSensors, Filter *filter)
        : State(sensors, numSensors, filter) {}

    void update(double currentTime = -1) override {
        State::update(currentTime);

        // Drone-specific logic
        maintainHover();
        checkBatteryLevel();
    }

    Vector<3> getTargetPosition() const { return targetPos; }
    void setTargetPosition(Vector<3> target) { targetPos = target; }

private:
    Vector<3> targetPos;

    void maintainHover() {
        // Position control logic
    }

    void checkBatteryLevel() {
        // Battery monitoring
    }
};
```

---

## Coordinate System

Astra uses the following conventions:

**Local Frame (position, velocity, acceleration):**
- **X**: East
- **Y**: North
- **Z**: Up (altitude)
- Origin set at the first GPS fix

**Global Frame (orientation):**
- Quaternion representing vehicle attitude relative to Earth frame
- Can be converted to Euler angles (roll, pitch, yaw) using quaternion methods

**GPS Frame (coordinates):**
- Latitude and longitude in decimal degrees
- Altitude in meters (if available from GPS)

---

## Data Logging

`State` automatically registers its variables for CSV telemetry logging via the `DataReporter` interface. When you pass your state to `AstraConfig`, its columns are automatically added to the telemetry file:

**Logged columns:**
- `State - position_x`, `State - position_y`, `State - position_z`
- `State - velocity_x`, `State - velocity_y`, `State - velocity_z`
- `State - acceleration_x`, `State - acceleration_y`, `State - acceleration_z`
- `State - orientation_w`, `State - orientation_x`, `State - orientation_y`, `State - orientation_z`
- `State - lat`, `State - lon`
- `State - heading`

If you extend `State` and add custom variables you want logged, use the `addColumn()` method from `DataReporter`:

```cpp
class CustomState : public State {
public:
    CustomState(Sensor **sensors, int numSensors, Filter *filter)
        : State(sensors, numSensors, filter) {
        addColumn("%d", &stage, "stage");
        addColumn("%.2f", &batteryVoltage, "battery_v");
    }

private:
    int stage = 0;
    float batteryVoltage = 0.0f;
};
```

---

## Protected Members for Extension

When extending `State`, you have access to these protected members:

| Member | Type | Description |
|--------|------|-------------|
| `currentTime` | `double` | Current time in seconds |
| `lastTime` | `double` | Previous update time |
| `sensors` | `Sensor**` | Array of all sensors |
| `numSensors` | `int` | Count of enabled sensors |
| `maxNumSensors` | `int` | Total sensor count (including disabled) |
| `position` | `Vector<3>` | Current position estimate |
| `velocity` | `Vector<3>` | Current velocity estimate |
| `acceleration` | `Vector<3>` | Current acceleration estimate |
| `orientation` | `Quaternion` | Current orientation |
| `coordinates` | `Vector<2>` | GPS coordinates |
| `heading` | `double` | Direction of travel |
| `origin` | `Vector<3>` | GPS origin point |
| `filter` | `Filter*` | Optional Kalman filter |

You can read and modify these in your overridden methods. Be cautious when modifying state variables directly—they're normally updated by sensor fusion.

---

## Best Practices

1. **Always call parent update()**: When overriding `update()`, call `State::update(currentTime)` first to ensure sensors are read and state is estimated.

2. **Check sensor validity**: Use `sensorOK()` before accessing sensor data to avoid null pointer issues.

3. **Use appropriate coordinate frames**: Remember that `position` is relative to the GPS origin, while `coordinates` are absolute lat/lon.

4. **Log important events**: Use `LOGI()`, `LOGW()`, `LOGE()` macros to record state transitions and important detections.

5. **Disable bias correction at launch**: If using sensor bias correction, disable it when you detect launch to avoid corrupting flight data.

6. **Keep update() fast**: The `update()` method is called frequently. Keep custom logic efficient.

---

## Summary

- `State` manages sensors and estimates vehicle state
- Provides position, velocity, acceleration, orientation, and GPS data
- Can be used directly or extended for custom behavior
- Integrates with Kalman filters for sensor fusion
- Automatically logs telemetry via `DataReporter`
- No required methods to override—extend only if needed

For usage examples, see [Basic Usage](../basic-use.md). For filter integration, see [Filters](filters.md).

---
