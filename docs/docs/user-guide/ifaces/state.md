# State

`State` is Astra’s math-only state estimator. It **does not own sensors**.  
Instead, it consumes vectors (gyro/accel/GPS/baro) and outputs position, velocity, acceleration, and orientation.

`Astra` drives the update flow automatically.

---

## Coordinate System

**Inertial frame: ENU**

- **X** = East  
- **Y** = North  
- **Z** = Up  

Orientation and position are expressed in this frame.

---

## Construction

You can build your own filter stack:

```cpp
#include <Filters/DefaultKalmanFilter.h>
#include <Filters/Mahony.h>

DefaultKalmanFilter kf;
MahonyAHRS ahrs;

State state(&kf, &ahrs);
```

Or use the default implementation:

```cpp
DefaultState state;
```

---

## Update API (Vector‑Based)

`State::update()` is deprecated. Use the vector API:

```cpp
void updateOrientation(const Vector<3>& gyro,
                       const Vector<3>& accel,
                       double dt);

void predict(double dt);

void updateMeasurements(const Vector<3>& gpsPos,
                        const Vector<3>& gpsVel,
                        double baroAlt,
                        bool hasGPS,
                        bool hasBaro);
```

`Astra` calls these for you. You generally don’t call them directly.

---

## Outputs

```cpp
Vector<3> getPosition() const;        // ENU position (m)
Vector<3> getVelocity() const;        // ENU velocity (m/s)
Vector<3> getAcceleration() const;    // ENU linear accel (m/s^2)
Quaternion getOrientation() const;    // Body → ENU
Vector<2> getCoordinates() const;     // GPS lat/lon
double getHeading() const;            // degrees
```

---

## DefaultState

`DefaultState` creates and owns:

- `DefaultKalmanFilter`
- `MahonyAHRS`

It’s the easiest way to get started:

```cpp
DefaultState state;
```

---

## Extending State

You can subclass `State` to add custom logic (launch detection, staging, etc.).

```cpp
class RocketState : public State {
public:
    RocketState(LinearKalmanFilter* f, MahonyAHRS* a) : State(f, a) {}

    void updateCustomLogic() {
        if (getAcceleration().z() > 20) {
            LOGI("Launch detected");
        }
    }
};
```

---

## Deprecated Methods

These remain for backward compatibility but return errors:

- `int update(double currentTime = -1)`
- `void predictState(double currentTime = -1)`

Use the vector-based API instead.

