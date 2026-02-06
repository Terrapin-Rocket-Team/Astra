# SensorManager

`SensorManager` owns the active sensor set and provides clean access to sensor data.

---

## Responsibilities

- Calls `begin()` on each configured sensor
- Updates sensors based on `shouldUpdate()`
- Tracks which sensors produced new data this cycle
- Tracks init failures and health status

---

## Key Methods

```cpp
void update(double currentTime);

bool hasAccelUpdate() const;
bool hasGyroUpdate() const;
bool hasBaroUpdate() const;
bool hasGPSUpdate() const;

Vector<3> getAcceleration() const;
Vector<3> getAngularVelocity() const;
double getBarometricAltitude() const;
Vector<3> getGPSPosition() const;
Vector<3> getGPSVelocity() const;
```

---

## Typical Usage

Most users do not interact with `SensorManager` directly; `Astra` manages it internally.

If you need direct access (custom loop or testing), instantiate and populate it manually.

