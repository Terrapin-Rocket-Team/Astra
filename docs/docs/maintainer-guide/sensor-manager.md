# SensorManager (Internal)

`SensorManager` owns the active sensor set and provides access to sensor data and update flags.
It is an internal utility used by `Astra` and `AstraConfig`.

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

Most users should not interact with `SensorManager` directly; `Astra` manages it internally.

If you need direct access (custom loop or testing), instantiate and populate it manually in your own code or tests.
