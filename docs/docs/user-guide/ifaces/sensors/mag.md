# Magnetometer

`Mag` is the base class for magnetometers.

---

## Key Methods

```cpp
Vector<3> getMag() const; // microtesla (uT)
```

---

## Implementations

| Class | Notes |
|------|------|
| `MMC5603NJ` | Adafruit MMC5603 |

Example:

```cpp
#include <Sensors/HW/Mag/MMC5603NJ.h>

MMC5603NJ mag("Mag", &Wire);
```

---

## With an IMU

If your IMU provides magnetometer data:

```cpp
IMU9DoF* imu = ...;
Mag* mag = imu->getMagSensor();
```
