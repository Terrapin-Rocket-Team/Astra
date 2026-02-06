# Accelerometer

`Accel` is the base class for 3‑axis accelerometers.

---

## Key Methods

```cpp
Vector<3> getAccel() const; // m/s^2
bool isHealthy() const;
```

`Accel` tracks stuck readings and will mark itself unhealthy if values stop changing.

---

## Implementations

| Class | Notes |
|------|------|
| `ADXL375` | High‑g ADXL375 |
| `H3LIS331DL` | High‑g LIS331 |

Example:

```cpp
#include <Sensors/HW/Accel/ADXL375.h>

ADXL375 accel("HighG", Wire, 0x1D);
```

---

## DualRangeAccel

`DualRangeAccel` combines a low‑g and high‑g sensor with a blended transition region.

```cpp
DualRangeAccel combo(&lowG, &highG, 150.0, 120.0);
```

- `maxLowG` and `minHighG` define the blend window  
- Values are in the same units as `getAccel()` (**m/s²**)
- Each underlying sensor keeps its own mounting orientation
