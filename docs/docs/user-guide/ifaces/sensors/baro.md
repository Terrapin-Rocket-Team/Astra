# Barometer

`Barometer` standardizes pressure sensors and computes altitude automatically.

---

## Units

- Pressure: **hPa** (mbar)
- Temperature: **°C**
- Altitude: **meters ASL**

---

## Key Methods

```cpp
double getPressure() const;    // hPa
double getTemp() const;        // °C
double getTempF() const;       // °F
double getPressureAtm() const; // atm
double getASLAltM() const;     // meters
double getASLAltFt() const;    // feet
```

---

## Implementations

| Class | Notes |
|------|------|
| `DPS368` | DPS310-compatible (Adafruit driver) |
| `BMP390` | Bosch BMP3XX |
| `MS5611` | TE MS5611 |

Example:

```cpp
#include <Sensors/HW/Baro/DPS368.h>

DPS368 baro("Baro", &Wire, 0x77);
```

---

## Altitude Calculation

Altitude is computed internally with:

```
alt_m = 44307.69 * (1 - (pressure / 1013.25) ^ 0.190284)
```

Where pressure is in **hPa**.

---

## Logging Columns

By default, barometers log:

- `Pres (hPa)`
- `Temp (C)`
- `Alt ASL (m)`
