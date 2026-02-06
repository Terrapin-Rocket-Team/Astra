# GPS

`GPS` provides position, velocity, heading, and time-of-day data.

---

## Key Methods

```cpp
Vector<3> getPos() const;   // lat, lon, alt (degrees, degrees, meters)
Vector<3> getVel() const;   // NED velocity (m/s)
double getHeading() const;  // degrees
bool getHasFix() const;     // true when fix quality >= 4
int getFixQual() const;     // satellites

Vector<3> getDisplacement(Vector<3> origin) const;
```

Time accessors:

```cpp
const char* getTimeOfDay() const; // "HH:MM:SS"
int8_t getHour() const;
int8_t getMinute() const;
int8_t getSecond() const;
uint8_t getDay() const;
uint8_t getMonth() const;
uint16_t getYear() const;
```

---

## Implementations

| Class | Notes |
|------|------|
| `MAX_M10S` | u-blox M10S |
| `SAM_M10Q` | Alias of MAX_M10S |

Example:

```cpp
#include <Sensors/HW/GPS/MAX_M10S.h>

MAX_M10S gps("GPS", Wire, 0x42);
```

---

## Logging Columns

GPS logs:

- `Lat (deg)`
- `Lon (deg)`
- `Alt (m)`
- `vN (m/s)`
- `vE (m/s)`
- `vD (m/s)`
- `Fix Quality`
- `Time of Day`

