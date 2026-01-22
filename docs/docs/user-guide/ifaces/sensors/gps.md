# GPS

The `GPS` class provides a standardized interface for satellite-based positioning modules in Astra. It extends the [`Sensor`](../sensor.md) interface with GPS-specific functionality including position tracking, fix quality monitoring, and displacement calculations from an origin point.

---

## Overview

GPS modules provide geographic location, altitude, velocity, and timing information. The `GPS` base class handles:

1. **Position tracking** - Latitude, longitude, and altitude
2. **Origin reference** - First valid fix becomes the reference point
3. **Displacement calculation** - Distance from origin in meters
4. **Fix quality monitoring** - Satellite count and fix status
5. **Time tracking** - UTC time from satellites

Implementing a new GPS module requires only defining `init()` and `read()` methods.

---

## Available Methods

### Position Data

```cpp
virtual Vector<3> getPos() const;              // [lat, lon, alt]
virtual Vector<3> getDisplacement(Vector<3> origin) const;  // XYZ distance in meters
```

Returns current position as latitude (degrees), longitude (degrees), and altitude (meters).

**Example:**
```cpp
MAX_M10S gps;
Vector<3> pos = gps.getPos();
// pos[0] = latitude (e.g., 39.123456)
// pos[1] = longitude (e.g., -77.654321)
// pos[2] = altitude (e.g., 150.5 meters)

Vector<3> displacement = gps.getDisplacement(launchSite);
// displacement[0] = east/west in meters
// displacement[1] = north/south in meters
// displacement[2] = altitude change in meters
```

### Fix Status

```cpp
virtual bool getHasFix() const;                // Currently has valid fix
virtual int getFixQual() const;                // Number of satellites
```

Check GPS status before using position data.

**Example:**
```cpp
if (gps.getHasFix()) {
    int numSats = gps.getFixQual();
    LOGI("GPS: %d satellites", numSats);
    Vector<3> pos = gps.getPos();
}
```

### Heading

```cpp
virtual double getHeading() const;             // Direction of travel in degrees
```

Returns compass heading (0-360°) based on GPS velocity vector.

### Time Data

```cpp
virtual const char *getTimeOfDay() const;      // "HH:MM:SS" string
virtual int8_t getHour() const;
virtual int8_t getMinute() const;
virtual int8_t getSecond() const;
virtual uint8_t getDay() const;
virtual uint8_t getMonth() const;
virtual uint16_t getYear() const;
```

GPS provides UTC time from satellites, useful for timestamping without an RTC.

**Example:**
```cpp
LOGI("Current time: %s", gps.getTimeOfDay());  // "14:35:22"
LOGI("Date: %d/%d/%d", gps.getMonth(), gps.getDay(), gps.getYear());
```

---

## Logged Data Columns

The `GPS` class automatically registers these columns for telemetry:

| Column | Format | Units | Description |
|--------|--------|-------|-------------|
| `latitude` | `%.7f` | degrees | WGS84 latitude |
| `longitude` | `%.7f` | degrees | WGS84 longitude |
| `altitude_m` | `%.1f` | meters | GPS altitude |
| `satellites` | `%d` | count | Number of satellites |
| `heading` | `%.1f` | degrees | Direction of travel |

**CSV example:**
```csv
...,GPS - latitude,GPS - longitude,GPS - altitude_m,GPS - satellites,GPS - heading
...,39.1234567,-77.6543210,150.5,12,45.3
```

---

## Displacement Calculation

The GPS class automatically tracks displacement from the first valid fix (origin). This uses a fast geodesic approximation based on [cheap-ruler](https://github.com/mapbox/cheap-ruler) for accurate local distance calculations.

The origin is set automatically on the first fix. Displacement is calculated as:

- **X (East)**: Positive east, negative west
- **Y (North)**: Positive north, negative south
- **Z (Up)**: Altitude change in meters

This makes it easy to track vehicle movement relative to launch site without complex geodesy.

---

## Implementing a Custom GPS

To add support for a new GPS module, inherit from `GPS` and implement `init()` and `read()`:

### Basic Structure

```cpp
#ifndef MY_GPS_H
#define MY_GPS_H

#include "Sensors/GPS/GPS.h"
#include <SomeGPSLibrary.h>

namespace astra {

class MyGPS : public GPS {
public:
    MyGPS(const char *name = "MyGPS");

protected:
    bool init() override;
    bool read() override;

private:
    SomeGPSDriver hardware;
};

}

#endif
```

### Implementation Example

```cpp
#include "MyGPS.h"

using namespace astra;

MyGPS::MyGPS(const char *name) : GPS(name) {
    // GPS columns are registered by parent
}

bool MyGPS::init() {
    // Initialize UART communication
    hardware.begin(9600);

    // Configure GPS module
    hardware.setUpdateRate(10);  // 10Hz
    hardware.enableGNSS(true);   // Enable all constellations

    LOGI("MyGPS: Initialized successfully");
    return true;
}

bool MyGPS::read() {
    // Read and parse GPS data
    if (!hardware.available()) {
        return false;
    }

    if (hardware.parse()) {
        // Check if we have a valid fix
        if (hardware.fixType() >= 3) {  // 3D fix
            // Update protected members
            position[0] = hardware.latitude();
            position[1] = hardware.longitude();
            position[2] = hardware.altitude();

            fixQual = hardware.numSatellites();
            hasFix = true;

            // Update heading if available
            if (hardware.hasSpeed() && hardware.speed() > 0.5) {
                heading = hardware.course();
            }

            // Update time
            hr = hardware.hour();
            min = hardware.minute();
            sec = hardware.second();
            day = hardware.day();
            month = hardware.month();
            year = hardware.year();

            return true;
        }
    }

    hasFix = false;
    return false;
}
```

### Important Notes

1. **Position units**: Latitude/longitude in decimal degrees, altitude in meters
2. **Fix validation**: Only update position when `fixType >= 3` (3D fix)
3. **Heading calculation**: Usually derived from velocity vector
4. **Time is UTC**: GPS time is always UTC, not local time

---

## Available Implementations

Astra includes drivers for common GPS modules:

### MAX_M10S

u-blox MAX-M10 GNSS module (GPS, GLONASS, Galileo, BeiDou).

```cpp
#include "Sensors/HW/GPS/MAX_M10S.h"

MAX_M10S gps;
```

**Specifications:**
- Cold start: <24s
- Accuracy: 2.0m CEP
- Update rate: Up to 25Hz
- Interface: UART, I2C, SPI

### SAM_M8Q

u-blox SAM-M8Q concurrent GNSS module.

```cpp
#include "Sensors/GPS/SAM_M8Q.h"

SAM_M8Q gps;
```

**Specifications:**
- Cold start: <26s
- Accuracy: 2.5m CEP
- Update rate: Up to 10Hz
- Interface: UART, I2C

---

## Usage Examples

### Basic Usage

```cpp
#include "Sensors/HW/GPS/MAX_M10S.h"

MAX_M10S gps;

void setup() {
    Serial.begin(115200);

    if (!gps.begin()) {
        Serial.println("GPS init failed!");
    }

    Serial.println("Waiting for GPS fix...");
}

void loop() {
    gps.update();

    if (gps.getHasFix()) {
        Vector<3> pos = gps.getPos();

        Serial.print("Position: ");
        Serial.print(pos[0], 7);  // Latitude
        Serial.print(", ");
        Serial.print(pos[1], 7);  // Longitude
        Serial.print(", ");
        Serial.print(pos[2], 1);  // Altitude
        Serial.println(" m");

        Serial.print("Satellites: ");
        Serial.println(gps.getFixQual());

        Serial.print("Time: ");
        Serial.println(gps.getTimeOfDay());
    } else {
        Serial.println("No fix");
    }

    delay(1000);
}
```

### Tracking Displacement from Origin

```cpp
MAX_M10S gps;
Vector<3> launchSite;
bool haveOrigin = false;

void loop() {
    gps.update();

    if (gps.getHasFix()) {
        if (!haveOrigin) {
            // Save first fix as origin
            launchSite = gps.getPos();
            haveOrigin = true;
            LOGI("Origin set: %.7f, %.7f", launchSite[0], launchSite[1]);
        } else {
            // Calculate displacement from origin
            Vector<3> displacement = gps.getDisplacement(launchSite);

            LOGI("Displacement: E:%.1f N:%.1f U:%.1f",
                displacement[0], displacement[1], displacement[2]);
        }
    }
}
```

---

## Integration with State

When you pass a GPS to `State`, it contributes to position and velocity estimation:

```cpp
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;

Sensor *sensors[] = {&gps, &imu, &baro};
State vehicleState(sensors, 3, nullptr);
```

The `State` class uses GPS data to:
- Set the position origin on first fix
- Provide horizontal position (X, Y)
- Provide altitude (Z) as a cross-check with barometer
- Calculate velocity from position changes
- Set the heading based on motion direction

---

## Coordinate Systems

### GPS Coordinates

GPS outputs in WGS84 geodetic coordinates:
- **Latitude**: -90° (South Pole) to +90° (North Pole)
- **Longitude**: -180° (West) to +180° (East)
- **Altitude**: Meters above WGS84 ellipsoid (not sea level!)

### Local Displacement

Displacement is in a local ENU (East-North-Up) frame:
- **X (East)**: Positive east, negative west
- **Y (North)**: Positive north, negative south
- **Z (Up)**: Positive up, negative down

This matches Astra's standard coordinate system.

---

## Fix Quality

GPS fix quality indicates positioning accuracy:

| Fix Type | Satellites | Description |
|----------|-----------|-------------|
| No fix | 0-3 | No valid position |
| 2D fix | 3 | Horizontal only, no altitude |
| 3D fix | 4+ | Full 3D position |
| DGPS | 4+ | Differential correction |
| RTK | 5+ | Centimeter-level accuracy |

Always check for at least a 3D fix (`fixQual >= 4`) before trusting position data.

---

## Accuracy Considerations

GPS accuracy depends on several factors:

1. **Satellite count**: More satellites = better accuracy
   - 4 sats: ~10-20m
   - 8+ sats: ~2-5m

2. **Satellite geometry**: Spread out satellites are better than clustered

3. **Environment**:
   - Clear sky: Best accuracy
   - Trees/buildings: Multipath errors
   - Indoors: Usually no fix

4. **Module quality**: Professional modules (M10) outperform hobby modules

5. **Speed**: High-speed flight can reduce accuracy

For best results, use GPS for slow horizontal motion and barometer for vertical motion.

---

## Best Practices

1. **Wait for fix before use**:
   ```cpp
   if (!gps.getHasFix()) {
       LOGW("No GPS fix yet");
       return;
   }
   ```

2. **Check satellite count**:
   ```cpp
   if (gps.getFixQual() < 6) {
       LOGW("Poor GPS quality: %d satellites", gps.getFixQual());
   }
   ```

3. **Use GPS time for logging**: GPS time is more accurate than RTC

4. **Combine with barometer**: Use baro for altitude, GPS for horizontal position

5. **Test antenna placement**: Metal and carbon fiber block GPS signals

6. **Update at reasonable rates**: 10Hz is usually sufficient, 1Hz works for slow vehicles

---

## Common Issues

### "GPS never gets a fix"

**Causes:**
- Antenna obscured by metal/carbon fiber
- Indoors or poor sky visibility
- Insufficient warm-up time

**Solutions:**
- Move antenna away from conductive materials
- Test outdoors with clear sky view
- Wait 30-60 seconds for cold start

### "Position jumps around"

**Causes:**
- Poor satellite geometry
- Multipath (signal reflections)
- Low satellite count

**Solutions:**
- Wait for more satellites (8+)
- Use filtering in State estimation
- Check antenna placement

### "Altitude doesn't match barometer"

**Causes:**
- GPS altitude is above ellipsoid, not sea level
- GPS vertical accuracy is worse than horizontal
- Barometer drift over time

**Solutions:**
- This is normal—use barometer for altitude
- GPS altitude is mainly useful for cross-checking
- Combine both in a Kalman filter

---

## Advanced: Time Zone Handling

The GPS class includes automatic timezone detection based on longitude:

```cpp
void findTimeZone() {
    // Automatically called during updates
    // Sets hrOffset for local time
}
```

Access local time:
```cpp
int8_t localHour = gps.getHour() + hrOffset;
```

---

## Summary

- `GPS` provides standardized satellite positioning interface
- Automatically tracks origin and displacement
- Implement only `init()` and `read()` for new modules
- Integrates with `State` for position estimation
- Provides UTC time from satellites
- Several implementations included (MAX_M10S, SAM_M8Q)

For general sensor information, see the [Sensor Interface](../sensor.md) documentation.

---
