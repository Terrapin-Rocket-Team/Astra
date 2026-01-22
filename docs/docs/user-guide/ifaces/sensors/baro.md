# Barometer

The `Barometer` class provides a standardized interface for barometric pressure sensors in Astra. It extends the [`Sensor`](../sensor.md) interface with pressure-specific functionality and automatically handles pressure-to-altitude conversion using standard atmospheric models.

---

## Overview

Barometers measure atmospheric pressure and convert it to altitude. The `Barometer` base class handles:

1. **Pressure to altitude conversion** using the hypsometric formula
2. **Automatic ground-level calibration** at initialization
3. **Temperature compensation** for accurate readings
4. **Telemetry registration** for pressure, temperature, and altitude

Implementing a new barometer requires only defining `init()` and `read()` methods—altitude calculation is automatic.

---

## Available Methods

### Pressure Readings

```cpp
virtual double getPressure() const;        // Pressure in Pascals
virtual double getPressureAtm() const;     // Pressure in atmospheres
```

Returns the most recent pressure reading.

**Example:**
```cpp
DPS368 baro;
double pressure = baro.getPressure();      // 101325.0 Pa (sea level)
double atm = baro.getPressureAtm();        // 1.0 atm
```

### Temperature Readings

```cpp
virtual double getTemp() const;            // Temperature in °C
virtual double getTempF() const;           // Temperature in °F
```

Returns temperature from the barometer's internal sensor. Useful for temperature compensation and environmental monitoring.

**Example:**
```cpp
double tempC = baro.getTemp();             // 22.5°C
double tempF = baro.getTempF();            // 72.5°F
```

### Altitude Readings

```cpp
virtual double getASLAltM() const;         // Altitude above sea level in meters
virtual double getASLAltFt() const;        // Altitude above sea level in feet
```

Returns altitude calculated from current pressure using the standard atmosphere model.

**Example:**
```cpp
double altM = baro.getASLAltM();           // 150.2 meters ASL
double altFt = baro.getASLAltFt();         // 492.8 feet ASL
```

---

## Altitude Calculation

The barometer uses the hypsometric formula to convert pressure to altitude:

```cpp
altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
```

Where:
- `seaLevelPressure` = 101325 Pa (standard atmosphere at sea level)
- Result is in meters

This formula assumes standard atmospheric conditions. For maximum accuracy in non-standard conditions, you'd need to adjust the sea-level pressure reference or use GPS altitude as ground truth.

---

## Logged Data Columns

The `Barometer` class automatically registers these columns for telemetry:

| Column | Format | Units | Description |
|--------|--------|-------|-------------|
| `pressure_pa` | `%.2f` | Pascals | Atmospheric pressure |
| `temp_c` | `%.1f` | °C | Temperature |
| `alt_asl_m` | `%.2f` | meters | Altitude above sea level |

**CSV example:**
```csv
...,Barometer - pressure_pa,Barometer - temp_c,Barometer - alt_asl_m
...,101325.50,22.3,150.20
...,101320.00,22.4,150.65
```

---

## Implementing a Custom Barometer

To add support for a new barometer sensor, inherit from `Barometer` and implement `init()` and `read()`:

### Basic Structure

```cpp
#ifndef MY_BAROMETER_H
#define MY_BAROMETER_H

#include "Sensors/Baro/Barometer.h"
#include <SomeBarometerLibrary.h>

namespace astra {

class MyBarometer : public Barometer {
public:
    MyBarometer(const char *name = "MyBaro");

protected:
    bool init() override;
    bool read() override;

private:
    SomeBarometerDriver hardware;
};

}

#endif
```

### Implementation Example

```cpp
#include "MyBarometer.h"

using namespace astra;

MyBarometer::MyBarometer(const char *name) : Barometer(name) {
    // Constructor - barometer columns are registered by parent
}

bool MyBarometer::init() {
    // Initialize I2C communication
    if (!hardware.begin()) {
        LOGE("MyBarometer: Failed to initialize");
        return false;
    }

    // Configure sensor settings
    hardware.setOversampleRate(16);  // High precision
    hardware.setMode(CONTINUOUS);

    LOGI("MyBarometer: Initialized successfully");
    return true;
}

bool MyBarometer::read() {
    // Read from hardware
    if (!hardware.measurementReady()) {
        return false;
    }

    // Update protected member variables
    pressure = hardware.readPressure();  // Must be in Pascals
    temp = hardware.readTemperature();   // Must be in Celsius

    // Parent's update() will automatically calculate altitude
    return true;
}
```

### Important Notes

1. **Units matter**: `pressure` must be in Pascals, `temp` in Celsius
2. **Update protected members**: Set `pressure` and `temp` in `read()`, altitude is calculated automatically
3. **Return false on failure**: If communication fails, return `false` to signal an error
4. **Don't calculate altitude**: The parent class handles this automatically

---

## Available Implementations

Astra includes drivers for several common barometers:

### DPS368 / DPS310

Infineon high-precision barometer. DPS310 is NRND (not recommended for new designs), use DPS368.

```cpp
#include "Sensors/HW/Baro/DPS368.h"

DPS368 baro;                           // Default: address 0x77, Wire
DPS368 baro2("Baro2", 0x76, &Wire1);  // Custom address and I2C bus
```

**Specifications:**
- Range: 300-1200 hPa
- Accuracy: ±0.002 hPa (±0.02m)
- Interface: I2C (0x76 or 0x77)

### BMP390

Bosch high-accuracy barometer, recommended for new designs.

```cpp
#include "Sensors/HW/Baro/BMP390.h"

BMP390 baro;
```

**Specifications:**
- Range: 300-1250 hPa
- Accuracy: ±0.5 Pa (±0.05m)
- Interface: I2C or SPI

### BMP280

Bosch barometer. NRND, but still commonly available.

```cpp
#include "Sensors/Baro/BMP280.h"

BMP280 baro;
```

**Specifications:**
- Range: 300-1100 hPa
- Accuracy: ±1 Pa (±0.1m)
- Interface: I2C or SPI

### MS5611F

TE Connectivity high-resolution barometer.

```cpp
#include "Sensors/HW/Baro/MS5611.h"

MS5611F baro;
```

**Specifications:**
- Range: 10-1200 hPa
- Accuracy: ±1.5 hPa (±15m)
- Interface: I2C or SPI

---

## Usage Example

### Basic Usage

```cpp
#include "Sensors/HW/Baro/DPS368.h"

DPS368 baro;

void setup() {
    if (!baro.begin()) {
        Serial.println("Barometer init failed!");
    }
}

void loop() {
    baro.update();

    Serial.print("Pressure: ");
    Serial.print(baro.getPressure());
    Serial.println(" Pa");

    Serial.print("Altitude: ");
    Serial.print(baro.getASLAltM());
    Serial.println(" m");

    delay(100);
}
```

### Multiple Barometers

You can use multiple barometers for redundancy:

```cpp
DPS368 baro1("Baro1", 0x77);
DPS368 baro2("Baro2", 0x76);

Sensor *sensors[] = {&baro1, &baro2};
State vehicleState(sensors, 2, nullptr);

void loop() {
    system.update();

    // Access individual barometers
    double alt1 = baro1.getASLAltM();
    double alt2 = baro2.getASLAltM();

    // Check for sensor agreement
    if (abs(alt1 - alt2) > 5.0) {
        LOGW("Barometer disagreement: %.1f vs %.1f", alt1, alt2);
    }
}
```

**CSV output:**
```csv
...,Baro1 - pressure_pa,Baro1 - alt_asl_m,Baro2 - pressure_pa,Baro2 - alt_asl_m
...,101325.0,150.2,101323.5,150.3
```

---

## Integration with State

When you pass barometers to `State`, their altitude data contributes to state estimation:

```cpp
DPS368 baro;
MAX_M10S gps;
BMI088andLIS3MDL imu;

Sensor *sensors[] = {&gps, &imu, &baro};
State vehicleState(sensors, 3, nullptr);
```

The `State` class uses barometer altitude to:
- Provide vertical position estimates
- Detect launch (altitude change)
- Detect apogee (maximum altitude)
- Detect landing (return to ground level)

---

## Coordinate System

Barometer altitude is always **vertical** (Z-axis in Astra's coordinate system):

- **Positive Z**: Upward
- **Zero altitude**: Set at initialization (ground level)
- **Units**: Meters (use `getASLAltM()`) or feet (use `getASLAltFt()`)

When used with GPS and IMU in `State`, the barometer provides the Z component while GPS provides horizontal position.

---

## Calibration and Accuracy

### Ground-Level Calibration

The barometer automatically calibrates to ground level during `begin()`. The first pressure reading becomes the reference point for altitude calculations.

**Important:** Ensure the vehicle is at rest at the desired reference altitude when calling `begin()`.

### Accuracy Considerations

Barometric altitude accuracy depends on:

1. **Sensor precision**: DPS368 (~2cm) vs BMP280 (~10cm)
2. **Temperature changes**: Compensated internally by most sensors
3. **Weather changes**: Pressure varies with weather (~10 hPa = ~80m error)
4. **Altitude range**: More accurate at lower altitudes

For flights longer than a few minutes or in changing weather, combine barometer data with GPS altitude for best results.

---

## Best Practices

1. **Initialize at rest**: Call `begin()` when the vehicle is stationary at launch site

2. **Check initialization**: Always verify the sensor initialized successfully
   ```cpp
   if (!baro.begin()) {
       LOGE("Barometer failed to initialize");
   }
   ```

3. **Use multiple barometers**: Redundancy helps detect sensor failures

4. **Monitor temperature**: Large temperature swings can affect accuracy
   ```cpp
   if (baro.getTemp() < -20 || baro.getTemp() > 80) {
       LOGW("Barometer temperature out of normal range");
   }
   ```

5. **Filter rapid changes**: At high speeds, use a Kalman filter to smooth altitude estimates

6. **Compare with GPS**: Use GPS altitude to validate barometer readings

---

## Common Issues

### "Altitude jumping around"

**Cause:** Vibration, rapid temperature changes, or electrical noise

**Solution:**
- Add physical dampening to the sensor
- Use lower update rates
- Implement a Kalman filter in `State`

### "Altitude drifting over time"

**Cause:** Weather-related pressure changes

**Solution:**
- For short flights (<10 min), drift is usually acceptable
- For longer missions, use GPS altitude as reference
- Consider implementing pressure trend compensation

### "Negative altitude readings"

**Cause:** Pressure increased after calibration (weather changed or moved to lower elevation)

**Solution:** Normal behavior. Altitude is relative to initialization point.

---

## Advanced: Custom Altitude Calculation

If you need a different altitude calculation (e.g., for non-Earth atmospheres or custom pressure models), override the `update()` method:

```cpp
class CustomBarometer : public Barometer {
protected:
    bool update() override {
        // Read sensor
        if (!read()) {
            return false;
        }

        // Custom altitude calculation
        altitudeASL = myCustomAltitudeFormula(pressure);

        return true;
    }

private:
    double myCustomAltitudeFormula(double pressure) {
        // Your custom formula here
        return 0.0;
    }
};
```

---

## Summary

- `Barometer` provides a standardized interface for pressure sensors
- Automatically calculates altitude from pressure
- Implement only `init()` and `read()` for new sensors
- Supports multiple simultaneous barometers
- Integrates seamlessly with `State` for altitude tracking
- Several high-quality implementations included (DPS368, BMP390, MS5611F)

For general sensor information, see the [Sensor Interface](../sensor.md) documentation.

---
