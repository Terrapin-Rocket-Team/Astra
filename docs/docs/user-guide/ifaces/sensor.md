# Sensor Interface

The `Sensor` base class provides a unified interface for all hardware sensors in Astra. Whether you're working with barometers, GPS units, IMUs, encoders, or custom hardware, they all share the same initialization and update patterns. By inheriting from `DataReporter`, sensors automatically integrate with the telemetry logging system.

---

## Overview

The `Sensor` interface ensures that all sensor types:

1. **Initialize uniformly** via `begin()`
2. **Update consistently** via `update()`
3. **Identify themselves** with type hashing for runtime queries
4. **Log automatically** through inherited `DataReporter`
5. **Integrate seamlessly** with `State` for sensor fusion

All built-in sensors (Barometer, IMU, GPS, Encoder, LightSensor) inherit from `Sensor` or its specialized subclasses.

---

## Core Methods

### begin()

```cpp
virtual bool begin();
```

Initializes the sensor hardware. This typically involves:
- Setting up I2C/SPI communication
- Configuring sensor registers
- Performing initial calibration
- Registering data columns for logging

Returns `true` if initialization succeeds, `false` otherwise. Failed sensors are automatically disabled by `State`.

**Called by:** `State::begin()` during system initialization

### update()

```cpp
virtual bool update();
```

Reads fresh data from the sensor. By default, this calls the protected `read()` method. Some sensor types override this to perform additional processing (e.g., barometers calculate altitude from pressure).

Returns `true` if the read succeeds, `false` on communication failure.

**Called by:** `State::update()` at the configured update rate

### getType() and getTypeString()

```cpp
virtual const SensorType getType() const;
virtual const char *getTypeString() const;
```

Returns the sensor's type identifier. `SensorType` is a `uint32_t` hash of the type string, computed at compile time using FNV-1a hashing.

**Example:**
```cpp
IMU imu;
SensorType type = imu.getType();           // Hash value
const char *name = imu.getTypeString();    // "IMU"
```

This type system allows `State` to query sensors by type:

```cpp
Sensor *gpsPtr = vehicleState.getSensor(gps.getType());
```

### isInitialized()

```cpp
virtual bool isInitialized() const;
```

Returns `true` if `begin()` succeeded, `false` otherwise.

You can also use the bool conversion operator:

```cpp
if (myBarometer) {
    // Sensor is initialized
}
```

---

## Sensor Type System

Astra uses compile-time string hashing to identify sensor types. Each sensor declares its type in the constructor:

```cpp
Barometer::Barometer(const char *name)
    : Sensor("Barometer", name) {
    // ...
}
```

The `Sensor` constructor hashes `"Barometer"` to create a unique 32-bit identifier. This allows type-safe runtime queries without RTTI overhead.

**Available sensor type strings:**
- `"Barometer"`
- `"GPS"`
- `"IMU"`
- `"Encoder"`
- `"LightSensor"`
- Any custom string for your own sensors

---

## DataReporter Integration

Since `Sensor` inherits from `DataReporter`, all sensors automatically register their data for CSV logging.

**Example from Barometer:**
```cpp
Barometer::Barometer(const char *name) : Sensor("Barometer", name) {
    addColumn("%.2f", &pressure, "pressure_pa");
    addColumn("%.1f", &temp, "temp_c");
    addColumn("%.2f", &altitudeASL, "alt_asl_m");
}
```

When you pass sensors to `State`, and `State` to `AstraConfig`, all sensor data appears automatically in telemetry:

```csv
...,Barometer - pressure_pa,Barometer - temp_c,Barometer - alt_asl_m
...,101325.50,22.3,0.00
```

See the [DataReporter documentation](data-reporter.md) for more details.

---

## Implementing a Custom Sensor

To create your own sensor, inherit from `Sensor` and implement the protected `init()` and `read()` methods:

### Basic Structure

```cpp
#ifndef MY_SENSOR_H
#define MY_SENSOR_H

#include "Sensors/Sensor.h"

namespace astra {

class MySensor : public Sensor {
public:
    MySensor(const char *name = "MySensor");

    // Getter methods for your data
    float getDistance() const { return distance; }
    bool isDetected() const { return detected; }

protected:
    bool init() override;
    bool read() override;

private:
    float distance = 0.0f;
    bool detected = false;

    // Hardware-specific members
    int i2cAddress = 0x42;
};

}

#endif
```

### Implementation Example

```cpp
#include "MySensor.h"
#include <Wire.h>

using namespace astra;

MySensor::MySensor(const char *name) : Sensor("MySensor", name) {
    // Register columns for telemetry
    addColumn("%.2f", &distance, "distance_cm");
    addColumn("%d", &detected, "detected");
}

bool MySensor::init() {
    // Initialize I2C communication
    Wire.begin();

    // Check if sensor responds
    Wire.beginTransmission(i2cAddress);
    if (Wire.endTransmission() != 0) {
        LOGE("MySensor: Failed to detect sensor at 0x%02X", i2cAddress);
        return false;
    }

    // Configure sensor registers
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x00); // Config register
    Wire.write(0x01); // Config value
    Wire.endTransmission();

    LOGI("MySensor: Initialized successfully");
    return true;
}

bool MySensor::read() {
    // Read data from sensor
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x10); // Data register
    if (Wire.endTransmission() != 0) {
        return false;
    }

    Wire.requestFrom(i2cAddress, 2);
    if (Wire.available() >= 2) {
        uint8_t high = Wire.read();
        uint8_t low = Wire.read();
        uint16_t raw = (high << 8) | low;

        distance = raw * 0.1f; // Convert to cm
        detected = (distance < 400.0f);
        return true;
    }

    return false;
}
```

### Using Your Custom Sensor

```cpp
#include "MySensor.h"

MySensor distanceSensor("Distance");
MAX_M10S gps;
DPS368 baro;

Sensor *sensors[] = {&gps, &baro, &distanceSensor};
State vehicleState(sensors, 3, nullptr);

void loop() {
    system.update();

    // Access sensor data
    if (distanceSensor.isDetected()) {
        float dist = distanceSensor.getDistance();
        LOGI("Distance: %.2f cm", dist);
    }
}
```

---

## Specialized Sensor Interfaces

Astra provides specialized base classes for common sensor types. These extend `Sensor` with type-specific methods:

### Barometer

Provides pressure and altitude data:

```cpp
class Barometer : public Sensor {
    virtual double getPressure() const;
    virtual double getTemp() const;
    virtual double getASLAltM() const;
    // ...
};
```

See [Barometer documentation](sensors/baro.md) for details.

### IMU

Provides 9DOF inertial data (accelerometer, gyroscope, magnetometer):

```cpp
class IMU : public Sensor {
    virtual Vector<3> getAccelerationGlobal() const;
    virtual Vector<3> getGyroscopeGlobal() const;
    virtual Vector<3> getMagnetometerGlobal() const;
    virtual Quaternion getOrientation() const;
    // ...
};
```

See [IMU documentation](sensors/imu.md) for details.

### GPS

Provides position and velocity data:

```cpp
class GPS : public Sensor {
    virtual Vector<2> getCoordinates() const;
    virtual double getAltitude() const;
    virtual Vector<3> getVelocity() const;
    virtual int getNumSatellites() const;
    // ...
};
```

See [GPS documentation](sensors/gps.md) for details.

### Encoder

Provides rotational position and velocity:

```cpp
class AstraEncoder : public Sensor {
    virtual double getPosition() const;
    virtual double getVelocity() const;
    // ...
};
```

See [Encoder documentation](sensors/enc.md) for details.

### LightSensor

Provides ambient light measurements:

```cpp
class LightSensor : public Sensor {
    virtual float getLux() const;
    // ...
};
```

See [LightSensor documentation](sensors/light.md) for details.

---

## Available Sensor Implementations

Astra includes implementations for common sensors:

**Barometers:**
- BMP280
- BMP390
- DPS368
- MS5611F
- MockBarometer (for testing)

**IMUs:**
- BMI088 (6DOF - Accel + Gyro)
- BMI088andLIS3MDL (9DOF)
- BNO055 (9DOF with hardware fusion)
- MockIMU (for testing)

**GPS:**
- MAX_M10S
- SAM_M8Q
- MockGPS (for testing)

**Accelerometers (standalone):**
- ADXL375 (high-g)
- H3LIS331DL (high-g)

**Magnetometers (standalone):**
- LIS3MDL
- MMC5603NJ
- MMC5633

**Encoders:**
- AstraEncoder

**Light Sensors:**
- LightSensor (generic)

---

## Sensor Naming

Each sensor can have a unique name for identification in logs:

```cpp
DPS368 baro1("Baro1");
DPS368 baro2("Baro2");

Sensor *sensors[] = {&baro1, &baro2};
```

**CSV output:**
```csv
...,Baro1 - pressure_pa,Baro1 - temp_c,Baro2 - pressure_pa,Baro2 - temp_c
```

If no name is provided, a unique name is auto-generated (`Barometer0`, `Barometer1`, etc.).

---

## Sensor Initialization and Error Handling

When `State::begin()` is called, it attempts to initialize all sensors. If a sensor fails:

1. It's marked as disabled
2. An error is logged via `LOGE()`
3. `State` continues with remaining sensors
4. The failed sensor is excluded from updates

**Example log output:**
```
0.123 [ERROR]: Barometer: Failed to detect sensor
0.124 [INFO]: State: Initialized with 2 of 3 sensors
```

Always check `isInitialized()` before relying on sensor data:

```cpp
if (barometer.isInitialized()) {
    double pressure = barometer.getPressure();
}
```

Or use `State::sensorOK()`:

```cpp
Sensor *baro = state.getSensor(barometer.getType());
if (state.sensorOK(baro)) {
    // Safe to use
}
```

---

## Protected Members

When implementing a sensor, you have access to these protected members:

| Member | Type | Description |
|--------|------|-------------|
| `initialized` | `bool` | Set by `begin()`, indicates successful init |
| `type` | `SensorType` | Hash of the type string |
| `typeString` | `const char*` | Human-readable type name |

You can also access all `DataReporter` protected members for managing logged columns.

---

## Virtual Method Override Pattern

The `Sensor` class uses a template method pattern:

**Public methods (don't override):**
- `begin()` - Calls `init()` and sets `initialized`
- `update()` - Calls `read()` and returns result

**Protected methods (override these):**
- `init()` - Hardware initialization
- `read()` - Data acquisition

This pattern ensures consistent error handling and state management while giving you control over hardware-specific details.

---

## Best Practices

1. **Log initialization clearly**: Use `LOGI()` for success, `LOGE()` for failure. Include relevant details like I2C addresses.

2. **Handle communication failures gracefully**: Return `false` from `read()` on failure. Don't crash or hang.

3. **Register columns in constructor**: Make all `addColumn()` calls during construction, not in `init()`.

4. **Initialize member variables**: Always give sensible defaults (0, NaN, etc.) to prevent garbage data if sensor fails.

5. **Use appropriate precision**: GPS needs `%.7f`, temperatures might only need `%.1f`.

6. **Provide getter methods**: Don't expose raw member variables. Use getter methods for type safety and future flexibility.

7. **Document units**: Make it clear what units your methods return (meters, feet, Pascals, etc.).

8. **Test with mock sensors**: Use or create mock sensors for testing without hardware.

---

## Example: Adding Bias Correction Support

If your sensor suffers from drift, you can add bias correction:

```cpp
class DriftingSensor : public Sensor {
public:
    DriftingSensor() : Sensor("DriftingSensor", "Drift") {
        addColumn("%.2f", &value, "value");
        addColumn("%.2f", &bias, "bias");
    }

    void useBiasCorrection(bool enable) {
        correctingBias = enable;
        if (enable) {
            biasHistory.clear();
        }
    }

protected:
    bool init() override {
        // Initialize hardware
        return true;
    }

    bool read() override {
        float rawValue = readFromHardware();

        if (correctingBias && biasHistory.size() < 100) {
            biasHistory.push(rawValue);
            bias = calculateAverage(biasHistory);
        }

        value = rawValue - bias;
        return true;
    }

private:
    float value = 0.0f;
    float bias = 0.0f;
    bool correctingBias = false;
    CircBuffer<float> biasHistory{100};

    float readFromHardware() { return 10.5f; }
    float calculateAverage(const CircBuffer<float> &buf) {
        // Calculate average
        return 0.0f;
    }
};
```

Then disable bias correction at launch in your custom `State::update()`:

```cpp
void CustomState::update(double currentTime) {
    State::update(currentTime);

    if (stage == 0 && acceleration.z() > 20) {
        stage = 1;
        // Disable bias correction at launch
        for (int i = 0; i < numSensors; i++) {
            // Cast to your sensor type if it has useBiasCorrection
            DriftingSensor *ds = dynamic_cast<DriftingSensor*>(sensors[i]);
            if (ds) {
                ds->useBiasCorrection(false);
            }
        }
    }
}
```

---

## Summary

- `Sensor` is the base class for all hardware sensors
- Implements initialization, updates, and type identification
- Automatically integrates with telemetry via `DataReporter`
- Use specialized subclasses (Barometer, IMU, GPS) when available
- Override `init()` and `read()` to implement custom sensors
- All sensors are managed by `State` for unified updates and logging

For specific sensor types, see:
- [Barometer](sensors/baro.md)
- [IMU](sensors/imu.md)
- [GPS](sensors/gps.md)
- [Encoder](sensors/enc.md)
- [LightSensor](sensors/light.md)

---
