# Light Sensor

The `LightSensor` class provides an interface for ambient light sensors in Astra. It extends the [`Sensor`](../sensor.md) interface to measure light intensity, useful for detecting deployment events, day/night transitions, or environmental monitoring.

---

## Overview

Light sensors measure ambient light intensity and convert it to a lux value. The `LightSensor` base class:

1. **Measures light intensity** - in lux (lumens per square meter)
2. **Tracks initial conditions** - reference light level at startup
3. **Integrates with telemetry** - automatic logging

Implementing a new light sensor requires defining `init()` and `read()` methods.

---

## Available Methods

### getLux()

```cpp
virtual const double getLux() const;
```

Returns the current light intensity in lux.

**Example:**
```cpp
LightSensor lightSensor;
double lux = lightSensor.getLux();

if (lux < 10.0) {
    LOGI("Dark environment");
} else if (lux < 1000.0) {
    LOGI("Indoor lighting");
} else {
    LOGI("Bright sunlight");
}
```

**Reference lux values:**
- 0.0001 lux: Moonless night
- 0.1 lux: Full moon
- 10 lux: Dim indoor lighting
- 100 lux: Dark overcast day
- 400 lux: Office lighting
- 1000 lux: Overcast day
- 10000 lux: Full daylight
- 100000 lux: Direct sunlight

---

## Logged Data Columns

The `LightSensor` class automatically registers:

| Column | Format | Units | Description |
|--------|--------|-------|-------------|
| `lux` | `%.2f` | lux | Light intensity |

---

## Implementing a Custom Light Sensor

### Basic Structure

```cpp
#ifndef MY_LIGHT_SENSOR_H
#define MY_LIGHT_SENSOR_H

#include "Sensors/LightSensor/LightSensor.h"
#include <SomeLightSensorLibrary.h>

namespace astra {

class MyLightSensor : public LightSensor {
public:
    MyLightSensor(const char *name = "Light");

protected:
    bool init() override;
    bool read() override;

private:
    SomeLightSensorDriver hardware;
};

}

#endif
```

### Implementation Example

```cpp
#include "MyLightSensor.h"

using namespace astra;

MyLightSensor::MyLightSensor(const char *name) : LightSensor(name) {}

bool MyLightSensor::init() {
    if (!hardware.begin()) {
        LOGE("MyLightSensor: Failed to initialize");
        return false;
    }

    // Configure sensor
    hardware.setGain(GAIN_AUTO);
    hardware.setIntegrationTime(100);  // 100ms

    // Store initial light level
    initialLux = hardware.readLux();

    LOGI("MyLightSensor: Initialized (initial: %.2f lux)", initialLux);
    return true;
}

bool MyLightSensor::read() {
    if (!hardware.dataReady()) {
        return false;
    }

    // Update lux value
    lux = hardware.readLux();
    return true;
}
```

---

## Usage Example

### Detecting Nose Cone Separation

```cpp
#include "Sensors/LightSensor/LightSensor.h"

MyLightSensor lightSensor;
bool noseConeDeployed = false;

void setup() {
    lightSensor.begin();
}

void loop() {
    lightSensor.update();

    double currentLux = lightSensor.getLux();

    // Detect sudden increase in light (nose cone ejected)
    if (!noseConeDeployed && currentLux > initialLux * 10) {
        LOGI("Nose cone deployed! Lux jumped from %.2f to %.2f",
             initialLux, currentLux);
        noseConeDeployed = true;
    }
}
```

### Day/Night Detection

```cpp
void checkTimeOfDay() {
    double lux = lightSensor.getLux();

    if (lux < 1.0) {
        // Night
        setLowPowerMode(true);
    } else {
        // Day
        setLowPowerMode(false);
    }
}
```

---

## Integration with State

Light sensors can be added to the sensor array:

```cpp
MyLightSensor lightSensor("Light");
DPS368 baro;
BMI088andLIS3MDL imu;

Sensor *sensors[] = {&baro, &imu, &lightSensor};
State vehicleState(sensors, 3, nullptr);
```

---

## Common Applications

1. **Deployment Detection**: Detect fairing/nose cone separation by light increase
2. **Day/Night Sensing**: Adjust power modes or operation based on lighting
3. **Environmental Monitoring**: Log ambient light conditions
4. **Orientation Detection**: Some configurations can infer sun angle

---

## Best Practices

1. **Position carefully**: Mount where expected light changes occur
2. **Consider saturation**: Very bright light can saturate sensors
3. **Account for shadowing**: Vehicle orientation affects readings
4. **Use thresholds wisely**: Set deployment thresholds well above noise

---

## Summary

- `LightSensor` measures ambient light intensity in lux
- Implement `init()` and `read()` for hardware-specific sensors
- Useful for deployment detection and environmental monitoring
- Simple interface with single measurement value

For general sensor information, see the [Sensor Interface](../sensor.md) documentation.

---
