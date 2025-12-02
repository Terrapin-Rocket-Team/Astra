# Basic Usage

!!! warning "Important!"
    In order for PIO to recognize that you are working on a PIO project, you *must* open VSCode in the root directory of that project—the directory that contains the `platformio.ini` file. Without this, PlatformIO **will not initialize** and you will be unable to build or use proper IntelliSense.

---

## Intro

Getting started with Astra requires only a few essential steps: create your sensors, pass them to a `State` object, configure an `Astra` system with `AstraConfig`, and call `init()` and `update()` in your setup and loop. Astra handles sensor reading, state estimation, and data logging automatically. You can customize behavior as needed, but let's start with the basics.

---

## Initial Integration

### Creating Sensor Objects

First, instantiate the sensors you'll be using. Astra provides implementations for common sensor types. For this example, we'll use an IMU, GPS, and barometer:

```cpp title="main.cpp"
#include <Arduino.h>
#include "Utils/Astra.h"
#include "State/State.h"
#include "Sensors/GPS/MAX_M10S.h"
#include "Sensors/IMU/BMI088andLIS3MDL.h"
#include "Sensors/Baro/DPS368.h"

using namespace astra;

MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;
```

These are the sensors used on TRT's Avionics Sensor Board v1.1. You can substitute any compatible sensor implementation—see the [Sensor Interface](ifaces/sensor.md) documentation for available options.

### Creating the Sensor Array

Next, create an array of pointers to your sensors. This array will be passed to the `State` object:

```cpp title="main.cpp" hl_lines="4"
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;
Sensor *sensors[] = {&gps, &imu, &baro};
```

### Creating a State Object

The `State` class handles sensor fusion and provides unified access to vehicle position, velocity, acceleration, and orientation. Instantiate it with your sensor array and an optional Kalman filter (use `nullptr` if you don't need filtering yet):

```cpp title="main.cpp" hl_lines="6"
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;
Sensor *sensors[] = {&gps, &imu, &baro};

State vehicleState(sensors, 3, nullptr);
```

The second parameter is the number of sensors in the array.

### Configuring the Astra System

Now create an `AstraConfig` object using the builder pattern. At minimum, you must provide the state object. You can also configure additional features like BlinkBuzz pins, update rates, and logging behavior:

```cpp title="main.cpp" hl_lines="8-11"
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;
Sensor *sensors[] = {&gps, &imu, &baro};

State vehicleState(sensors, 3, nullptr);

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withBuzzerPin(13)
                        .withBBPin(LED_BUILTIN);
```

Here we've configured a buzzer on pin 13 and the built-in LED for BlinkBuzz patterns. See the [Configuration Options](#configuration-options) section below for all available options.

### Creating the Astra Object

With your config ready, instantiate the main `Astra` system object:

```cpp title="main.cpp" hl_lines="13"
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;
Sensor *sensors[] = {&gps, &imu, &baro};

State vehicleState(sensors, 3, nullptr);

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withBuzzerPin(13)
                        .withBBPin(LED_BUILTIN);

Astra system(&config);
```

### Initializing and Updating

Finally, call `init()` in your `setup()` function and `update()` in your `loop()` function. That's it—Astra handles the rest:

```cpp title="main.cpp" hl_lines="17 22"
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;
Sensor *sensors[] = {&gps, &imu, &baro};

State vehicleState(sensors, 3, nullptr);

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withBuzzerPin(13)
                        .withBBPin(LED_BUILTIN);

Astra system(&config);

void setup()
{
    system.init();
}

void loop()
{
    system.update();
}
```

### What Just Happened?

When you call `system.init()`:

- All sensors are initialized
- The state estimation system is set up
- Logging backends are configured
- BlinkBuzz utilities are prepared

When you call `system.update()`:

- All sensors are read at the configured rate
- State variables (position, velocity, acceleration, orientation) are updated
- Data is logged to configured destinations
- BlinkBuzz patterns are processed

You now have a fully functional telemetry system.

---

## Configuration Options

The `AstraConfig` class uses a builder pattern, allowing you to chain method calls to configure only what you need. Here are the most commonly used options:

### Update Rates

Control how often sensors are read and state is updated:

/// tab | withUpdateRate()
```cpp
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUpdateRate(10); // 10Hz (100ms between updates)
```
///

/// tab | withUpdateInterval()
```cpp
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUpdateInterval(100); // 100ms between updates (10Hz)
```
///

!!! note
    10 Hz is the default. If you set it too high, the system may struggle to keep up. We recommend staying at or below 50 Hz unless you have specific performance requirements and have tested thoroughly.

### Logging Rates

Control how often telemetry data is written to storage:

/// tab | withLoggingRate()
```cpp
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withLoggingRate(20); // 20Hz logging
```
///

/// tab | withLoggingInterval()
```cpp
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withLoggingInterval(50); // Log every 50ms
```
///

Separating update rate from logging rate lets you read sensors frequently while conserving storage space.

### Sensor Bias Correction

Astra can automatically zero sensors before launch by averaging readings over time:

```cpp
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUsingSensorBiasCorrection(true) // Enable correction
                        .withSensorBiasCorrectionDataLength(2) // Average over 2 seconds
                        .withSensorBiasCorrectionDataIgnore(1); // Ignore the most recent 1 second
```

This is useful for compensating for sensor drift while sitting on the pad. The system averages data over the specified length while ignoring the most recent data (to avoid including early motion in the baseline).

!!! danger "Requires Launch Detection"
    Bias correction must be disabled once your vehicle launches. If you're extending `State`, call `sensor->useBiasCorrection(false)` when you detect launch. Without this, sensors will continuously re-zero during flight, producing garbage data.

### BlinkBuzz Configuration

Configure pins for LED and buzzer output patterns:

```cpp
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withBuzzerPin(13)         // Special BUZZER pin for default events
                        .withBBPin(LED_BUILTIN)    // Additional pin for custom patterns
                        .withBBPin(12)             // You can add up to 50 pins
                        .withBBAsync(true, 50);    // Enable async mode with queue size
```

The buzzer pin has special default behaviors. Additional pins can be controlled via the `bb` global object. See the [BlinkBuzz documentation](utils/blinkbuzz.md) for pattern usage.

### Adding Custom Data Reporters

If you have additional data sources beyond sensors and state (e.g., custom hardware or computed values), add them to logging:

```cpp
MyCustomReporter reporter1;
MyCustomReporter reporter2;
DataReporter *reporters[] = {&reporter1, &reporter2};

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withOtherDataReporters(reporters);
```

See the [DataReporter documentation](ifaces/data-reporter.md) for implementation details.

### Custom Log Sinks

By default, Astra logs to available storage and serial. You can provide custom log sinks:

```cpp
SDCardFile mySDLog;
UARTLog mySerialLog(Serial, 115200);
ILogSink *logs[] = {&mySDLog, &mySerialLog};

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withDataLogs(logs, 2);
```

---

## Using BlinkBuzz

BlinkBuzz provides synchronous and asynchronous LED/buzzer patterns. After adding pins via `AstraConfig`, use the global `bb` object:

```cpp
// Synchronous (blocks)
bb.on(LED_BUILTIN);
bb.off(LED_BUILTIN);
bb.onoff(LED_BUILTIN, 500);           // On for 500ms
bb.onoff(LED_BUILTIN, 200, 3);        // Blink 3 times, 200ms each
bb.onoff(LED_BUILTIN, 200, 3, 500);   // Blink 3 times with 500ms pauses

// Asynchronous (non-blocking)
bb.aonoff(LED_BUILTIN, 1000);         // Turn on for 1 second without blocking
bb.aonoff(LED_BUILTIN, 100, 5);       // Blink 5 times without blocking
bb.aonoff(LED_BUILTIN, 100, 0, 200);  // Blink indefinitely with 200ms pauses
```

Asynchronous patterns require that `system.update()` is called regularly. See the [BlinkBuzz documentation](utils/blinkbuzz.md) for advanced patterns including morse code.

---

## Using the Logging System

Astra provides two separate logging systems:

### Event Logging

For human-readable status messages, use the global `EventLogger` macros:

```cpp
#include "RecordData/Logging/EventLogger.h"

LOGI("System initialized successfully");
LOGW("GPS fix lost, using dead reckoning");
LOGE("Barometer I2C communication failed");
```

These log to configured sinks with timestamps and severity levels.

### Data Logging

Telemetry data logging happens automatically via the `DataLogger` system. Every `DataReporter` (including `State` and all `Sensor` subclasses) automatically registers its data columns. You don't need to manually log this data—just call `system.update()`.

To access logged data, see the [Logger documentation](utils/logger.md) for file formats and retrieval methods.

---

## Extending State for Custom Behavior

While the base `State` class works for many applications, you may want to add custom logic. Simply extend it:

```cpp title="CustomState.h"
#ifndef CUSTOM_STATE_H
#define CUSTOM_STATE_H

#include <State/State.h>

using namespace astra;

class CustomState : public State {
public:
    CustomState(Sensor **sensors, int numSensors, Filter *filter);

    void update(double currentTime = -1) override;

    int getFlightStage() const { return flightStage; }

private:
    int flightStage = 0;
    void detectLaunch();
    void detectLanding();
};

#endif
```

```cpp title="CustomState.cpp"
#include "CustomState.h"
#include "RecordData/Logging/EventLogger.h"

CustomState::CustomState(Sensor **sensors, int numSensors, Filter *filter)
    : State(sensors, numSensors, filter) {}

void CustomState::update(double currentTime) {
    // Call parent update to handle sensors and state estimation
    State::update(currentTime);

    // Add your custom logic
    detectLaunch();
    detectLanding();
}

void CustomState::detectLaunch() {
    if (flightStage == 0 && acceleration.z() > 20) {
        flightStage = 1;
        LOGI("Launch detected!");

        // Disable sensor bias correction after launch
        for (int i = 0; i < numSensors; i++) {
            sensors[i]->useBiasCorrection(false);
        }
    }
}

void CustomState::detectLanding() {
    if (flightStage == 1 && velocity.z() < 0.1 && position.z() < 5) {
        flightStage = 2;
        LOGI("Landing detected!");
    }
}
```

Then use your custom state instead of the base class:

```cpp title="main.cpp"
#include "CustomState.h"

// ... sensors setup ...

CustomState vehicleState(sensors, 3, nullptr);

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUsingSensorBiasCorrection(true); // Disabled by CustomState at launch

Astra system(&config);
```

---

## Accessing State Data

The `State` object provides getters for all estimated quantities:

```cpp
Vector<3> pos = vehicleState.getPosition();        // meters from origin
Vector<3> vel = vehicleState.getVelocity();        // m/s
Vector<3> accel = vehicleState.getAcceleration();  // m/s²
Quaternion orient = vehicleState.getOrientation(); // vehicle attitude
Vector<2> coords = vehicleState.getCoordinates();  // lat, lon in degrees
double heading = vehicleState.getHeading();        // degrees
```

You can access this data anywhere in your code after `system.update()` has been called.

---

## Next Steps

This covers the essentials of integrating Astra into your project. For more advanced usage:

- **[State Interface](ifaces/state.md)** - Details on state estimation and customization
- **[Sensor Interface](ifaces/sensor.md)** - Available sensors and implementing custom ones
- **[DataReporter Interface](ifaces/data-reporter.md)** - Adding custom telemetry data
- **[Filter Interface](ifaces/filters.md)** - Implementing Kalman filters for state estimation
- **[Logger Documentation](utils/logger.md)** - Log file formats and data retrieval
- **[BlinkBuzz Documentation](utils/blinkbuzz.md)** - Advanced LED and buzzer patterns

---
