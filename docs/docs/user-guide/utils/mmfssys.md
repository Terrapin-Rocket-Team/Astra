# Astra

The `Astra` class (formerly `AstraSystem`) is the main entry point for using the Astra flight software library. It coordinates sensors, state estimation, logging, and utilities through a single interface. You configure Astra using the builder-pattern `AstraConfig` class, then call `init()` and `update()` to handle everything automatically.

---

## Overview

The Astra system integrates:

1. **State estimation** - Reads sensors, updates vehicle state
2. **Data logging** - Writes telemetry to storage (SD, flash, etc.)
3. **Event logging** - Records human-readable messages
4. **BlinkBuzz** - Non-blocking LED/buzzer patterns
5. **Timing control** - Manages update rates for sensors and logging

You create an `AstraConfig` object with your desired settings, pass it to `Astra`, and let the system handle the rest.

---

## Basic Usage

### Minimal Setup

```cpp
#include <Astra.h>

// Create sensors
MAX_M10S gps;
DPS368 baro;
BMI088andLIS3MDL imu;

Sensor *sensors[] = {&gps, &baro, &imu};
State vehicleState(sensors, 3, nullptr);

// Configure Astra
AstraConfig config = AstraConfig()
                        .withState(&vehicleState);

Astra system(&config);

void setup() {
    Serial.begin(115200);
    system.init();
}

void loop() {
    system.update();
}
```

### Complete Example

```cpp
#include <Astra.h>
#include <RecordData/Logging/LoggingBackend/USBLog.h>
#include <RecordData/Logging/LoggingBackend/FileLogSink.h>

// Sensors
MAX_M10S gps;
DPS368 baro;
BMI088andLIS3MDL imu;

Sensor *sensors[] = {&gps, &baro, &imu};

// State with custom subclass
class RocketState : public State {
public:
    RocketState(Sensor **sensors, int numSensors, Filter *filter)
        : State(sensors, numSensors, filter) {}

    void update(double currentTime) override {
        State::update(currentTime);

        // Custom stage detection logic
        if (acceleration.z() > 20) {
            LOGI("Launch detected!");
        }
    }
};

RocketState vehicleState(sensors, 3, nullptr);

// Logging backends
FileLogSink fileLog;
USBLog usbLog;
ILogSink *logs[] = {&fileLog, &usbLog};

// Configuration
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUpdateRate(50)           // 50 Hz sensor updates
                        .withLoggingRate(20)          // 20 Hz data logging
                        .withBuzzerPin(13)            // Buzzer on pin 13
                        .withBBPin(LED_BUILTIN)       // LED indicator
                        .withBBAsync(true, 100)       // Async patterns
                        .withDataLogs(logs, 2);       // File + USB logging

Astra system(&config);

void setup() {
    Serial.begin(115200);
    system.init();

    LOGI("System initialized");
}

void loop() {
    system.update();

    // Your application code here
}
```

---

## AstraConfig

The `AstraConfig` class uses a builder pattern for configuration. Each method returns a reference to the config object, allowing chaining.

### Required Configuration

**`withState(State *state)`**

Pass a `State` object containing your sensors. This is **required** for Astra to function.

```cpp
State vehicleState(sensors, numSensors, filter);

AstraConfig config = AstraConfig()
                        .withState(&vehicleState);
```

The state's sensors are automatically registered for updates and logging.

---

### Update Timing

Control how often sensors are read and state is updated.

**`withUpdateRate(double rate)`**

Set update rate in Hertz. Mutually exclusive with `withUpdateInterval()`.

```cpp
config.withUpdateRate(50);  // 50 Hz (20ms intervals)
```

**`withUpdateInterval(unsigned int interval)`**

Set update interval in milliseconds. Mutually exclusive with `withUpdateRate()`.

```cpp
config.withUpdateInterval(20);  // 20ms (50 Hz)
```

**Default:** 10 Hz (100ms)

**Recommendations:**
- **10-20 Hz**: Basic telemetry, low-speed vehicles
- **50-100 Hz**: High-speed rockets, precise state estimation
- **200+ Hz**: IMU-heavy applications, high-g environments

---

### Logging Timing

Control how often data is written to storage.

**`withLoggingRate(double rate)`**

Set logging rate in Hertz. Mutually exclusive with `withLoggingInterval()`.

```cpp
config.withLoggingRate(20);  // 20 Hz
```

**`withLoggingInterval(unsigned int interval)`**

Set logging interval in milliseconds. Mutually exclusive with `withLoggingRate()`.

```cpp
config.withLoggingInterval(50);  // 50ms (20 Hz)
```

**Default:** 10 Hz (100ms)

**Note:** Logging rate can be lower than update rate to save storage space. For example, update sensors at 100 Hz but log at 20 Hz.

---

### Logging Backends

Specify where event logs and telemetry should be written.

**`withDataLogs(ILogSink **logs, uint8_t numLogs)`**

Add logging backends for event logs (LOGI/LOGW/LOGE). Multiple backends can be active simultaneously.

```cpp
FileLogSink fileLog;
USBLog usbLog;
ILogSink *logs[] = {&fileLog, &usbLog};

config.withDataLogs(logs, 2);
```

**Available backends:**
- `FileLogSink` - Write to SD card or internal flash
- `USBLog` - Write to USB serial
- `UARTLog` - Write to hardware UART
- `PrintLog` - Write to any `Print` object
- `CircBufferLog` - Buffer in RAM

See [Logger documentation](logger.md) for details.

---

### BlinkBuzz Configuration

Configure LED and buzzer patterns.

**`withBuzzerPin(unsigned int pin)`**

Set the buzzer pin. Creates a named `BUZZER` constant for use with BlinkBuzz.

```cpp
config.withBuzzerPin(13);  // Buzzer on pin 13
```

**`withBBPin(unsigned int pin)`**

Add an LED or buzzer pin to BlinkBuzz. Can be called multiple times to add multiple pins.

```cpp
config.withBBPin(LED_BUILTIN)
      .withBBPin(GPS_INDICATOR_PIN)
      .withBBPin(ERROR_LED_PIN);
```

**`withBBAsync(bool enable, unsigned int queueSize)`**

Enable asynchronous BlinkBuzz patterns with specified queue size.

```cpp
config.withBBAsync(true, 100);  // 100 events per pin
```

**Default:** Enabled with queue size 50

See [BlinkBuzz documentation](blinkbuzz.md) for usage patterns.

---

### Sensor Bias Correction

Configure automatic sensor bias correction (re-zeroing) before launch.

**`withUsingSensorBiasCorrection(bool enable)`**

Enable continuous sensor bias correction while on the ground.

```cpp
config.withUsingSensorBiasCorrection(true);
```

!!! warning "Requires Launch Detection"
    Bias correction only works before launch is detected. Without proper launch detection in your `State` class, sensors will continue re-zeroing during flight, corrupting data.

**`withSensorBiasCorrectionDataLength(unsigned int seconds)`**

Duration (in seconds) over which to average sensor data for bias estimation. Affected by update rate.

```cpp
config.withSensorBiasCorrectionDataLength(5);  // Average over 5 seconds
```

**Default:** 2 seconds

**`withSensorBiasCorrectionDataIgnore(unsigned int seconds)`**

Duration (in seconds) of recent data to ignore when calculating bias. Helps avoid transients.

```cpp
config.withSensorBiasCorrectionDataIgnore(1);  // Ignore last 1 second
```

**Default:** 1 second

**Example:**

```cpp
// Average 10 seconds of data, ignoring the most recent 2 seconds
config.withUsingSensorBiasCorrection(true)
      .withSensorBiasCorrectionDataLength(10)
      .withSensorBiasCorrectionDataIgnore(2)
      .withUpdateRate(50);  // 50 Hz

// This will average 400 samples: (10s - 2s) * 50Hz = 400 samples
```

---

### Additional Data Reporters

Add custom data sources for logging beyond sensors.

**`withOtherDataReporters(DataReporter **reporters)`**

Add additional `DataReporter` objects for telemetry logging.

```cpp
class CustomTelemetry : public DataReporter {
public:
    CustomTelemetry() : DataReporter("Custom") {
        addColumn("%.2f", &voltage, "battery_v");
        addColumn("%.1f", &temperature, "temp_c");
    }

    void update() {
        voltage = readBatteryVoltage();
        temperature = readTemperature();
    }

private:
    double voltage = 0;
    double temperature = 0;
};

CustomTelemetry customData;
DataReporter *reporters[] = {&customData};

config.withOtherDataReporters(reporters);
```

**Note:** Sensors passed via `withState()` are automatically logged—no need to add them again.

---

### Event System (Removed)

**`withNoDefaultEventListener()`**

This method is **deprecated**. The event system was removed in the transition from MMFS to Astra.

---

### Log Formatting

Customize event log message formatting.

**`withLogPrefixFormatting(const char *prefix)`**

Set the prefix format for event log messages. Use `$time` and `$logType` as placeholders.

```cpp
config.withLogPrefixFormatting("[$logType] $time: ");
```

**Default:** `"$time - [$logType] "`

**Example output:**
```
1.234 - [INFO] GPS initialized
1.456 - [WARN] Barometer calibration pending
2.789 - [ERROR] IMU communication failed
```

With custom format `"[$logType] $time: "`:
```
[INFO] 1.234: GPS initialized
[WARN] 1.456: Barometer calibration pending
[ERROR] 2.789: IMU communication failed
```

---

## Astra Methods

### init()

```cpp
void init();
```

Initialize the Astra system. This:
1. Initializes sensors via `State::begin()`
2. Initializes logging backends
3. Initializes BlinkBuzz
4. Logs initialization status

**Call in `setup()`:**

```cpp
void setup() {
    Serial.begin(115200);
    system.init();
}
```

**Log output example:**
```
0.123 - [INFO] State initialized with 3 sensors
0.124 - [INFO] GPS initialized
0.125 - [INFO] Barometer initialized
0.126 - [INFO] IMU initialized
0.127 - [INFO] Astra system ready
```

### update()

```cpp
bool update(double currentTime = -1);
```

Update the Astra system. This:
1. Reads sensors and updates state (at configured update rate)
2. Writes telemetry to storage (at configured logging rate)
3. Updates BlinkBuzz patterns

**Returns:** `true` if state was updated this call, `false` otherwise

**Call in `loop()`:**

```cpp
void loop() {
    system.update();

    // Your code here
}
```

**Optional time parameter:**

By default, `update()` uses `millis()` for timing. You can pass a custom time in milliseconds:

```cpp
double customTime = micros() / 1000.0;  // Use microseconds
system.update(customTime);
```

---

## Practical Examples

### Rocket with Stage Detection

```cpp
class RocketState : public State {
public:
    RocketState(Sensor **s, int n, Filter *f) : State(s, n, f) {}

    void update(double currentTime) override {
        State::update(currentTime);

        int newStage = determineStage();
        if (newStage != stage) {
            stage = newStage;
            onStageChange();
        }
    }

private:
    int stage = 0;

    int determineStage() {
        if (stage == 0 && acceleration.z() > 20) return 1;  // Launch
        if (stage == 1 && velocity.z() < 0) return 2;        // Apogee
        if (stage == 2 && position.z() < 10) return 3;       // Landing
        return stage;
    }

    void onStageChange() {
        LOGI("Stage transition: %d -> %d", stage - 1, stage);

        // Beep stage number
        bb.clearQueue(BUZZER);
        bb.aonoff(BUZZER, 200, stage);
    }
};
```

### Multi-Platform Logging

```cpp
#if defined(ARDUINO_ARCH_ESP32)
    #include <RecordData/Logging/LoggingBackend/FileLogSink.h>
    FileLogSink sdLog("/logs/flight.txt");
#elif defined(ARDUINO_ARCH_STM32)
    #include <RecordData/Logging/LoggingBackend/SD_SDIO.h>
    SD_SDIO sdioLog;
#else
    #include <RecordData/Logging/LoggingBackend/USBLog.h>
    USBLog usbLog;
#endif

USBLog serialLog;

ILogSink *logs[] = {
    #if defined(ARDUINO_ARCH_ESP32)
        &sdLog,
    #elif defined(ARDUINO_ARCH_STM32)
        &sdioLog,
    #else
        &usbLog,
    #endif
    &serialLog
};

config.withDataLogs(logs, 2);
```

### High-Rate IMU with Lower Logging Rate

```cpp
// Update sensors at 200 Hz for responsive state estimation
// Log data at 50 Hz to conserve storage
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUpdateRate(200)
                        .withLoggingRate(50);
```

### Battery Monitoring with DataReporter

```cpp
class BatteryMonitor : public DataReporter {
public:
    BatteryMonitor(int pin) : DataReporter("Battery"), voltagePin(pin) {
        addColumn("%.2f", &voltage, "voltage");
        addColumn("%.1f", &percent, "percent");
    }

    void update() {
        int raw = analogRead(voltagePin);
        voltage = (raw / 1024.0) * 3.3 * 2;  // Voltage divider
        percent = map(voltage, 3.0, 4.2, 0, 100);
    }

private:
    int voltagePin;
    double voltage = 0;
    double percent = 0;
};

BatteryMonitor battery(A0);
DataReporter *reporters[] = {&battery};

config.withOtherDataReporters(reporters);
```

---

## Update Rate Considerations

### Sensor Update Rate

Higher rates provide:
- More responsive state estimation
- Better filtering performance
- Reduced lag in event detection

But consume:
- More CPU time
- More power

**Typical values:**
- **GPS**: 1-10 Hz (limited by sensor)
- **Barometer**: 10-50 Hz
- **IMU**: 50-200 Hz (accelerometer/gyro)

Match your update rate to your fastest sensor.

### Logging Rate

Higher rates provide:
- Better post-flight analysis
- More accurate reconstructions

But consume:
- Storage space
- Write bandwidth

**Typical values:**
- **Low-rate telemetry**: 1-5 Hz
- **Standard telemetry**: 10-20 Hz
- **High-rate telemetry**: 50-100 Hz

**Example storage calculation:**

Assume 100 bytes per log entry:
- 10 Hz × 100 bytes × 60 seconds = 60 KB/minute
- 50 Hz × 100 bytes × 60 seconds = 300 KB/minute

For a 5-minute flight:
- 10 Hz = 300 KB
- 50 Hz = 1.5 MB

---

## Configuration Best Practices

1. **Start simple**: Begin with default rates (10 Hz) and adjust as needed.

2. **Match update rate to sensors**: No benefit updating at 100 Hz if GPS only provides 10 Hz.

3. **Separate update and logging rates**: Update fast for responsive control, log slower to save storage.

4. **Test initialization**: Always check that `init()` succeeds and sensors initialize properly:
   ```cpp
   system.init();
   if (!gps.isInitialized()) {
       LOGE("GPS failed to initialize");
   }
   ```

5. **Monitor timing**: Check if `update()` is keeping up with desired rate:
   ```cpp
   double lastTime = millis();
   void loop() {
       system.update();
       double dt = millis() - lastTime;
       if (dt > 25) {  // Expecting 20ms for 50 Hz
           LOGW("Update slow: %.1f ms", dt);
       }
       lastTime = millis();
   }
   ```

6. **Use appropriate queue sizes**: BlinkBuzz queue size depends on pattern complexity. Simple patterns need 10-20, complex patterns need 50-100.

---

## Common Issues

### "State update rate too slow"

**Cause:** Loop contains delays or blocking operations

**Solution:** Remove `delay()` calls, use non-blocking patterns

### "Log writes failing"

**Cause:** Logging rate too high for storage backend

**Solution:** Reduce logging rate or use faster storage (SDIO vs SPI)

### "Sensors not initializing"

**Cause:** I2C/SPI misconfiguration, hardware issues

**Solution:** Check wiring, I2C addresses, use sensor test programs

### "BlinkBuzz patterns not working"

**Cause:** Queue overflow, insufficient update rate

**Solution:** Increase queue size, ensure `update()` is called frequently

---

## Summary

- `Astra` coordinates sensors, state, logging, and utilities
- Configure via builder-pattern `AstraConfig` class
- `init()` in setup, `update()` in loop
- Control update and logging rates independently
- Support for multiple logging backends simultaneously
- Integrate BlinkBuzz for non-blocking indicators
- Optional sensor bias correction with launch detection

For complete system integration examples, see [Basic Use](../basic-use.md).

---
