# DataReporter

The `DataReporter` interface allows you to add custom data to Astra's CSV telemetry logs. Every sensor and the State object implements this interface, automatically registering their data for logging. You can extend it to log additional data sources like battery monitors, pyro channel status, or custom calculations.

---

## Overview

The `DataReporter` class provides:

1. **Automatic CSV Column Registration** – Define variables once, they're automatically included in telemetry
2. **Type-Safe Logging** – Printf-style format strings ensure correct data formatting
3. **Flexible Column Management** – Add, insert, or remove columns as needed
4. **Automatic Naming** – Optional unique names for each data source

All `Sensor` subclasses and `State` already implement `DataReporter`. You typically only need to extend it for custom data sources.

---

## How It Works

When you create a `DataReporter`:

1. Call `addColumn()` in your constructor to register variables
2. Pass it to `AstraConfig` via `withOtherDataReporters()`
3. The `DataLogger` automatically includes your columns in the CSV
4. Your variables are logged at the configured logging rate

The system uses a linked list of `DataPoint` structures internally, but you only interact with the simple `addColumn()` interface.

---

## Basic Usage

### Creating a Custom Reporter

```cpp
#include "RecordData/DataReporter/DataReporter.h"

using namespace astra;

class BatteryMonitor : public DataReporter {
public:
    BatteryMonitor() : DataReporter("Battery") {
        // Register columns with format strings
        addColumn("%.2f", &voltage, "voltage");
        addColumn("%.1f", &current, "current");
        addColumn("%.1f", &temperature, "temp");
        addColumn("%d", &cellCount, "cells");
    }

    void update() {
        // Read your hardware and update variables
        voltage = readVoltageFromADC();
        current = readCurrentSensor();
        temperature = readTemperatureSensor();
    }

private:
    float voltage = 0.0f;
    float current = 0.0f;
    float temperature = 0.0f;
    int cellCount = 4;

    float readVoltageFromADC() { /* ... */ return 16.8f; }
    float readCurrentSensor() { /* ... */ return 5.2f; }
    float readTemperatureSensor() { /* ... */ return 25.5f; }
};
```

### Registering with Astra

```cpp
BatteryMonitor battery;
DataReporter *reporters[] = {&battery};

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withOtherDataReporters(reporters);

Astra system(&config);

void loop() {
    battery.update();  // Update your data
    system.update();   // Astra logs it automatically
}
```

### CSV Output

Your data appears in the telemetry file:

```csv
TELEM/State - position_x,State - position_y,...,Battery - voltage,Battery - current,Battery - temp,Battery - cells
0.0,0.0,...,16.80,5.2,25.5,4
0.1,0.0,...,16.75,5.3,25.6,4
```

---

## addColumn()

```cpp
template <typename T>
void addColumn(const char *fmt, T *variable, const char *label);
```

Registers a variable for logging.

**Parameters:**
- `fmt`: Printf-style format string (e.g., `"%.2f"` for 2 decimal places, `"%d"` for integers)
- `variable`: Pointer to the variable to log
- `label`: Column label in the CSV header

**Common format strings:**

| Type | Format | Example Output |
|------|--------|----------------|
| `int` | `"%d"` | `42` |
| `float` (2 decimals) | `"%.2f"` | `3.14` |
| `double` (7 decimals) | `"%.7f"` | `-77.1234567` |
| `unsigned int` | `"%u"` | `1024` |
| `bool` | `"%d"` | `1` or `0` |

**Example:**
```cpp
int stage = 0;
float altitude = 0.0f;
double latitude = 39.1234567;
bool armed = false;

addColumn("%d", &stage, "stage");
addColumn("%.1f", &altitude, "alt_m");
addColumn("%.7f", &latitude, "lat");  // GPS coordinates need precision
addColumn("%d", &armed, "armed");
```

---

## insertColumn()

```cpp
template <typename T>
void insertColumn(int place, const char *fmt, T *variable, const char *label);
```

Inserts a column at a specific position (0-indexed).

**Example:**
```cpp
class OrderedReporter : public DataReporter {
public:
    OrderedReporter() : DataReporter("Ordered") {
        addColumn("%.1f", &temp1, "temp1");
        addColumn("%.1f", &temp3, "temp3");

        // Insert temp2 between them
        insertColumn(1, "%.1f", &temp2, "temp2");

        // Order in CSV: temp1, temp2, temp3
    }

private:
    float temp1 = 0.0f;
    float temp2 = 0.0f;
    float temp3 = 0.0f;
};
```

---

## removeColumn()

```cpp
void removeColumn(const char *label);
```

Removes a column by its label. Useful when extending another DataReporter but not wanting all its columns.

**Example:**
```cpp
class SimplifiedIMU : public IMU {
public:
    SimplifiedIMU() {
        // Remove magnetometer data from logging
        removeColumn("mag_x");
        removeColumn("mag_y");
        removeColumn("mag_z");
    }
};
```

---

## getName() and setName()

```cpp
const char *getName() const;
void setName(const char *name);
```

Get or set the reporter's name. The name appears as a prefix in CSV columns.

If no name is provided in the constructor, a unique name is auto-generated using a static counter (`Reporter0`, `Reporter1`, etc.).

**Example:**
```cpp
BatteryMonitor battery;
battery.setName("MainBattery");
// CSV columns: "MainBattery - voltage", "MainBattery - current", etc.
```

---

## Advanced Example: Multi-Instance Reporters

You can create multiple instances of the same reporter class with different names:

```cpp
class TemperatureSensor : public DataReporter {
public:
    TemperatureSensor(int pin, const char *name)
        : DataReporter(name), pin(pin) {
        addColumn("%.1f", &temperature, "temp_c");
    }

    void update() {
        temperature = readTemperature(pin);
    }

private:
    int pin;
    float temperature = 0.0f;

    float readTemperature(int p) { /* ... */ return 25.0f; }
};

// Create multiple sensors
TemperatureSensor boardTemp(A0, "BoardTemp");
TemperatureSensor motorTemp(A1, "MotorTemp");
TemperatureSensor batteryTemp(A2, "BatteryTemp");

DataReporter *reporters[] = {&boardTemp, &motorTemp, &batteryTemp};

void loop() {
    boardTemp.update();
    motorTemp.update();
    batteryTemp.update();
    system.update();
}
```

**CSV Output:**
```csv
...,BoardTemp - temp_c,MotorTemp - temp_c,BatteryTemp - temp_c
...,24.5,67.2,28.1
```

---

## Advanced Example: Computed Values

You can log derived calculations:

```cpp
class FlightMetrics : public DataReporter {
public:
    FlightMetrics(State *state) : DataReporter("Metrics"), state(state) {
        addColumn("%.1f", &maxAltitude, "max_alt_m");
        addColumn("%.1f", &maxVelocity, "max_vel_ms");
        addColumn("%.1f", &maxAccel, "max_accel_ms2");
        addColumn("%.2f", &flightTime, "flight_time_s");
    }

    void update() {
        Vector<3> pos = state->getPosition();
        Vector<3> vel = state->getVelocity();
        Vector<3> accel = state->getAcceleration();

        // Track maximums
        if (pos.z() > maxAltitude) maxAltitude = pos.z();
        if (vel.magnitude() > maxVelocity) maxVelocity = vel.magnitude();
        if (accel.magnitude() > maxAccel) maxAccel = accel.magnitude();

        // Flight time (simple example)
        if (launched) {
            flightTime = (millis() - launchTime) / 1000.0f;
        }
    }

    void setLaunched() {
        launched = true;
        launchTime = millis();
    }

private:
    State *state;
    float maxAltitude = 0.0f;
    float maxVelocity = 0.0f;
    float maxAccel = 0.0f;
    float flightTime = 0.0f;
    bool launched = false;
    unsigned long launchTime = 0;
};
```

---

## How Sensors Use DataReporter

All sensors automatically implement `DataReporter`. Here's a simplified example of how:

```cpp
class GPS : public Sensor {
public:
    GPS() : Sensor("GPS", "MyGPS") {
        // Register GPS-specific columns
        addColumn("%.7f", &latitude, "latitude");
        addColumn("%.7f", &longitude, "longitude");
        addColumn("%.1f", &altitude, "altitude_m");
        addColumn("%d", &numSatellites, "satellites");
        addColumn("%d", &fixQuality, "fix_quality");
    }

protected:
    bool read() override {
        // Read from hardware, update variables
        latitude = readLatitude();
        longitude = readLongitude();
        // ...
        return true;
    }

private:
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0f;
    int numSatellites = 0;
    int fixQuality = 0;
};
```

The sensor's `update()` method is called by `State`, which reads new values. The `DataLogger` automatically includes these values in the CSV.

---

## Internal Structure

For those interested in the implementation, `DataReporter` uses a linked list of `DataPoint` structures:

```cpp
struct DataPoint {
    const char *fmt;                          // Printf format
    const char *label;                        // Column label
    DataPoint *next;                          // Next in list
    const void *data;                         // Pointer to value
    void (*emit)(Print *, const DataPoint *); // Type-erased printer
};
```

Each column you add creates a `DataPoint` node. When logging occurs, the system walks this list and emits formatted values to all configured `ILogSink` destinations.

You don't need to interact with `DataPoint` directly—use the `addColumn()` interface instead.

---

## getDataPoints() and getNumColumns()

```cpp
DataPoint *getDataPoints();
int getNumColumns();
```

These methods provide access to the internal column list. Typically used by the logging system itself. You rarely need to call these directly.

---

## Best Practices

1. **Call addColumn() in constructor**: Register all columns during initialization for cleaner code.

2. **Choose appropriate precision**: GPS coordinates need `%.7f`, temperatures might only need `%.1f`. More precision = more storage used.

3. **Use descriptive labels**: Short but clear. `"lat"` is better than `"l"`, but `"latitude_deg"` might be excessive.

4. **Name your reporters**: Especially when using multiple instances. Makes CSV parsing easier.

5. **Update before system.update()**: Ensure your custom reporters have fresh data before Astra logs it.

6. **Keep update() fast**: Data updates happen frequently. Avoid blocking operations.

7. **Initialize member variables**: Always initialize variables to sensible defaults (0, -1, NaN, etc.).

---

## Integration with DataLogger

The `DataLogger` uses `DataReporter` like this:

1. During initialization, it walks through all reporters calling `getDataPoints()`
2. It writes CSV headers using the `label` fields
3. During each logging cycle, it calls `emit()` on each `DataPoint`
4. The `emit()` function uses the stored format string to write the value

This architecture allows type-safe, efficient logging without runtime type checking.

---

## Summary

- `DataReporter` is an interface for adding custom data to CSV telemetry
- Use `addColumn()` to register variables with format strings
- All sensors and State implement this automatically
- Extend it for custom hardware or computed values
- Columns appear in CSV with `"ReporterName - label"` format
- Called automatically by `DataLogger` when `system.update()` runs

For examples of reporters in action, see [Basic Usage](../basic-use.md#adding-custom-data-reporters) and the [Sensor Interface](sensor.md) documentation.

---
