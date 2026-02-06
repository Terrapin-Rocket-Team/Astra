# DataReporter

`DataReporter` is the telemetry interface. Any class that inherits from it can register CSV columns and be logged by `DataLogger`.

Sensors and `State` already inherit from `DataReporter`.

---

## How It Works

1. Create a `DataReporter`
2. Register columns in the constructor
3. Implement `begin()` and `update()`
4. The reporter auto-registers with `DataLogger`

`DataLogger` will call `update()` automatically **if `autoUpdate` is true**.

---

## Minimal Example

```cpp
#include <RecordData/DataReporter/DataReporter.h>

using namespace astra;

class BatteryMonitor : public DataReporter {
public:
    BatteryMonitor() : DataReporter("Battery") {
        addColumn("%.2f", &voltage, "voltage_v");
        addColumn("%.1f", &tempC, "temp_c");
    }

    int begin() override { return 0; }

    int update(double currentTime = -1) override {
        voltage = readVoltage();
        tempC = readTemp();
        return 0;
    }

private:
    float voltage = 0.0f;
    float tempC = 0.0f;

    float readVoltage() { return 12.4f; }
    float readTemp() { return 25.0f; }
};
```

Construct this before `Astra::init()` so the header includes its columns.

---

## Column Registration

```cpp
addColumn("%.3f", &value, "label");
insertColumn(0, "%.1f", &other, "first");
removeColumn("label");
```

Formats use `printf` rules.

---

## Auto Update Control

```cpp
reporter.setAutoUpdate(false);
```

If `autoUpdate` is false, you must call `update()` yourself before telemetry logging.

Sensors and `State` set `autoUpdate = false` because Astra updates them separately.

