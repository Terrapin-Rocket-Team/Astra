# Logging System

Astra provides two separate logging systems that work together to record both telemetry data and system events:

- **DataLogger**: Automatic CSV telemetry recording from sensors and state
- **EventLogger**: Human-readable timestamped messages with severity levels

Both loggers use a flexible backend system (`ILogSink`) that allows you to write to multiple destinations simultaneously: SD cards, internal flash, serial ports, or circular buffers.

---

## Event Logging

Event logging is for human-readable status messages, warnings, and errors. It's perfect for debugging, monitoring system state, and recording important events during operation.

### Basic Usage

Use the convenient macros to log messages:

```cpp
#include "RecordData/Logging/EventLogger.h"

LOGI("System initialized successfully");
LOGW("GPS fix lost, using dead reckoning");
LOGE("Barometer I2C communication failed");
```

### Message Format

By default, event log entries are formatted as:

```
0.123 [INFO]: System initialized successfully
5.678 [WARNING]: GPS fix lost, using dead reckoning
10.456 [ERROR]: Barometer I2C communication failed
```

The timestamp shows seconds since microcontroller startup, followed by the severity level and your message.

### Configuration

Event logging is typically configured early in `setup()`:

```cpp
#include "RecordData/Logging/EventLogger.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"

UARTLog serialLog(Serial, 115200, true);
FileLogSink fileLog("LOG.txt", StorageBackend::SD_SPI, true);
ILogSink *logs[] = {&serialLog, &fileLog};

void setup() {
    EventLogger::configure(logs, 2);

    LOGI("Logging started");
}
```

In this example, messages go to both serial and a file simultaneously.

### Using with AstraConfig

When using the `Astra` system, you can configure event logging through `AstraConfig`:

```cpp
UARTLog serialLog(Serial, 115200);
FileLogSink fileLog("LOG.txt", StorageBackend::SD_SPI);
ILogSink *logs[] = {&serialLog, &fileLog};

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withDataLogs(logs, 2);  // Event and data loggers share sinks

Astra system(&config);
```

### Available Log Sinks

Astra provides several built-in log sink implementations:

#### UARTLog
Logs to hardware serial ports:

```cpp
UARTLog log(Serial1, 115200, true);  // Hardware serial, 115200 baud, with prefixes
```

#### USBLog
Logs to USB serial:

```cpp
USBLog log(Serial, 9600, true);  // USB serial, 9600 baud, with prefixes
```

#### FileLogSink
Logs to storage (SD card, internal flash, or eMMC):

```cpp
// Automatic backend creation
FileLogSink log("LOG.txt", StorageBackend::SD_SPI, true);

// Available backend types:
// - StorageBackend::EMMC
// - StorageBackend::SD_SDIO
// - StorageBackend::SD_SPI
```

#### CircBufferLog
Logs to a circular buffer in memory (useful for buffering before storage is ready):

```cpp
CircBufferLog buffer(5000, true);  // 5000 byte buffer with prefixes

// Later, transfer buffered data to another sink
buffer.transfer(serialLog);
```

#### PrintLog
Logs to any Arduino `Print` object:

```cpp
PrintLog customLog(myPrintObject, true);
```

### Example: Buffered Logging with Handshake

This pattern is useful when you want to buffer logs until an external system (like a Pi) is ready:

```cpp
CircBufferLog bufferLog(5000, true);
UARTLog serialLog(Serial1, 115200, true);
ILogSink *bufferSinks[] = {&bufferLog};
ILogSink *serialSinks[] = {&serialLog};

void setup() {
    // Start logging to buffer
    EventLogger::configure(bufferSinks, 1);

    LOGI("System starting up...");
    system.init();
}

bool handshakeDone = false;

void loop() {
    if (!handshakeDone && Serial1.available()) {
        String cmd = Serial1.readStringUntil('\n');
        if (cmd == "PING") {
            Serial1.println("PONG");

            // Switch to serial logging and transfer buffer
            EventLogger::configure(serialSinks, 1);
            bufferLog.transfer(serialLog);

            LOGI("Handshake complete, buffer transferred");
            handshakeDone = true;
        }
    }

    system.update();
}
```

---

## Data Logging

Data logging automatically records CSV telemetry from all sensors and the state object. Unlike event logging, you don't manually log data—it happens automatically when `system.update()` is called.

### How It Works

Every `DataReporter` (including `State` and all `Sensor` subclasses) registers its data columns automatically. The `DataLogger` handles:

1. Writing CSV headers on startup
2. Recording data at the configured logging rate
3. Writing to multiple destinations simultaneously

### CSV Format

The CSV file includes a header row followed by data rows:

```csv
TELEM/State - position_x,State - position_y,State - position_z,GPS - latitude,GPS - longitude,...
0.000,0.000,0.000,39.123,-77.456,...
0.100,0.001,0.002,39.123,-77.456,...
```

Each column is prefixed with the data reporter's name (e.g., "State", "GPS", "Baro1") followed by the variable name.

### Configuration

Data logging is configured via `AstraConfig`:

```cpp
FileLogSink dataFile("TELEM.csv", StorageBackend::SD_SDIO, true);
ILogSink *dataSinks[] = {&dataFile};

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withDataLogs(dataSinks, 1)
                        .withLoggingRate(20);  // Log at 20Hz

Astra system(&config);
```

### Separating Update and Logging Rates

You can read sensors more frequently than you log data:

```cpp
AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUpdateRate(50)    // Read sensors at 50Hz
                        .withLoggingRate(20);  // Write data at 20Hz
```

This conserves storage while maintaining high sensor update rates for control algorithms.

### Adding Custom Data Reporters

To log custom data beyond sensors and state, implement the `DataReporter` interface:

```cpp
#include "RecordData/DataReporter/DataReporter.h"

class BatteryMonitor : public DataReporter {
public:
    BatteryMonitor() : DataReporter("Battery") {
        addColumn("%.2f", &voltage, "voltage");
        addColumn("%.1f", &current, "current");
        addColumn("%.1f", &temperature, "temp");
    }

    void update() {
        voltage = readVoltage();
        current = readCurrent();
        temperature = readTemperature();
    }

private:
    float voltage = 0.0f;
    float current = 0.0f;
    float temperature = 0.0f;

    float readVoltage() { /* ... */ }
    float readCurrent() { /* ... */ }
    float readTemperature() { /* ... */ }
};
```

Then add it to `AstraConfig`:

```cpp
BatteryMonitor battery;
DataReporter *reporters[] = {&battery};

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withOtherDataReporters(reporters);
```

Your custom columns will appear in the CSV automatically.

---

## Log Sink Prefixes

Some log sinks support automatic prefixing to distinguish between different data types in the same file or stream.

When `wantsPrefix()` returns `true`, the logging system prepends:

- **`LOG/`** for event log messages
- **`TELEM/`** for CSV telemetry data

This is particularly useful when logging everything to a single serial connection where you need to parse different message types.

Example output with prefixes enabled:

```
LOG/0.123 [INFO]: System initialized
TELEM/State - x,State - y,GPS - lat,GPS - lon
TELEM/0.0,0.0,39.123,-77.456
LOG/1.234 [WARNING]: GPS fix lost
TELEM/0.1,0.2,39.123,-77.456
```

---

## Storage Backends

Astra's flexible storage system adapts to your hardware automatically:

### Available Backends

| Backend | Platform Support | Use Case |
|---------|-----------------|----------|
| `SD_SPI` | All platforms | SD cards via SPI (universally compatible) |
| `SD_SDIO` | STM32, Teensy 4.x | SD cards via SDIO (faster than SPI) |
| `EMMC` | Some STM32 boards | Soldered eMMC storage (most robust) |

### Automatic Selection

The `StorageFactory` can automatically select the best available backend:

```cpp
IStorage *storage = StorageFactory::create(StorageBackend::AUTO);
```

Priority order: EMMC → SD_SDIO → SD_SPI

### Platform-Specific Notes

**Teensy:**
- Supports SD_SPI via built-in SD card slot
- Teensy 4.x supports SD_SDIO for improved performance

**STM32:**
- SD_SDIO requires specific pin configurations (check your board's documentation)
- Some boards have built-in eMMC for maximum reliability

**ESP32:**
- Primarily SD_SPI support
- Some boards support SDIO mode with compatible SD cards

---

## File Management

### Automatic File Naming

Log files are automatically numbered to avoid overwriting existing data:

```
001_LOG.txt
002_LOG.txt
001_TELEM.csv
002_TELEM.csv
```

The number increments based on existing files in storage.

### Retrieving Data

For serial retrieval of logged data, see the [RetrieveData utility](retrieve-data.md) documentation.

---

## Best Practices

1. **Always check logger status:**
   ```cpp
   if (EventLogger::available()) {
       LOGI("Logger ready");
   }
   ```

2. **Use appropriate log levels:**
   - `LOGI()` for normal operation
   - `LOGW()` for recoverable issues
   - `LOGE()` for critical failures

3. **Buffer early logs:**
   Start with a `CircBufferLog` until storage is verified, then transfer buffered messages

4. **Match logging rates to needs:**
   High-frequency logging fills storage quickly. Use 10-20 Hz unless you have specific requirements.

5. **Test storage before flight:**
   Verify your storage backend is working in ground tests. A simple `LOGI()` message will fail gracefully if storage isn't available.

6. **Use SDIO when possible:**
   SDIO is faster and more reliable than SPI for SD cards on supported platforms

---

## Advanced: Manual Logger Configuration

If you're not using the `Astra` system, you can configure loggers manually:

```cpp
#include "RecordData/Logging/EventLogger.h"
#include "RecordData/Logging/DataLogger.h"

UARTLog serialLog(Serial, 115200, true);
FileLogSink dataFile("TELEM.csv", StorageBackend::SD_SPI, true);

ILogSink *eventSinks[] = {&serialLog};
ILogSink *dataSinks[] = {&dataFile};

void setup() {
    // Configure event logging
    EventLogger::configure(eventSinks, 1);

    // Configure data logging
    State *state = &vehicleState;
    DataReporter *reporters[] = {state};
    for (int i = 0; i < state->getNumMaxSensors(); i++) {
        reporters[i+1] = state->getSensors()[i];
    }
    DataLogger::configure(dataSinks, 1, reporters, numReporters);

    if (!DataLogger::available()) {
        LOGE("Data logger failed to initialize");
    }
}

void loop() {
    // Update sensors and state
    vehicleState.update();

    // Manually log data
    DataLogger::instance().appendLine();
}
```

Using `Astra` and `AstraConfig` handles all of this automatically.

---
