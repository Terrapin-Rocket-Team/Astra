# Logging System

Astra has two independent logging channels:

- **EventLogger** → human-readable status (`LOG/` prefix when enabled)
- **DataLogger** → CSV telemetry (`TELEM/` prefix when enabled)

Both log through `ILogSink` backends.

---

## EventLogger (LOG/)

### Configure

```cpp
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

UARTLog eventLog(Serial, 115200, true);
ILogSink* eventSinks[] = { &eventLog };

void setup() {
    EventLogger::configure(eventSinks, 1);
}
```

Alternatively, pass the same sinks to `AstraConfig.withEventLogs()` and let Astra configure it during `init()`.

### Use

```cpp
LOGI("System initialized");
LOGW("GPS fix lost");
LOGE("Barometer init failed");
```

---

## DataLogger (TELEM/)

`DataLogger` writes CSV telemetry from every `DataReporter`. Sensors and `State` register automatically.

### Configure via AstraConfig

```cpp
FileLogSink telemFile("TELEM.csv", StorageBackend::SD_CARD, true);
ILogSink* telemSinks[] = { &telemFile };

AstraConfig config = AstraConfig()
    .withState(&state)
    .withDataLogs(telemSinks, 1);
```

### Header Timing

The telemetry header is written **once** at logger initialization. Ensure all `DataReporter`s are constructed before `Astra::init()` (or before `DataLogger::configure()` if used directly).

---

## ILogSink Backends

### UARTLog / USBLog

```cpp
UARTLog log1(Serial1, 115200, true);
USBLog log2(Serial, 115200, true);
```

### PrintLog

```cpp
PrintLog log(myPrint, true);
```

### CircBufferLog

```cpp
CircBufferLog buf(5000, true);
```

`CircBufferLog::transfer()` can flush buffered data into another sink later.

### FileLogSink

```cpp
FileLogSink log("TELEM.csv", StorageBackend::SD_CARD, true);
```

`FileLogSink` automatically finds a unique filename:

```
TELEM.csv
TELEM_1.csv
TELEM_2.csv
```

---

## Storage Backends

The storage backend enum is platform-specific:

| Platform | StorageBackend values |
|---------|------------------------|
| STM32   | `EMMC`, `SD_CARD` |
| ESP32   | `SD_CARD` |
| Teensy | `SD_CARD` |
| Native | `NONE` |

See `IStorage` and `StorageFactory` for backend details.

---

## Prefixes

If `wantsPrefix()` is enabled on a sink:

- `EventLogger` writes `LOG/`
- `DataLogger` writes `TELEM/`

This is useful when multiplexing over a shared serial link.

---

## Reconfiguring Loggers

You can reconfigure loggers at runtime if a new sink becomes available:

```cpp
EventLogger::configure(newEventSinks, count);
DataLogger::configure(newDataSinks, count);
```

This is useful for plug‑in devices (e.g., a ground station that connects after boot).
