# Multi-Platform Storage System

A unified storage abstraction layer for Astra that supports multiple platforms (STM32, ESP32, Teensy 4.1) and storage backends (eMMC, SD card via SPI/SDIO, onboard flash).

## Architecture

The system uses a layered architecture:

```
Application Layer (EventLogger, DataLogger)
    ↓
ILogSink (FileLogSink adapter)
    ↓
IStorage (filesystem operations) → creates → IFile (file handles)
    ↓
Platform-Specific Backends (EMMCBackend, SDCardBackend, etc.)
    ↓
Hardware Libraries (STM32SD, SdFat, LittleFS)
```

## Core Interfaces

### IFile - File Handle
Represents a single open file with read/write operations.

```cpp
class IFile {
    virtual size_t write(uint8_t b) = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) = 0;
    virtual bool flush() = 0;
    virtual int read() = 0;
    virtual int readBytes(uint8_t* buffer, size_t length) = 0;
    virtual bool close() = 0;
    // ... more methods
};
```

### IStorage - Storage Backend
Represents a storage medium (eMMC, SD card, flash) and manages the filesystem.

```cpp
class IStorage {
    virtual bool begin() = 0;
    virtual IFile* openRead(const char* filename) = 0;
    virtual IFile* openWrite(const char* filename, bool append = true) = 0;
    virtual bool exists(const char* filename) = 0;
    // ... more methods
};
```

### StorageBackend Enum
Platform-specific enum for selecting storage type at runtime.

```cpp
enum class StorageBackend {
    #if defined(ENV_STM)
    EMMC,      // eMMC via MMC interface
    SD_SDIO,   // SD card via SDIO
    SD_SPI,    // SD card via SPI
    FLASH,     // Onboard flash (future)
    #elif defined(ENV_ESP)
    SD_SPI,    // SD card via SPI
    FLASH,     // Onboard flash (future)
    #elif defined(ENV_TEENSY)
    SD_SDIO,   // Built-in SD slot
    SD_SPI,    // SD card via SPI
    FLASH,     // Onboard flash (future)
    #endif
};
```

## Platform Support Matrix

| Platform    | SD (SPI) | SD (SDIO) | eMMC | Flash (LittleFS) |
|-------------|----------|-----------|------|------------------|
| STM32       | ✅       | ✅        | ✅   | 🔮 Future        |
| ESP32       | ✅       | ❌        | ❌   | 🔮 Future        |
| Teensy 4.1  | ✅       | ✅        | ❌   | 🔮 Future        |

## Usage Examples

### Basic Logging with FileLogSink

```cpp
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
using namespace astra;

void setup() {
    // Create file log sinks (automatically creates storage backend)
    FileLogSink eventLog("events.log", StorageBackend::EMMC);
    FileLogSink telemLog("telemetry.csv", StorageBackend::EMMC);

    // Configure EventLogger
    ILogSink* sinks[] = {&eventLog, &telemLog};
    EventLogger::configure(sinks, 2);

    // Use logging macros
    LOGI("System started");
    LOGW("Temperature high: 85C");
    LOGE("Sensor failure!");
}
```

### Redundant Logging (Same Data to Multiple Backends)

```cpp
// Write same data to both eMMC and flash for redundancy
FileLogSink primaryLog("flight.log", StorageBackend::EMMC);
FileLogSink backupLog("flight.log", StorageBackend::FLASH);

ILogSink* sinks[] = {&primaryLog, &backupLog};
EventLogger::configure(sinks, 2);

// All logs go to both eMMC and flash!
LOGI("Mission critical data");
```

### High-Rate Telemetry Logging

```cpp
#include "RecordData/Storage/StorageFactory.h"
using namespace astra;

IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);
emmc->begin();

// Open file once, keep it open for high-rate writes
IFile* telemFile = emmc->openWrite("telemetry.csv", true);

void loop() {
    static int writeCount = 0;

    // Format CSV line
    char buffer[128];
    int len = snprintf(buffer, 128, "%lu,%.2f,%.2f,%.2f\n",
                       millis(), temp, pressure, altitude);

    // Write to file (no open/close overhead!)
    telemFile->write((uint8_t*)buffer, len);

    // Flush periodically (every 100 writes)
    if (++writeCount % 100 == 0) {
        telemFile->flush();
    }
}

void shutdown() {
    telemFile->flush();
    telemFile->close();
    delete telemFile;
    emmc->end();
    delete emmc;
}
```

### Multiple Files Open Simultaneously

```cpp
IStorage* sd = StorageFactory::create(StorageBackend::SD_SDIO);
sd->begin();

// Open multiple files at the same time
IFile* events = sd->openWrite("events.log", true);
IFile* telem = sd->openWrite("telemetry.csv", true);
IFile* config = sd->openRead("config.txt");

// Write to different files independently
events->write("System started\n");
telem->write("0,25.3,101325\n");

// Read from config
uint8_t buffer[256];
config->readBytes(buffer, 256);

// Clean up
events->close();
telem->close();
config->close();
delete events;
delete telem;
delete config;
```

### File Reading and Retrieval

```cpp
#include "RecordData/Retrieval/FileReader.h"
using namespace astra;

IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);
emmc->begin();

FileReader reader(emmc);

// Print entire file to Serial
reader.printFile("flight_001.csv");

// Process file line-by-line
reader.readLines("data.log", [](const char* line) {
    Serial.print("Line: ");
    Serial.println(line);
});

// Check if file exists
if (reader.exists("config.txt")) {
    Serial.println("Config found!");
}

// Delete old files
reader.deleteFile("old_data.log");

// Interactive CLI for file operations
reader.handleCommands();
// Commands: cmd/sf <file>, cmd/rm <file>, cmd/help, cmd/quit
```

### Platform-Specific Defaults

```cpp
// Each platform can have different defaults based on priority

#if defined(ENV_STM)
    // STM32: eMMC is highest priority
    FileLogSink log("data.log", StorageBackend::EMMC);

#elif defined(ENV_TEENSY)
    // Teensy: Built-in SD slot via SDIO
    FileLogSink log("data.log", StorageBackend::SD_SDIO);

#elif defined(ENV_ESP)
    // ESP32: SD card via SPI
    FileLogSink log("data.log", StorageBackend::SD_SPI);
#endif
```

### Advanced: Shared Backend Instance

```cpp
// Create one backend instance, share it across multiple FileLogSinks
IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);
emmc->begin();

// Both sinks share the same backend
FileLogSink eventLog("events.log", emmc);
FileLogSink telemLog("telemetry.csv", emmc);

// Both write to different files on the same eMMC
ILogSink* sinks[] = {&eventLog, &telemLog};
EventLogger::configure(sinks, 2);
```

## Implementation Details

### Backend Implementations

**STM32:**
- `EMMCBackend` - Uses STM32SD library for eMMC/SD via MMC interface
- `SDCardSDIOBackend` - Uses SdFat SDIO mode
- `SDCardSPIBackend` - Uses SdFat SPI mode

**ESP32:**
- `SDCardSPIBackend` - Uses SdFat SPI mode

**Teensy 4.1:**
- `SDCardSDIOBackend` - Uses SdFat SDIO for built-in SD slot
- `SDCardSPIBackend` - Uses SdFat SPI mode

### Pin Configurations (Hardcoded)

**STM32 eMMC (MMC interface):**
- PC8: D0
- PC9: D1
- PC10: D2
- PC11: D3
- PC12: CLK
- PD2: CMD

**SD Card SPI:**
- Platform-specific CS pin (configurable in constructor)
- Default: SDCARD_SS_PIN or SS

**SD Card SDIO:**
- Teensy 4.1: Built-in SD slot (automatic)
- STM32: Board-specific SDIO pins

## File Structure

```
src/RecordData/Storage/
├── IFile.h                           # File handle interface
├── IStorage.h                        # Storage backend interface + StorageBackend enum
├── StorageFactory.h                  # Factory for creating backends
├── StorageFactory.cpp                # Factory implementation
├── Backends/
│   ├── EMMC/
│   │   ├── EMMCFile.h                # IFile wrapper for STM32SD
│   │   └── EMMCBackend.h             # IStorage implementation (STM32 only)
│   ├── SDCard/
│   │   ├── SDCardFile.h              # IFile wrapper for SdFat
│   │   ├── SDCardSPIBackend.h        # SPI mode (all platforms)
│   │   └── SDCardSDIOBackend.h       # SDIO mode (STM32, Teensy)
│   └── Flash/
│       └── (Future: LittleFS support)
└── README.md                         # This file

src/RecordData/Logging/LoggingBackend/
├── ILogSink.h                        # Logging interface (includes FileLogSink)
└── FileLogSink.cpp                   # FileLogSink implementation

src/RecordData/Retrieval/
└── FileReader.h                      # File reading utilities
```

## Design Principles

1. **IFile = file handle, IStorage = filesystem**
2. **Multiple files can be open simultaneously** via separate IFile instances
3. **FileLogSink wraps IStorage + IFile** for logging system integration
4. **StorageBackend enum is platform-specific** (won't compile with unsupported types)
5. **Factory pattern hides backend creation** complexity
6. **#ifdef guards ensure only compatible backends compile** per platform
7. **Keep files open for high-rate logging** performance (flush periodically)

## Migration Guide

### From Old FileHandler/SDMMCBackend

**Old code:**
```cpp
FileHandler file(StorageMedium::EMMC);
file.begin();
file.write("data");
```

**New code:**
```cpp
IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);
emmc->begin();
IFile* file = emmc->openWrite("data.log");
file->write("data");
file->close();
```

**Or use FileLogSink for logging:**
```cpp
FileLogSink log("data.log", StorageBackend::EMMC);
log.begin();
log.write("data");
```

### From Old RetrieveSDCardData

**Old code:**
```cpp
RetrieveSDCardData retriever;
retriever.printFile("data.csv");
```

**New code:**
```cpp
IStorage* sd = StorageFactory::create(StorageBackend::SD_SPI);
sd->begin();
FileReader reader(sd);
reader.printFile("data.csv");
```

## Future Enhancements

- ✅ Phase 1: Core interfaces and factory (DONE)
- ✅ Phase 2: Priority 1 backends - Teensy SD, STM32 eMMC (DONE)
- ✅ Phase 3: FileLogSink integration (DONE)
- ✅ Phase 4: Priority 2 backends - ESP32 SD SPI (DONE)
- ✅ Phase 5: FileReader for retrieval (DONE)
- 🔮 Phase 6: Flash storage via LittleFS (all platforms)
- 🔮 Phase 7: Directory listing and iteration
- 🔮 Phase 8: File searching and filtering
- 🔮 Phase 9: Configurable pin assignments via constructors
