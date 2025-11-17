/**
 * Example usage of the multi-platform storage system
 *
 * This file demonstrates various use cases for the storage system.
 * These are NOT meant to be compiled directly, but serve as copy-paste
 * examples for your own application code.
 */

#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "RecordData/Logging/EventLogger.h"
#include "RecordData/Storage/StorageFactory.h"
#include "RecordData/Retrieval/FileReader.h"

using namespace astra;

// ============================================================================
// Example 1: Basic Logging with FileLogSink
// ============================================================================

void example1_basicLogging() {
    // Create file log sink (backend is created automatically)
    FileLogSink eventLog("events.log", StorageBackend::EMMC);

    // Begin the log sink (initializes backend and opens file)
    if (!eventLog.begin()) {
        Serial.println("Failed to initialize event log!");
        return;
    }

    // Configure EventLogger to use our file sink
    ILogSink* sinks[] = {&eventLog};
    EventLogger::configure(sinks, 1);

    // Now all log messages go to the file
    LOGI("System initialized");
    LOGW("Battery low: 15%");
    LOGE("Critical sensor failure!");

    // Clean up
    eventLog.end();
}

// ============================================================================
// Example 2: Redundant Logging (Multiple Backends)
// ============================================================================

void example2_redundantLogging() {
    // Write to both eMMC and flash for safety
    FileLogSink primaryLog("flight.log", StorageBackend::EMMC);
    FileLogSink backupLog("flight.log", StorageBackend::INTERNAL_FLASH);

    // Begin both
    primaryLog.begin();
    backupLog.begin();

    // Configure logger with both sinks
    ILogSink* sinks[] = {&primaryLog, &backupLog};
    EventLogger::configure(sinks, 2);

    // All messages go to BOTH backends!
    LOGI("Mission critical data logged to dual storage");

    primaryLog.end();
    backupLog.end();
}

// ============================================================================
// Example 3: High-Rate Telemetry with Multiple Files
// ============================================================================

void example3_highRateTelemetry() {
    // Create storage backend
    IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);
    if (!emmc->begin()) {
        Serial.println("Failed to initialize eMMC!");
        return;
    }

    // Open multiple files simultaneously
    IFile* telemFile = emmc->openWrite("telemetry.csv", true);
    IFile* eventFile = emmc->openWrite("events.log", true);

    if (!telemFile || !eventFile) {
        Serial.println("Failed to open files!");
        return;
    }

    // Write CSV header
    telemFile->write((const uint8_t*)"time,temp,pressure,altitude\n", 28);

    // High-rate logging loop (e.g., 1000 Hz)
    int writeCount = 0;
    for (int i = 0; i < 10000; i++) {
        // Simulate sensor readings
        float temp = 25.0 + random(-100, 100) / 100.0;
        float pressure = 101325.0 + random(-1000, 1000);
        float altitude = 1000.0 + random(-100, 100);

        // Format CSV line
        char buffer[128];
        int len = snprintf(buffer, 128, "%lu,%.2f,%.2f,%.2f\n",
                           millis(), temp, pressure, altitude);

        // Write to telemetry file (stays open for performance!)
        telemFile->write((uint8_t*)buffer, len);

        // Occasional event logging
        if (i % 1000 == 0) {
            eventFile->write((const uint8_t*)"Checkpoint reached\n", 19);
        }

        // Flush periodically (every 100 writes = ~100ms at 1kHz)
        if (++writeCount % 100 == 0) {
            telemFile->flush();
            eventFile->flush();
        }
    }

    // Final flush and cleanup
    telemFile->flush();
    eventFile->flush();
    telemFile->close();
    eventFile->close();
    delete telemFile;
    delete eventFile;

    emmc->end();
    delete emmc;
}

// ============================================================================
// Example 4: File Reading and Retrieval
// ============================================================================

void example4_fileRetrieval() {
    // Create storage backend
    IStorage* sd = StorageFactory::create(StorageBackend::SD_SDIO);
    if (!sd->begin()) {
        Serial.println("Failed to initialize SD card!");
        return;
    }

    FileReader reader(sd);

    // Check if file exists
    if (reader.exists("flight_001.csv")) {
        Serial.println("Flight data found!");

        // Print entire file to Serial
        reader.printFile("flight_001.csv");

        // Or process line by line
        reader.readLines("flight_001.csv", [](const char* line) {
            // Parse CSV, process data, etc.
            Serial.print("Line: ");
            Serial.println(line);
        });
    }

    // Delete old files
    if (reader.exists("old_data.log")) {
        reader.deleteFile("old_data.log");
        Serial.println("Deleted old data");
    }

    sd->end();
    delete sd;
}

// ============================================================================
// Example 5: Interactive File Browser
// ============================================================================

void example5_interactiveBrowser() {
    IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);
    if (!emmc->begin()) {
        Serial.println("Failed to initialize eMMC!");
        return;
    }

    FileReader reader(emmc);

    // Start interactive CLI
    // User can type commands like:
    //   cmd/sf flight.csv  - Show file
    //   cmd/rm old.log     - Remove file
    //   cmd/help           - Show help
    //   cmd/quit           - Exit
    reader.handleCommands();

    emmc->end();
    delete emmc;
}

// ============================================================================
// Example 6: Platform-Specific Auto-Selection
// ============================================================================

void example6_platformSpecific() {
    // Choose best backend for each platform automatically
    #if defined(ENV_STM)
    const StorageBackend BEST_BACKEND = StorageBackend::EMMC;
    #elif defined(ENV_TEENSY)
    const StorageBackend BEST_BACKEND = StorageBackend::SD_SDIO;
    #elif defined(ENV_ESP)
    const StorageBackend BEST_BACKEND = StorageBackend::SD_SPI;
    #endif

    FileLogSink log("data.log", BEST_BACKEND);
    log.begin();

    // Now log to the best backend for this platform
    log.write("Platform-optimized logging\n");

    log.end();
}

// ============================================================================
// Example 7: Shared Backend, Multiple FileLogSinks
// ============================================================================

void example7_sharedBackend() {
    // Create one backend instance
    IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);
    if (!emmc->begin()) return;

    // Multiple FileLogSinks share the same backend
    // (Each opens a different file)
    FileLogSink eventLog("events.log", emmc);
    FileLogSink telemLog("telemetry.csv", emmc);
    FileLogSink debugLog("debug.log", emmc);

    eventLog.begin();
    telemLog.begin();
    debugLog.begin();

    // Use them with EventLogger and DataLogger
    ILogSink* eventSinks[] = {&eventLog, &debugLog};
    EventLogger::configure(eventSinks, 2);

    ILogSink* telemSinks[] = {&telemLog};
    DataLogger::configure(telemSinks, 1);

    // Logging works normally
    LOGI("System operational");

    // Clean up (note: don't delete emmc, FileLogSinks don't own it)
    eventLog.end();
    telemLog.end();
    debugLog.end();
    emmc->end();
    delete emmc;
}

// ============================================================================
// Example 8: Manual File Operations (Low-Level)
// ============================================================================

void example8_manualFileOps() {
    IStorage* sd = StorageFactory::create(StorageBackend::SD_SPI);
    if (!sd->begin()) return;

    // Create a file
    IFile* file = sd->openWrite("config.txt", false);  // false = overwrite
    if (file) {
        file->write((const uint8_t*)"setting1=100\n", 13);
        file->write((const uint8_t*)"setting2=200\n", 13);
        file->flush();
        file->close();
        delete file;
    }

    // Read the file back
    file = sd->openRead("config.txt");
    if (file) {
        char buffer[64];
        int bytesRead = file->readBytes((uint8_t*)buffer, sizeof(buffer));
        buffer[bytesRead] = '\0';
        Serial.println(buffer);

        file->close();
        delete file;
    }

    // Check file operations
    if (sd->exists("config.txt")) {
        Serial.println("Config file exists");
    }

    // Create directory
    sd->mkdir("data");

    // Remove file
    sd->remove("old_file.txt");

    sd->end();
    delete sd;
}

// ============================================================================
// Example 9: Error Handling
// ============================================================================

void example9_errorHandling() {
    // Create backend
    IStorage* emmc = StorageFactory::create(StorageBackend::EMMC);

    if (!emmc) {
        Serial.println("ERROR: Backend not supported on this platform!");
        return;
    }

    if (!emmc->begin()) {
        Serial.println("ERROR: Failed to initialize storage!");
        delete emmc;
        return;
    }

    if (!emmc->ok()) {
        Serial.println("ERROR: Storage not ready!");
        emmc->end();
        delete emmc;
        return;
    }

    // Try to open file
    IFile* file = emmc->openWrite("test.log");
    if (!file) {
        Serial.println("ERROR: Failed to open file!");
        emmc->end();
        delete emmc;
        return;
    }

    if (!file->isOpen()) {
        Serial.println("ERROR: File not open!");
        delete file;
        emmc->end();
        delete emmc;
        return;
    }

    // Use file safely
    size_t written = file->write((const uint8_t*)"test", 4);
    if (written != 4) {
        Serial.println("WARNING: Incomplete write!");
    }

    // Always clean up
    file->close();
    delete file;
    emmc->end();
    delete emmc;

    Serial.println("Success!");
}
