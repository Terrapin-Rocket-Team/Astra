#ifndef SDCARD_SPI_BACKEND_H
#define SDCARD_SPI_BACKEND_H

#include "../../IStorage.h"
#include "SDCardFile.h"
#include <SdFat.h>
#include <Arduino.h>

namespace astra
{

/**
 * @brief IStorage implementation for SD cards using SPI interface
 *
 * Supported platforms: STM32, ESP32, Teensy 4.1 (all platforms)
 * Uses SdFat library in SPI mode for SD card access.
 *
 * Platform-specific default CS pins:
 * - Teensy: SS or BUILTIN_SDCARD
 * - ESP32: GPIO 5
 * - STM32: Board-specific (use default SS)
 */
class SDCardSPIBackend : public IStorage {
private:
    SdFat _sd;
    bool _initialized;
    uint8_t _csPin;
    uint32_t _speedMHz;

public:
    /**
     * @brief Construct SD card SPI backend
     * @param csPin Chip select pin (default: platform-specific)
     * @param speedMHz SPI clock speed in MHz (default: 25 MHz)
     */
    SDCardSPIBackend(uint8_t csPin = getDefaultCSPin(), uint32_t speedMHz = 25)
        : _initialized(false), _csPin(csPin), _speedMHz(speedMHz) {}

    bool begin() override {
        // Use SPI configuration with specified CS pin and speed
        if (!_sd.begin(SdSpiConfig(_csPin, SHARED_SPI, SD_SCK_MHZ(_speedMHz)))) {
            _initialized = false;
            return false;
        }
        _initialized = true;
        return true;
    }

    bool end() override {
        _initialized = false;
        return true;
    }

    bool ok() const override {
        return _initialized;
    }

    IFile* openRead(const char* filename) override {
        if (!_initialized) return nullptr;

        FsFile file = _sd.open(filename, O_READ);
        if (!file) return nullptr;

        return new SDCardFile(file);
    }

    IFile* openWrite(const char* filename, bool append = true) override {
        if (!_initialized) return nullptr;

        uint8_t mode = append ? (O_WRITE | O_CREAT | O_APPEND) : (O_WRITE | O_CREAT | O_TRUNC);
        FsFile file = _sd.open(filename, mode);
        if (!file) return nullptr;

        return new SDCardFile(file);
    }

    bool exists(const char* filename) override {
        if (!_initialized) return false;
        return _sd.exists(filename);
    }

    bool remove(const char* filename) override {
        if (!_initialized) return false;
        return _sd.remove(filename);
    }

    bool mkdir(const char* path) override {
        if (!_initialized) return false;
        return _sd.mkdir(path);
    }

    bool rmdir(const char* path) override {
        if (!_initialized) return false;
        return _sd.rmdir(path);
    }

private:
    static uint8_t getDefaultCSPin() {
        #ifdef SDCARD_SS_PIN
        return SDCARD_SS_PIN;  // Built-in SD card slot
        #elif defined(ESP32)
        return 5;  // Common ESP32 SD CS pin
        #else
        return SS;  // Default SPI SS pin
        #endif
    }
};

} // namespace astra

#endif // SDCARD_SPI_BACKEND_H
