#ifndef SDCARD_SPI_BACKEND_H
#define SDCARD_SPI_BACKEND_H

#include "../../IStorage.h"

#if defined(ENV_STM)
#include "../EMMC/EMMCFile.h"
#include <STM32SD.h>
#include <ff.h>  // For FA_OPEN_APPEND and other FatFs constants
#else
#include "SDCardFile.h"
#include <SdFat.h>
#endif

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
#if defined(ENV_STM)
    bool _initialized;
#else
    SdFat _sd;
    bool _initialized;
#endif
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
#if defined(ENV_STM)
        // On STM32, use STM32SD library with SPI mode
        if (!SD.begin(_csPin, SD_DETECT_NONE)) {
            _initialized = false;
            return false;
        }
        _initialized = true;
        return true;
#else
        // Use SPI configuration with specified CS pin and speed
        if (!_sd.begin(SdSpiConfig(_csPin, SHARED_SPI, SD_SCK_MHZ(_speedMHz)))) {
            _initialized = false;
            return false;
        }
        _initialized = true;
        return true;
#endif
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

#if defined(ENV_STM)
        File file = SD.open(filename, FILE_READ);
        if (!file) return nullptr;
        return new EMMCFile(file);
#else
        FsFile file = _sd.open(filename, O_READ);
        if (!file) return nullptr;
        return new SDCardFile(std::move(file));
#endif
    }

    IFile* openWrite(const char* filename, bool append = true) override {
        if (!_initialized) return nullptr;

#if defined(ENV_STM)
        uint8_t mode = append ? (FILE_WRITE | FA_OPEN_APPEND) : (FILE_WRITE | FA_OPEN_ALWAYS);
        File file = SD.open(filename, mode);
        if (!file) return nullptr;
        return new EMMCFile(file);
#else
        uint8_t mode = append ? (O_WRITE | O_CREAT | O_APPEND) : (O_WRITE | O_CREAT | O_TRUNC);
        FsFile file = _sd.open(filename, mode);
        if (!file) return nullptr;
        return new SDCardFile(std::move(file));
#endif
    }

    bool exists(const char* filename) override {
        if (!_initialized) return false;
#if defined(ENV_STM)
        return SD.exists(filename);
#else
        return _sd.exists(filename);
#endif
    }

    bool remove(const char* filename) override {
        if (!_initialized) return false;
#if defined(ENV_STM)
        return SD.remove(filename);
#else
        return _sd.remove(filename);
#endif
    }

    bool mkdir(const char* path) override {
        if (!_initialized) return false;
#if defined(ENV_STM)
        return SD.mkdir(path);
#else
        return _sd.mkdir(path);
#endif
    }

    bool rmdir(const char* path) override {
        if (!_initialized) return false;
#if defined(ENV_STM)
        return SD.rmdir(path);
#else
        return _sd.rmdir(path);
#endif
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
