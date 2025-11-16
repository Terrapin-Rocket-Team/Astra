#ifndef SDCARD_SDIO_BACKEND_H
#define SDCARD_SDIO_BACKEND_H

#if defined(ENV_STM) || defined(ENV_TEENSY)

#include "../../IStorage.h"
#include "SDCardFile.h"
#include <SdFat.h>

namespace astra
{

/**
 * @brief IStorage implementation for SD cards using SDIO interface
 *
 * Supported platforms: STM32, Teensy 4.1
 * Uses SdFat library in SDIO mode for high-speed SD card access.
 *
 * Teensy 4.1: Uses built-in SD card slot
 * STM32: Uses SDIO pins (board-specific configuration)
 */
class SDCardSDIOBackend : public IStorage {
private:
    SdFat _sd;
    bool _initialized;

public:
    SDCardSDIOBackend() : _initialized(false) {}

    bool begin() override {
        // Use SDIO configuration for built-in SD card
        if (!_sd.begin(SdioConfig(FIFO_SDIO))) {
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
};

} // namespace astra

#endif // ENV_STM || ENV_TEENSY

#endif // SDCARD_SDIO_BACKEND_H
