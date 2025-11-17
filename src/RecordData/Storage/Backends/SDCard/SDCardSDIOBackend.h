#ifndef SDCARD_SDIO_BACKEND_H
#define SDCARD_SDIO_BACKEND_H

#if defined(ENV_STM) || defined(ENV_TEENSY)

#include "../../IStorage.h"

#if defined(ENV_STM)
#include "../EMMC/EMMCFile.h"
#include <STM32SD.h>
#include <ff.h>  // For FA_OPEN_APPEND and other FatFs constants
#else
#include "SDCardFile.h"
#include <SdFat.h>
#endif

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
#if defined(ENV_STM)
    bool _initialized;
#else
    SdFat _sd;
    bool _initialized;
#endif

public:
    SDCardSDIOBackend() : _initialized(false) {}

    bool begin() override {
#if defined(ENV_STM)
        // On STM32, use STM32SD library for SDIO
        if (!SD.begin(SD_DETECT_NONE)) {
            _initialized = false;
            return false;
        }
        _initialized = true;
        return true;
#else
        // Use SDIO configuration for built-in SD card
        if (!_sd.begin(SdioConfig(FIFO_SDIO))) {
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
};

} // namespace astra

#endif // ENV_STM || ENV_TEENSY

#endif // SDCARD_SDIO_BACKEND_H
