#ifndef TEENSY_SDCARD_BACKEND_H
#define TEENSY_SDCARD_BACKEND_H

#include "../../IStorage.h"
#include "TeensyFile.h"
#include <SdFat.h>

namespace astra
{

/**
 * @brief IStorage implementation for SD cards using Teensy SDIO interface
 *
 * Uses SdFat library to access SD cards via SDIO interface.
 * Pins are defined by the Teensy board variant.
 */
class SDCardBackend : public IStorage {
private:
    SdFat _sd;
    bool _initialized;

public:
    SDCardBackend() : _initialized(false) {}

    bool begin() override {
        // Use SdFat with SDIO configuration
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
        return new TeensyFile(std::move(file));
    }

    IFile* openWrite(const char* filename, bool append = true) override {
        if (!_initialized) return nullptr;

        uint8_t mode = append ? (O_WRITE | O_CREAT | O_APPEND) : (O_WRITE | O_CREAT | O_TRUNC);
        FsFile file = _sd.open(filename, mode);
        if (!file) return nullptr;
        return new TeensyFile(std::move(file));
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

#endif // TEENSY_SDCARD_BACKEND_H
