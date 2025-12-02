#ifndef STM32_EMMC_BACKEND_H
#define STM32_EMMC_BACKEND_H

#include "../../IStorage.h"
#include "STM32File.h"
#include <STM32SD.h>
#include <ff.h>  // For FA_OPEN_APPEND and other FatFs constants

namespace astra
{

/**
 * @brief IStorage implementation for eMMC using STM32 MMC interface
 *
 * Uses STM32SD library to access eMMC via MMC interface (4-bit mode).
 *
 * Pin configuration (hardcoded):
 * - PC8:  D0
 * - PC9:  D1
 * - PC10: D2
 * - PC11: D3
 * - PC12: CLK
 * - PD2:  CMD
 */
class EMMCBackend : public IStorage {
private:
    bool _initialized;

public:
    EMMCBackend() : _initialized(false) {}

    bool begin() override {
        // STM32SD.begin() initializes with default MMC pins
        if (!SD.begin()) {
            _initialized = false;
            return false;
        }
        _initialized = true;
        return true;
    }

    bool end() override {
        // STM32SD doesn't have end() method
        _initialized = false;
        return true;
    }

    bool ok() const override {
        return _initialized;
    }

    IFile* openRead(const char* filename) override {
        if (!_initialized) return nullptr;

        File file = SD.open(filename, FILE_READ);
        if (!file) return nullptr;

        return new STM32File(file);
    }

    IFile* openWrite(const char* filename, bool append = true) override {
        if (!_initialized) return nullptr;

        // STM32SD uses FatFs modes: FA_WRITE | FA_OPEN_APPEND for append
        uint8_t mode = append ? (FILE_WRITE | FA_OPEN_APPEND) : (FILE_WRITE | FA_OPEN_ALWAYS);
        File file = SD.open(filename, mode);
        if (!file) return nullptr;

        return new STM32File(file);
    }

    bool exists(const char* filename) override {
        if (!_initialized) return false;
        return SD.exists(filename);
    }

    bool remove(const char* filename) override {
        if (!_initialized) return false;
        return SD.remove(filename);
    }

    bool mkdir(const char* path) override {
        if (!_initialized) return false;
        return SD.mkdir(path);
    }

    bool rmdir(const char* path) override {
        if (!_initialized) return false;
        return SD.rmdir(path);
    }
};

} // namespace astra

#endif // STM32_EMMC_BACKEND_H
