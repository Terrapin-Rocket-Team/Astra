#ifndef EMMC_FILE_H
#define EMMC_FILE_H

#if defined(ENV_STM)

#include "../../IFile.h"
#include <STM32SD.h>

namespace astra
{

/**
 * @brief IFile implementation wrapping STM32SD's File class
 *
 * Supported platforms: STM32 only
 * Works with eMMC and SD cards connected via MMC interface.
 */
class EMMCFile : public IFile {
private:
    File _handle;

public:
    EMMCFile(File handle) : _handle(handle) {}

    // Writing
    size_t write(uint8_t b) override {
        return _handle.write(b);
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        return _handle.write(buffer, size);
    }

    bool flush() override {
        _handle.flush();
        return true;
    }

    // Reading
    int read() override {
        return _handle.read();
    }

    int readBytes(uint8_t* buffer, size_t length) override {
        return _handle.read(buffer, length);
    }

    int available() override {
        return _handle.available();
    }

    // File operations
    bool seek(uint32_t pos) override {
        return _handle.seek(pos);
    }

    uint32_t position() override {
        return _handle.position();
    }

    uint32_t size() override {
        return _handle.size();
    }

    bool close() override {
        _handle.close();
        return true;
    }

    // Status
    bool isOpen() const override {
        // Cast away const since STM32SD's File::operator bool() is not const
        return const_cast<File&>(_handle) ? true : false;
    }
};

} // namespace astra

#endif // ENV_STM

#endif // EMMC_FILE_H
