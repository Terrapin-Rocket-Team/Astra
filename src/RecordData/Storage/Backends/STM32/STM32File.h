#ifndef STM32_FILE_H
#define STM32_FILE_H

#include "../../IFile.h"
#include <STM32SD.h>

namespace astra
{

/**
 * @brief IFile implementation wrapping STM32SD's File class
 *
 * This wrapper works with all STM32SD-based storage:
 * - eMMC chips via MMC/SDMMC interface
 * - SD cards via SDMMC interface
 *
 * The STM32SD library provides a unified File interface for both storage types.
 */
class STM32File : public IFile {
private:
    File _handle;

public:
    STM32File(File handle) : _handle(handle) {}

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

#endif // STM32_FILE_H
