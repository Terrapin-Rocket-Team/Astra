#ifndef SDCARD_FILE_H
#define SDCARD_FILE_H

#include "../../IFile.h"
#include <SdFat.h>

namespace astra
{

/**
 * @brief IFile implementation wrapping SdFat's FsFile
 *
 * Works on all platforms (STM32, ESP32, Teensy) that support SdFat.
 * Can be used for both SPI and SDIO SD card access.
 */
class SDCardFile : public IFile {
private:
    FsFile _handle;

public:
    SDCardFile(FsFile handle) : _handle(handle) {}

    // Writing
    size_t write(uint8_t b) override {
        return _handle.write(b);
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        return _handle.write(buffer, size);
    }

    bool flush() override {
        return _handle.sync();  // SdFat uses sync() instead of flush()
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
        return _handle.isOpen();
    }
};

} // namespace astra

#endif // SDCARD_FILE_H
