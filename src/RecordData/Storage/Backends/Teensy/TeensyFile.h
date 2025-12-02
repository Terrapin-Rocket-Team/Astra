#ifndef TEENSY_FILE_H
#define TEENSY_FILE_H

#include "../../IFile.h"
#include <SdFat.h>
#include <utility>

namespace astra
{

/**
 * @brief IFile implementation for Teensy using SdFat's FsFile
 *
 * Wraps SdFat library's FsFile handle for SD card access via SDIO.
 */
class TeensyFile : public IFile {
private:
    FsFile _handle;

public:
    TeensyFile(FsFile&& handle) : _handle(std::move(handle)) {}

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

#endif // TEENSY_FILE_H
