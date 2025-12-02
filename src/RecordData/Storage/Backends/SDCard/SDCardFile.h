#ifndef SDCARD_FILE_H
#define SDCARD_FILE_H

#include "../../IFile.h"

#if defined(ENV_ESP)
#include <cstdio>
#include <unistd.h>
#else
#include <SdFat.h>
#include <utility>
#endif

namespace astra
{

/**
 * @brief IFile implementation for SD card access
 *
 * ESP32: Uses FILE* with ESP32 SDMMC VFS FAT filesystem
 * STM32/Teensy: Uses SdFat's FsFile
 */
class SDCardFile : public IFile {
private:
#if defined(ENV_ESP)
    FILE* _handle;
    bool _isOpen;
#else
    FsFile _handle;
#endif

public:
#if defined(ENV_ESP)
    SDCardFile(FILE* handle) : _handle(handle), _isOpen(handle != nullptr) {}

    ~SDCardFile() {
        if (_isOpen && _handle) {
            close();
        }
    }
#else
    SDCardFile(FsFile&& handle) : _handle(std::move(handle)) {}
#endif

    // Writing
    size_t write(uint8_t b) override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return 0;
        return fwrite(&b, sizeof(uint8_t), 1, _handle);
#else
        return _handle.write(b);
#endif
    }

    size_t write(const uint8_t* buffer, size_t size) override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return 0;
        return fwrite(buffer, sizeof(uint8_t), size, _handle);
#else
        return _handle.write(buffer, size);
#endif
    }

    bool flush() override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return false;
        fflush(_handle);
        fsync(fileno(_handle));
        return true;
#else
        return _handle.sync();  // SdFat uses sync() instead of flush()
#endif
    }

    // Reading
    int read() override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return -1;
        return fgetc(_handle);
#else
        return _handle.read();
#endif
    }

    int readBytes(uint8_t* buffer, size_t length) override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return 0;
        return fread(buffer, 1, length, _handle);
#else
        return _handle.read(buffer, length);
#endif
    }

    int available() override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return 0;
        long pos = ftell(_handle);
        fseek(_handle, 0, SEEK_END);
        long size = ftell(_handle);
        fseek(_handle, pos, SEEK_SET);
        return size - pos;
#else
        return _handle.available();
#endif
    }

    // File operations
    bool seek(uint32_t pos) override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return false;
        return fseek(_handle, pos, SEEK_SET) == 0;
#else
        return _handle.seek(pos);
#endif
    }

    uint32_t position() override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return 0;
        return ftell(_handle);
#else
        return _handle.position();
#endif
    }

    uint32_t size() override {
#if defined(ENV_ESP)
        if (!_isOpen || !_handle) return 0;
        long pos = ftell(_handle);
        fseek(_handle, 0, SEEK_END);
        long size = ftell(_handle);
        fseek(_handle, pos, SEEK_SET);
        return size;
#else
        return _handle.size();
#endif
    }

    bool close() override {
#if defined(ENV_ESP)
        if (_isOpen && _handle) {
            fflush(_handle);
            fsync(fileno(_handle));
            fclose(_handle);
            _handle = nullptr;
            _isOpen = false;
        }
        return true;
#else
        _handle.close();
        return true;
#endif
    }

    // Status
    bool isOpen() const override {
#if defined(ENV_ESP)
        return _isOpen && _handle != nullptr;
#else
        return _handle.isOpen();
#endif
    }
};

} // namespace astra

#endif // SDCARD_FILE_H
