#ifndef ESP32_FILE_H
#define ESP32_FILE_H

#include "../../IFile.h"
#include <cstdio>
#include <unistd.h>

namespace astra
{

/**
 * @brief IFile implementation for ESP32 using FILE* from VFS FAT filesystem
 *
 * Wraps ESP-IDF's FILE* handle from the VFS FAT filesystem layer.
 */
class ESP32File : public IFile {
private:
    FILE* _handle;
    bool _isOpen;

public:
    ESP32File(FILE* handle) : _handle(handle), _isOpen(handle != nullptr) {}

    ~ESP32File() {
        if (_isOpen && _handle) {
            close();
        }
    }

    // Writing
    size_t write(uint8_t b) override {
        if (!_isOpen || !_handle) return 0;
        return fwrite(&b, sizeof(uint8_t), 1, _handle);
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        if (!_isOpen || !_handle) return 0;
        return fwrite(buffer, sizeof(uint8_t), size, _handle);
    }

    bool flush() override {
        if (!_isOpen || !_handle) return false;
        fflush(_handle);
        fsync(fileno(_handle));
        return true;
    }

    // Reading
    int read() override {
        if (!_isOpen || !_handle) return -1;
        return fgetc(_handle);
    }

    int readBytes(uint8_t* buffer, size_t length) override {
        if (!_isOpen || !_handle) return 0;
        return fread(buffer, 1, length, _handle);
    }

    int available() override {
        if (!_isOpen || !_handle) return 0;
        long pos = ftell(_handle);
        fseek(_handle, 0, SEEK_END);
        long size = ftell(_handle);
        fseek(_handle, pos, SEEK_SET);
        return size - pos;
    }

    // File operations
    bool seek(uint32_t pos) override {
        if (!_isOpen || !_handle) return false;
        return fseek(_handle, pos, SEEK_SET) == 0;
    }

    uint32_t position() override {
        if (!_isOpen || !_handle) return 0;
        return ftell(_handle);
    }

    uint32_t size() override {
        if (!_isOpen || !_handle) return 0;
        long pos = ftell(_handle);
        fseek(_handle, 0, SEEK_END);
        long size = ftell(_handle);
        fseek(_handle, pos, SEEK_SET);
        return size;
    }

    bool close() override {
        if (_isOpen && _handle) {
            fflush(_handle);
            fsync(fileno(_handle));
            fclose(_handle);
            _handle = nullptr;
            _isOpen = false;
        }
        return true;
    }

    // Status
    bool isOpen() const override {
        return _isOpen && _handle != nullptr;
    }
};

} // namespace astra

#endif // ESP32_FILE_H
