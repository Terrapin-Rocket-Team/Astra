#include "MockStorage.h"
#include "RecordData/Storage/StorageFactory.h"
#include <cstdio>
#include <iostream>

// MockFile implementation
MockFile::MockFile(const char* filename, const char* mode) {
    _file = fopen(filename, mode);
}

MockFile::~MockFile() {
    if (_file) {
        fclose(_file);
    }
}

size_t MockFile::write(uint8_t b) {
    if (!_file) return 0;
    return fwrite(&b, 1, 1, _file);
}

size_t MockFile::write(const uint8_t *buffer, size_t size) {
    if (!_file) return 0;
    return fwrite(buffer, 1, size, _file);
}

bool MockFile::flush() {
    if (!_file) return false;
    return fflush(_file) == 0;
}

int MockFile::read() {
    if (!_file) return -1;
    int c = fgetc(_file);
    return c;
}

int MockFile::readBytes(uint8_t *buffer, size_t length) {
    if (!_file) return 0;
    return fread(buffer, 1, length, _file);
}

int MockFile::available() {
    if (!_file) return 0;
    long current = ftell(_file);
    fseek(_file, 0, SEEK_END);
    long end = ftell(_file);
    fseek(_file, current, SEEK_SET);
    return end - current;
}

bool MockFile::seek(uint32_t pos) {
    if (!_file) return false;
    return fseek(_file, pos, SEEK_SET) == 0;
}

uint32_t MockFile::position() {
    if (!_file) return 0;
    return ftell(_file);
}

uint32_t MockFile::size() {
    if (!_file) return 0;
    long current = ftell(_file);
    fseek(_file, 0, SEEK_END);
    uint32_t size = ftell(_file);
    fseek(_file, current, SEEK_SET);
    return size;
}

bool MockFile::close() {
    if (!_file) return false;
    bool closed = fclose(_file) == 0;
    _file = nullptr;
    return closed;
}

bool MockFile::isOpen() const {
    return _file != nullptr;
}


// MockStorage implementation
bool MockStorage::begin() { return true; }
bool MockStorage::end() { return true; }
bool MockStorage::ok() const { return true; }

astra::IFile* MockStorage::openRead(const char *filename) {
    return new MockFile(filename, "rb");
}

astra::IFile* MockStorage::openWrite(const char *filename, bool append) {
    return new MockFile(filename, append ? "ab" : "wb");
}

bool MockStorage::exists(const char *filename) {
    if (FILE *file = fopen(filename, "r")) {
        fclose(file);
        return true;
    }
    return false;
}

bool MockStorage::remove(const char *filename) {
    return ::remove(filename) == 0;
}

bool MockStorage::mkdir(const char *path) {
    // Not properly implemented for cross-platform, but this might work on some systems
    // For Windows, might need to use _mkdir or CreateDirectory
    // For now, just return true to not block tests
    std::cout << "Warning: MockStorage::mkdir is not implemented. Path: " << path << std::endl;
    return true;
}

bool MockStorage::rmdir(const char *path) {
    std::cout << "Warning: MockStorage::rmdir is not implemented. Path: " << path << std::endl;
    return true;
}

// Native StorageFactory implementation
namespace astra {
    IStorage *StorageFactory::create(StorageBackend type) {
        // In native test environment, always return a MockStorage instance
        // regardless of the type requested.
        std::cout << "Creating MockStorage" << std::endl;
        return new MockStorage();
    }
}
