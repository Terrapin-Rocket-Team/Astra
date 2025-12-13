#include "ILogSink.h"
#include <cstring>
#include <cstdio>

namespace astra
{

// Helper function to find next available filename
static char* findNextFilename(IStorage* backend, const char* baseFilename) {
    if (!backend) return nullptr;

    // Find the extension (if any)
    const char* dot = strrchr(baseFilename, '.');
    char baseName[64];
    char extension[16];

    if (dot) {
        // Split into base and extension
        size_t baseLen = dot - baseFilename;
        if (baseLen >= sizeof(baseName)) baseLen = sizeof(baseName) - 1;
        strncpy(baseName, baseFilename, baseLen);
        baseName[baseLen] = '\0';

        strncpy(extension, dot, sizeof(extension) - 1);
        extension[sizeof(extension) - 1] = '\0';
    } else {
        // No extension
        strncpy(baseName, baseFilename, sizeof(baseName) - 1);
        baseName[sizeof(baseName) - 1] = '\0';
        extension[0] = '\0';
    }

    // Try filenames with incrementing numbers
    static char filename[80];
    for (int i = 0; i < 1000; i++) {
        if (i == 0) {
            // Try the original filename first
            snprintf(filename, sizeof(filename), "%s%s", baseName, extension);
        } else {
            // Try with number suffix
            snprintf(filename, sizeof(filename), "%s_%d%s", baseName, i, extension);
        }

        if (!backend->exists(filename)) {
            return filename;
        }
    }

    // If we couldn't find an available filename after 1000 attempts, return nullptr
    return nullptr;
}

// Constructor with automatic backend creation
FileLogSink::FileLogSink(const char* filename, StorageBackend type, bool prefix)
    : _filename(filename), _file(nullptr), _ownsBackend(true), _prefix(prefix) {

    _backend = StorageFactory::create(type);
}

// Constructor with provided backend
FileLogSink::FileLogSink(const char* filename, IStorage* backend, bool prefix)
    : _filename(filename), _backend(backend), _file(nullptr), _ownsBackend(false), _prefix(prefix) {
}

FileLogSink::~FileLogSink() {
    if (_file) {
        _file->close();
        delete _file;
        _file = nullptr;
    }

    if (_ownsBackend && _backend) {
        _backend->end();
        delete _backend;
        _backend = nullptr;
    }
}

bool FileLogSink::begin() {
    if (!_backend) return false;

    if (!_backend->begin()) {
        return false;
    }

    // Find the next available filename by iterating
    char* availableFilename = findNextFilename(_backend, _filename);
    if (!availableFilename) {
        return false;  // Couldn't find an available filename
    }

    // Open in write mode (not append) to create a new file
    _file = _backend->openWrite(availableFilename, false);
    return _file && _file->isOpen();
}

bool FileLogSink::end() {
    if (_file) {
        _file->flush();
        _file->close();
        delete _file;
        _file = nullptr;
    }

    if (_ownsBackend && _backend) {
        _backend->end();
    }

    return true;
}

bool FileLogSink::ok() const {
    return _backend && _backend->ok() && _file && _file->isOpen();
}

bool FileLogSink::wantsPrefix() const {
    return _prefix;
}

size_t FileLogSink::write(uint8_t b) {
    if (!_file || !_file->isOpen()) return 0;
    return _file->write(b);
}

size_t FileLogSink::write(const uint8_t* buffer, size_t size) {
    if (!_file || !_file->isOpen()) return 0;
    return _file->write(buffer, size);
}

void FileLogSink::flush() {
    if (_file && _file->isOpen()) {
        _file->flush();
    }
}

} // namespace astra
