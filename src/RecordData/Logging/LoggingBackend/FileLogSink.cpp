#include "ILogSink.h"

namespace astra
{

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

    _file = _backend->openWrite(_filename, true);  // Open in append mode
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
