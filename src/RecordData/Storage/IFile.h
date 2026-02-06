#ifndef IFILE_H
#define IFILE_H

#include <cstddef>
#include <cstdint>

namespace astra
{

    class IFile
    {
    public:
        virtual ~IFile() {}

        // Writing
        virtual size_t write(uint8_t b) = 0;
        virtual size_t write(const uint8_t *buffer, size_t size) = 0;
        virtual bool flush() = 0;

        // Reading
        virtual int read() = 0;
        virtual int readBytes(uint8_t *buffer, size_t length) = 0;
        virtual int available() = 0;

        // File operations
        virtual bool seek(uint32_t pos) = 0;
        virtual uint32_t position() = 0;
        virtual uint32_t size() = 0;
        virtual bool close() = 0;

        // Status
        virtual bool isOpen() const = 0;
        virtual operator bool() const { return isOpen(); }
    };
}
#endif