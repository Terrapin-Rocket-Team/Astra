#ifndef LOGGER_H
#define LOGGER_H
#include "SerialCompat.h"
#include "Utils/CircBuffer.h"
#include "../../Storage/IFile.h"
#include "../../Storage/IStorage.h"
#include "../../Storage/StorageFactory.h"
namespace astra
{

    class ILogSink : public Print
    {
    public:
        virtual bool begin() = 0;
        virtual bool end() = 0;
        virtual bool ok() const = 0;
        virtual bool wantsPrefix() const = 0;
        virtual size_t write(uint8_t) override = 0;
        using Print::write;
        // flush() inherited from print, as are all print() and write() variants.
    };

    class UARTLog : public ILogSink
    {
    private:
        SerialUART_t &s;
        int baud;
        bool prefix;
        bool rdy;

    public:
        UARTLog(SerialUART_t &s, int baud, bool prefix = false) : s(s), baud(baud), prefix(prefix) {}
        bool begin() override
        {
            s.begin(baud);
            return rdy = true;
        }
        bool end() override
        {
            s.end();
            return true;
        }
        bool wantsPrefix() const override { return prefix; }
        bool ok() const override { return rdy; }
        size_t write(uint8_t b) override { return s.write(b); }
    };
    class USBLog : public ILogSink
    {
    private:
        SerialUSB_t &s;
        int baud;
        bool prefix;
        bool rdy;

    public:
        USBLog(SerialUSB_t &s, int baud, bool prefix = false) : s(s), baud(baud), prefix(prefix) {}
        bool begin() override
        {
            s.begin(baud);
            return rdy = true;
        }
        bool end() override
        {
            s.end();
            rdy = false;
            return true;
        }
        bool wantsPrefix() const override { return prefix; }
        bool ok() const override { return rdy; }
        size_t write(uint8_t b) override { return s.write(b); }
    };

    class PrintLog : public ILogSink
    {
    private:
        Print &p;
        bool prefix;
        bool rdy;

    public:
        PrintLog(Print &p, bool prefix = false) : p(p), prefix(prefix) {}
        bool begin() override
        {
            return rdy = true;
        }
        bool end() override
        {
            rdy = false;
            return true;
        }
        bool wantsPrefix() const override { return prefix; }
        bool ok() const override { return rdy; }
        size_t write(uint8_t b) override { return p.write(b); }
    };
    class CircBufferLog : public ILogSink
    {
    private:
        CircBuffer<uint8_t> *buf = nullptr;
        int size;
        bool prefix;
        bool rdy = false;

    public:
        CircBufferLog(int size, bool prefix = false) : size(size), prefix(prefix) {}
        bool begin() override
        {
            if (rdy)
                return true;
            buf = new CircBuffer<uint8_t>(size);
            if (buf)
                return rdy = true;
            return false;
        }
        bool end() override
        {
            rdy = false;
            delete buf;
            buf = nullptr;
            return true;
        }
        bool wantsPrefix() const override { return prefix; }
        bool ok() const override { return rdy; }
        size_t write(uint8_t b) override
        {
            if (rdy)
            {
                buf->push(b);
                return 1;
            }
            return 0;
        }
        bool transfer(ILogSink &other)
        {
            if (!rdy || buf->isEmpty())
                return false;
            while (!buf->isEmpty())
                other.write(buf->pop());
            return true;
        }
    };

    class FileLogSink : public ILogSink
    {
    private:
        IStorage* _backend;
        IFile* _file;
        const char* _filename;
        bool _ownsBackend;
        bool _prefix;

    public:
        /**
         * @brief Construct FileLogSink with automatic backend creation
         * @param filename Path to log file
         * @param type Storage backend type (EMMC, SD_CARD, etc.)
         * @param prefix Whether to add prefixes to log messages
         */
        FileLogSink(const char* filename, StorageBackend type, bool prefix = false);

        /**
         * @brief Construct FileLogSink with provided backend (advanced usage)
         * @param filename Path to log file
         * @param backend Pre-created storage backend (not owned by FileLogSink)
         * @param prefix Whether to add prefixes to log messages
         */
        FileLogSink(const char* filename, IStorage* backend, bool prefix = false);

        ~FileLogSink();

        bool begin() override;
        bool end() override;
        bool ok() const override;
        bool wantsPrefix() const override;
        size_t write(uint8_t b) override;
        size_t write(const uint8_t* buffer, size_t size) override;
        void flush() override;
    };

};

#endif // LOGGER_H