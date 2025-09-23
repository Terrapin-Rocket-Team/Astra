#ifndef LOGGER_H
#define LOGGER_H
#include "SerialCompat.h"
namespace astra
{
    class State;        // Forward declaration
    class DataReporter; // Forward declaration
    class GPS;

    class ILogSink : public Print
    {

    public:
        virtual bool begin() = 0;
        virtual bool end() = 0;
        virtual bool ok() const = 0;
        virtual size_t write(uint8_t) override = 0;
        using Print::write;
        // flush() inherited from print, as are all print() and write() variants.
    };

    class UARTLog : public ILogSink
    {
    private:
        SerialUART_t &s;
        int baud;

    public:
        UARTLog(SerialUART_t &s, int baud) : s(s), baud(baud) {}
        bool begin() override
        {
            s.begin(baud);
            return true;
        }
        bool end() override
        {
            s.end();
            return true;
        }
        bool ok() const override { return s; }
        size_t write(uint8_t b) override { return s.write(b); }
    };
    class USBLog : public ILogSink
    {
    private:
        SerialUSB_t &s;
        int baud;

    public:
        USBLog(SerialUSB_t &s, int baud) : s(s), baud(baud) {}
        bool begin() override
        {
            s.begin(baud);
            return true;
        }
        bool end() override
        {
            s.end();
            return true;
        }
        bool ok() const override { return s; }
        size_t write(uint8_t b) override { return s.write(b); }
    };

    class PrintLog : public ILogSink
    {
    private:
        Print &p;

    public:
        PrintLog(Print &p) : p(p) {}
        bool begin() override
        {
            return true;
        }
        bool end() override
        {
            return true;
        }
        bool ok() const override { return true; }
        size_t write(uint8_t b) override { return p.write(b); }
    };
};

#endif // LOGGER_H