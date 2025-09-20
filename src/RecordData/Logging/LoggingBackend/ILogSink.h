#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

namespace mmfs
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

    class SerialLog : public ILogSink
    {
    private:
        SerialClass &s;
        int baud;

    public:
        SerialLog(SerialClass &s, int baud) : s(s), baud(baud) {}
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