#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "LoggingBackend/ILogSink.h"
#include "../DataReporter/DataReporter.h"

namespace astra
{
    class DataLogger
    {
    private:
        ILogSink **_sinks = nullptr;
        uint8_t _countSinks, _countReporters = 0;
        DataReporter **_rps = nullptr;
        bool _ok = false;
        static DataLogger _global;

    public:
        DataLogger(ILogSink **sinks, uint8_t numSinks, DataReporter **reporters, uint8_t numReporters);

        bool init();
        bool appendLine();

        static void configure(ILogSink **sinks, uint8_t numSinks, DataReporter **reporters, uint8_t numReporters);
        static DataLogger &instance();
        static bool available();
    };
}

#endif