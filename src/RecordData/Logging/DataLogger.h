#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "LoggingBackend/ILogSink.h"
#include "../DataReporter/DataReporter.h"

namespace astra
{
    class DataLogger
    {
    private:
        static constexpr uint8_t MAX_REPORTERS = 32;

        ILogSink **_sinks = nullptr;
        uint8_t _countSinks = 0;
        uint8_t _countReporters = 0;
        DataReporter *_reporterRegistry[MAX_REPORTERS];
        bool _ok = false;
        static DataLogger _global;

    public:
        DataLogger();
        DataLogger(ILogSink **sinks, uint8_t numSinks);

        bool init();
        bool appendLine();
        void printHeaderTo(ILogSink *sink);
        DataReporter *const *getReporters() const { return _reporterRegistry; }
        uint8_t getNumReporters() const { return _countReporters; }

        static void configure(ILogSink **sinks, uint8_t numSinks);
        static bool registerReporter(DataReporter *reporter);
        static bool unregisterReporter(DataReporter *reporter);
        static DataLogger &instance();
        static bool available();
    };
}

#endif