#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "LoggingBackend/ILogSink.h"
#include "../DataReporter/DataReporter.h"

namespace mmfs
{
    class DataLogger
    {
    private:
        ILogSink **_sinks = nullptr;
        uint8_t _countSinks, _countReporters = 0;
        bool _ok = false;
        DataReporter **_rps;
        static DataLogger _global; // defined in .cpp

    public:
        DataLogger(ILogSink **sinks, uint8_t numSinks, DataReporter **reporters, uint8_t numReporters) : _sinks(sinks), _countSinks(numSinks), _rps(reporters), _countReporters(numReporters) {};

        bool init()
        {
            bool any = false;
            for (uint8_t i = 0; i < _countSinks; i++)
                if (_sinks[i]->begin())
                {
                    any = true;
                    for (int j = 0; j < _countReporters; j++)
                    {
                        DataPoint *d = _rps[j]->getDataPoints();
                        while (d != nullptr)
                        {
                            _sinks[i]->printf("%s - %s", _rps[j]->getName(), d->label);
                            if (d != _rps[j]->getLastPoint())
                                _sinks[i]->write(',');
                            d = d->next;
                        }
                        if (j != _countReporters - 1)
                            _sinks[i]->write(',');
                        else
                            _sinks[i]->write('\n');
                    }
                }
            return _ok = any;
        }
        bool appendLine()
        {
            if (!_ok)
                return false;
            for (int i = 0; i < _countSinks; i++)
            {
                if (!_sinks[i]->ok())
                    continue;
                for (int j = 0; j < _countReporters; ++j)
                {
                    for (DataPoint *d = _rps[j]->getDataPoints(); d != nullptr; d = d->next)
                    {
                        d->emit(_sinks[i], d); // <<<<<<<<<<<<<< use the stored printer
                        if (d != _rps[j]->getLastPoint())
                            _sinks[i]->write(',');
                    }
                    if (j != _countReporters - 1)
                        _sinks[i]->write(',');
                    else
                        _sinks[i]->write('\n');
                }
            }
            return true;
        };

        // Global accessor pattern
        static void configure(ILogSink **sinks, uint8_t numSinks, DataReporter **reporters, uint8_t numReporters)
        {
            _global = DataLogger(sinks, numSinks, reporters, numReporters);
            _global.init();
        }
        static DataLogger &instance() { return _global; }
        static bool available() { return _global._ok; }
    };
}

#endif