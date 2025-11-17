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
        bool _ok = false;
        DataReporter **_rps = nullptr;       
        static DataLogger _global; // defined in .cpp

    public:
        DataLogger(ILogSink **sinks, uint8_t numSinks, DataReporter **reporters, uint8_t numReporters)
            : _countSinks(numSinks), _countReporters(numReporters)
        { //arrays copied internally 
            if (numSinks > 0 && sinks != nullptr)
            {
                _sinks = new ILogSink *[numSinks];
                for (uint8_t i = 0; i < numSinks; ++i)
                    _sinks[i] = sinks[i];
            }
            if (numReporters > 0 && reporters != nullptr)
            {
                _rps = new DataReporter *[numReporters];
                for (uint8_t j = 0; j < numReporters; ++j)
                    _rps[j] = reporters[j];
            }
        };

        bool init()
        {
            bool any = false;
            for (uint8_t i = 0; i < _countSinks; i++)
                if (_sinks[i]->begin())
                {
                    any = true;
                    if(_sinks[i]->wantsPrefix() && _countReporters > 0 && _rps[0]->getNumColumns() > 0)
                        _sinks[i]->print("TELEM/");

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
                    _sinks[i]->flush();
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
                _sinks[i]->flush();
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