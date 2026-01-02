#include "DataLogger.h"

namespace astra
{
    DataLogger DataLogger::_global{nullptr, 0, nullptr, 0};

    DataLogger::DataLogger(ILogSink **sinks, uint8_t numSinks, DataReporter **reporters, uint8_t numReporters)
        : _sinks(sinks), _countSinks(numSinks), _countReporters(numReporters), _rps(reporters)
    {
    }

    void DataLogger::printHeaderTo(ILogSink *sink)
    {
        if (!sink || !sink->ok())
            return;

        if (sink->wantsPrefix() && _countReporters > 0 && _rps[0]->getNumColumns() > 0)
            sink->print("TELEM/");

        for (int j = 0; j < _countReporters; j++)
        {
            DataPoint *d = _rps[j]->getDataPoints();
            while (d != nullptr)
            {
                sink->printf("%s - %s", _rps[j]->getName(), d->label);
                if (d != _rps[j]->getLastPoint())
                    sink->write(',');
                d = d->next;
            }
            if (j != _countReporters - 1)
                sink->write(',');
            else
                sink->write('\n');
        }
        sink->flush();
    }

    bool DataLogger::init()
    {
        bool any = false;
        for (uint8_t i = 0; i < _countSinks; i++)
            if (_sinks[i]->begin())
            {
                any = true;
                printHeaderTo(_sinks[i]);
            }
        return _ok = any;
    }

    bool DataLogger::appendLine()
    {
        if (!_ok)
            return false;
        for (int i = 0; i < _countSinks; i++)
        {
            if (!_sinks[i]->ok())
                continue;
            if (_sinks[i]->wantsPrefix() && _countReporters > 0 && _rps[0]->getNumColumns() > 0)
                _sinks[i]->print("TELEM/");
            for (int j = 0; j < _countReporters; ++j)
            {
                for (DataPoint *d = _rps[j]->getDataPoints(); d != nullptr; d = d->next)
                {
                    d->emit(_sinks[i], d);
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
    }

    void DataLogger::configure(ILogSink **sinks, uint8_t numSinks, DataReporter **reporters, uint8_t numReporters)
    {
        _global = DataLogger(sinks, numSinks, reporters, numReporters);
        _global.init();
    }

    DataLogger &DataLogger::instance()
    {
        return _global;
    }

    bool DataLogger::available()
    {
        return _global._ok;
    }
}