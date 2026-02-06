#include "DataLogger.h"

#ifdef NATIVE
#include <stdio.h>

// Simple Print wrapper for stdout on native builds
class StdoutPrint : public Print
{
public:
    size_t write(uint8_t c) override
    {
        return fputc(c, stdout) != EOF ? 1 : 0;
    }

    size_t write(const uint8_t *buffer, size_t size) override
    {
        return fwrite(buffer, 1, size, stdout);
    }
};

static StdoutPrint stdoutPrint;
#endif

namespace astra
{
    DataLogger DataLogger::_global{};

    DataLogger::DataLogger()
        : _sinks(nullptr), _countSinks(0), _countReporters(0), _reporterRegistry{}, _ok(false)
    {
    }

    DataLogger::DataLogger(ILogSink **sinks, uint8_t numSinks)
        : _sinks(sinks), _countSinks(numSinks), _countReporters(0), _reporterRegistry{}, _ok(false)
    {
    }

    void DataLogger::printHeaderTo(ILogSink *sink)
    {
        if (!sink || !sink->ok())
            return;

        if (sink->wantsPrefix() && _countReporters > 0 && _reporterRegistry[0]->getNumColumns() > 0)
            sink->print("TELEM/");

        for (int j = 0; j < _countReporters; j++)
        {
            DataPoint *d = _reporterRegistry[j]->getDataPoints();
            while (d != nullptr)
            {
                sink->printf("%s - %s", _reporterRegistry[j]->getName(), d->label);
                if (d != _reporterRegistry[j]->getLastPoint())
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

#ifdef NATIVE
        // On native builds, also output to stdout
        #ifndef NATIVE_NO_STDOUT_DATA
        if (_countReporters > 0 && _reporterRegistry[0]->getNumColumns() > 0)
        {
            stdoutPrint.print("TELEM/");
        }
        for (int j = 0; j < _countReporters; ++j)
        {
            for (DataPoint *d = _reporterRegistry[j]->getDataPoints(); d != nullptr; d = d->next)
            {
                d->emit(&stdoutPrint, d);
                if (d != _reporterRegistry[j]->getLastPoint())
                    stdoutPrint.write(',');
            }
            if (j != _countReporters - 1)
                stdoutPrint.write(',');
            else
                stdoutPrint.write('\n');
        }
        fflush(stdout);
        #endif
#endif

        for (int i = 0; i < _countSinks; i++)
        {
            if (!_sinks[i]->ok())
                continue;
            if (_sinks[i]->wantsPrefix() && _countReporters > 0 && _reporterRegistry[0]->getNumColumns() > 0)
                _sinks[i]->print("TELEM/");
            for (int j = 0; j < _countReporters; ++j)
            {
                for (DataPoint *d = _reporterRegistry[j]->getDataPoints(); d != nullptr; d = d->next)
                {
                    d->emit(_sinks[i], d);
                    if (d != _reporterRegistry[j]->getLastPoint())
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

    void DataLogger::configure(ILogSink **sinks, uint8_t numSinks)
    {
        _global._sinks = sinks;
        _global._countSinks = numSinks;
        _global.init();
    }

    bool DataLogger::registerReporter(DataReporter *reporter)
    {
        if (!reporter)
            return false;

        if (_global._countReporters >= MAX_REPORTERS)
            return false;

        for (uint8_t i = 0; i < _global._countReporters; i++)
        {
            if (_global._reporterRegistry[i] == reporter)
                return false;
        }

        _global._reporterRegistry[_global._countReporters++] = reporter;
        return true;
    }

    bool DataLogger::unregisterReporter(DataReporter *reporter)
    {
        if (!reporter)
            return false;

        for (uint8_t i = 0; i < _global._countReporters; i++)
        {
            if (_global._reporterRegistry[i] == reporter)
            {
                for (uint8_t j = i; j < _global._countReporters - 1; j++)
                {
                    _global._reporterRegistry[j] = _global._reporterRegistry[j + 1];
                }
                _global._countReporters--;
                _global._reporterRegistry[_global._countReporters] = nullptr;
                return true;
            }
        }
        return false;
    }

    DataLogger &DataLogger::instance()
    {
        return _global;
    }

    bool DataLogger::available()
    {
        return _global._ok;
    }

    void DataLogger::reset()
    {
        _global._sinks = nullptr;
        _global._countSinks = 0;
        _global._countReporters = 0;
        for (uint8_t i = 0; i < MAX_REPORTERS; i++)
        {
            _global._reporterRegistry[i] = nullptr;
        }
        _global._ok = false;
    }
}