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
    static DataLogger &globalLogger()
    {
        // Intentionally leaked to avoid static destruction order issues at shutdown.
        static DataLogger *logger = new DataLogger();
        return *logger;
    }

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

        bool hasAnyColumns = false;
        for (int j = 0; j < _countReporters; j++)
        {
            if (_reporterRegistry[j] && _reporterRegistry[j]->getNumColumns() > 0)
            {
                hasAnyColumns = true;
                break;
            }
        }

        if (sink->wantsPrefix() && hasAnyColumns)
            sink->print("TELEM/");

        auto hasNextReporterWithColumns = [&](int fromIdx) -> bool
        {
            for (int k = fromIdx + 1; k < _countReporters; k++)
            {
                if (_reporterRegistry[k] && _reporterRegistry[k]->getNumColumns() > 0)
                    return true;
            }
            return false;
        };

        for (int j = 0; j < _countReporters; j++)
        {
            if (!_reporterRegistry[j] || _reporterRegistry[j]->getNumColumns() <= 0)
                continue;

            DataPoint *d = _reporterRegistry[j]->getDataPoints();
            while (d != nullptr)
            {
                sink->printf("%s - %s", _reporterRegistry[j]->getName(), d->label);
                if (d != _reporterRegistry[j]->getLastPoint())
                    sink->write(',');
                d = d->next;
            }
            if (hasNextReporterWithColumns(j))
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
        bool hasAnyColumns = false;
        for (int j = 0; j < _countReporters; j++)
        {
            if (_reporterRegistry[j] && _reporterRegistry[j]->getNumColumns() > 0)
            {
                hasAnyColumns = true;
                break;
            }
        }

        if (hasAnyColumns)
        {
            stdoutPrint.print("TELEM/");
        }

        auto hasNextReporterWithColumns = [&](int fromIdx) -> bool
        {
            for (int k = fromIdx + 1; k < _countReporters; k++)
            {
                if (_reporterRegistry[k] && _reporterRegistry[k]->getNumColumns() > 0)
                    return true;
            }
            return false;
        };

        for (int j = 0; j < _countReporters; ++j)
        {
            if (!_reporterRegistry[j] || _reporterRegistry[j]->getNumColumns() <= 0)
                continue;

            for (DataPoint *d = _reporterRegistry[j]->getDataPoints(); d != nullptr; d = d->next)
            {
                d->emit(&stdoutPrint, d);
                if (d != _reporterRegistry[j]->getLastPoint())
                    stdoutPrint.write(',');
            }
            if (hasNextReporterWithColumns(j))
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

            bool hasAnyColumns = false;
            for (int j = 0; j < _countReporters; j++)
            {
                if (_reporterRegistry[j] && _reporterRegistry[j]->getNumColumns() > 0)
                {
                    hasAnyColumns = true;
                    break;
                }
            }
            if (_sinks[i]->wantsPrefix() && hasAnyColumns)
                _sinks[i]->print("TELEM/");

            auto hasNextReporterWithColumns = [&](int fromIdx) -> bool
            {
                for (int k = fromIdx + 1; k < _countReporters; k++)
                {
                    if (_reporterRegistry[k] && _reporterRegistry[k]->getNumColumns() > 0)
                        return true;
                }
                return false;
            };

            for (int j = 0; j < _countReporters; ++j)
            {
                if (!_reporterRegistry[j] || _reporterRegistry[j]->getNumColumns() <= 0)
                    continue;

                for (DataPoint *d = _reporterRegistry[j]->getDataPoints(); d != nullptr; d = d->next)
                {
                    d->emit(_sinks[i], d);
                    if (d != _reporterRegistry[j]->getLastPoint())
                        _sinks[i]->write(',');
                }
                if (hasNextReporterWithColumns(j))
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
        auto &logger = globalLogger();
        logger._sinks = sinks;
        logger._countSinks = numSinks;
        logger.init();
    }

    bool DataLogger::registerReporter(DataReporter *reporter)
    {
        if (!reporter)
            return false;

        auto &logger = globalLogger();

        if (logger._countReporters >= MAX_REPORTERS)
            return false;

        for (uint8_t i = 0; i < logger._countReporters; i++)
        {
            if (logger._reporterRegistry[i] == reporter)
                return false;
        }

        logger._reporterRegistry[logger._countReporters++] = reporter;
        return true;
    }

    bool DataLogger::unregisterReporter(DataReporter *reporter)
    {
        if (!reporter)
            return false;

        auto &logger = globalLogger();

        for (uint8_t i = 0; i < logger._countReporters; i++)
        {
            if (logger._reporterRegistry[i] == reporter)
            {
                for (uint8_t j = i; j < logger._countReporters - 1; j++)
                {
                    logger._reporterRegistry[j] = logger._reporterRegistry[j + 1];
                }
                logger._countReporters--;
                logger._reporterRegistry[logger._countReporters] = nullptr;
                return true;
            }
        }
        return false;
    }

    DataLogger &DataLogger::instance()
    {
        return globalLogger();
    }

    bool DataLogger::available()
    {
        return globalLogger()._ok;
    }

    void DataLogger::reset()
    {
        auto &logger = globalLogger();
        logger._sinks = nullptr;
        logger._countSinks = 0;
        logger._countReporters = 0;
        for (uint8_t i = 0; i < MAX_REPORTERS; i++)
        {
            logger._reporterRegistry[i] = nullptr;
        }
        logger._ok = false;
    }
}
