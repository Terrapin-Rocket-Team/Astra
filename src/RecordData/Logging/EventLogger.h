#ifndef INFO_LOGGER_H
#define INFO_LOGGER_H

#include "LoggingBackend/ILogSink.h"

namespace astra
{
    class EventLogger
    {
    private:
        ILogSink **_sinks = nullptr;
        uint8_t _count = 0;
        bool _ok = false;
        int _maxMsgLen = 0;
        static EventLogger _global; // defined in .cpp

    public:
        EventLogger(ILogSink **sinks, uint8_t count, int maxMsgLen = 500) : _sinks(sinks), _count(count), _maxMsgLen(maxMsgLen) {};

        bool init()
        {
            bool any = false;
            for (uint8_t i = 0; i < _count; i++)
                any |= _sinks[i]->begin();
            return _ok = any;
        }
        bool info(const char *fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            const bool rc = vrecord("INFO", fmt, ap);
            va_end(ap);
            return rc;
        }
        bool warn(const char *fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            const bool rc = vrecord("WARNING", fmt, ap);
            va_end(ap);
            return rc;
        }
        bool err(const char *fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            const bool rc = vrecord("ERROR", fmt, ap);
            va_end(ap);
            return rc;
        }

        // Global accessor pattern
        static void configure(ILogSink **sinks, uint8_t count)
        {
            _global = EventLogger(sinks, count);
            _global.init();
        }
        static EventLogger &instance() { return _global; }
        static bool available() { return _global._ok; }

    private:
        int min(int a, int b){
            return a < b ? a : b;
        }

        bool vrecord(const char *lvl, const char *fmt, va_list ap)
        {
            if (!_ok)
                return false;
            char *msg = new char[_maxMsgLen];
            const int n = vsnprintf(msg, _maxMsgLen, fmt, ap);
            if (n <= 0)
                return false;

            char pre[32];
            const int m = snprintf(pre, sizeof(pre), "%.3f [%s]: ", millis() / 1000.0, lvl);

            bool wroteAny = false;
            for (uint8_t i = 0; i < _count; i++)
            {
                ILogSink *s = _sinks[i];
                if (!s->ok())
                    continue;
                s->write((const uint8_t *)pre, (size_t)m);
                s->write((const uint8_t *)msg, (size_t)min(n, (int)(_maxMsgLen - 1)));
                s->write((const uint8_t *)"\n", 1);
                s->flush();
                wroteAny = true;
            }
            return wroteAny;
        }
    };
}
#define LOGI(...) ::astra::EventLogger::instance().info(__VA_ARGS__)
#define LOGW(...) ::astra::EventLogger::instance().warn(__VA_ARGS__)
#define LOGE(...) ::astra::EventLogger::instance().err(__VA_ARGS__)
#endif