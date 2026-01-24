#include "EventLogger.h"

#ifdef NATIVE
#include <stdio.h>
#endif

namespace astra
{
    EventLogger EventLogger::_global{nullptr, 0};

    EventLogger::EventLogger(ILogSink **sinks, uint8_t count, int maxMsgLen)
        : _count(count), _maxMsgLen(maxMsgLen)
    {
        if (count > 0 && sinks != nullptr)
        {
            _sinks = new ILogSink *[count];
            for (uint8_t i = 0; i < count; ++i)
                _sinks[i] = sinks[i];
        }
    }

    bool EventLogger::init()
    {
        bool anySinksOk = false;
        for (uint8_t i = 0; i < _count; i++)
            anySinksOk |= _sinks[i]->begin();
        return _ok = anySinksOk;
    }

    bool EventLogger::info(const char *fmt, ...)
    {
#if defined(NATIVE) && defined(NOLOGI)
        return false;
#endif
        va_list ap;
        va_start(ap, fmt);
        const bool rc = vrecord("INFO", fmt, ap);
        va_end(ap);
        return rc;
    }

    bool EventLogger::warn(const char *fmt, ...)
    {
#if defined(NATIVE) && defined(NOLOGW)
        return false;
#endif
        va_list ap;
        va_start(ap, fmt);
        const bool rc = vrecord("WARNING", fmt, ap);
        va_end(ap);
        return rc;
    }

    bool EventLogger::err(const char *fmt, ...)
    {
#if defined(NATIVE) && defined(NOLOGE)
        return false;
#endif
        va_list ap;
        va_start(ap, fmt);
        const bool rc = vrecord("ERROR", fmt, ap);
        va_end(ap);
        return rc;
    }
    bool EventLogger::dbg(const char *fmt, ...)
    {
#ifdef DEBUG
        va_list ap;
        va_start(ap, fmt);
        const bool rc = vrecord("DEBUG", fmt, ap);
        va_end(ap);
        return rc;
#else
        return false;
#endif
    }

    void EventLogger::configure(ILogSink **sinks, uint8_t count)
    {
        _global = EventLogger(sinks, count);
        _global.init();
    }

    EventLogger &EventLogger::instance()
    {
        return _global;
    }

    bool EventLogger::available()
    {
        return _global._ok;
    }

    int EventLogger::min(int a, int b)
    {
        return a < b ? a : b;
    }

    bool EventLogger::vrecord(const char *lvl, const char *fmt, va_list ap)
    {
        if (!_ok)
            return false;

        const int kBufSize = 512;
        char msg[kBufSize];
        const int n = vsnprintf(msg, kBufSize, fmt, ap);
        if (n <= 0)
            return false;

        char pre[32];
        int m = snprintf(pre, sizeof(pre), "%.3f [%s]: ", millis() / 1000.0, lvl);

#ifdef NATIVE
        // On native builds, also output to stdout/stderr
        #ifndef NATIVE_NO_STDOUT_LOG
        FILE* stream = (strcmp(lvl, "ERROR") == 0) ? stderr : stdout;
        fprintf(stream, "%s%s\n", pre, msg);
        fflush(stream);
        #endif
#endif

        bool wroteAny = false;
        for (uint8_t i = 0; i < _count; i++)
        {
            ILogSink *s = _sinks[i];
            if (!s->ok())
                continue;
            if (s->wantsPrefix())
                s->print("LOG/");
            s->write((const uint8_t *)pre, (size_t)m);
            s->write((const uint8_t *)msg, (size_t)min(n, (int)(kBufSize - 1)));
            s->write((const uint8_t *)"\n", 1);
            s->flush();
            wroteAny = true;
        }
        return wroteAny;
    }
}