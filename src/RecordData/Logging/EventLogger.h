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
        static EventLogger _global;

    public:
        EventLogger(ILogSink **sinks, uint8_t count, int maxMsgLen = 500);

        bool init();
        bool info(const char *fmt, ...);
        bool warn(const char *fmt, ...);
        bool err(const char *fmt, ...);

        static void configure(ILogSink **sinks, uint8_t count);
        static EventLogger &instance();
        static bool available();

    private:
        int min(int a, int b);
        bool vrecord(const char *lvl, const char *fmt, va_list ap);
    };
}
#define LOGI(...) ::astra::EventLogger::instance().info(__VA_ARGS__)
#define LOGW(...) ::astra::EventLogger::instance().warn(__VA_ARGS__)
#define LOGE(...) ::astra::EventLogger::instance().err(__VA_ARGS__)
#endif