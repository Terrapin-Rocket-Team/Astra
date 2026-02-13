#ifndef RADIO_LOG_H
#define RADIO_LOG_H

#include "ILogSink.h"

namespace astra
{
    class RadioLog : public UARTLog
    {
    public:
        explicit RadioLog(SerialUART_t &s);
        bool begin() override;
    };
}

#endif // RADIO_LOG_H
