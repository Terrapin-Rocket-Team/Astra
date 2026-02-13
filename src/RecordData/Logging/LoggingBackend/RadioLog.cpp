#include "RadioLog.h"
#include <cstring>

namespace astra
{
    RadioLog::RadioLog(SerialUART_t &s) : UARTLog(s, 115200, true)
    {
    }

    bool RadioLog::begin()
    {
        if (rdy)
            return true;

        UARTLog::begin();
        s.println("RAD/PING");
        delay(100);

        char buf[150];
        const uint64_t timeoutStart = millis();
        while ((millis() - timeoutStart) < 1000)
        {
            while (s.available())
            {
                int n = s.readBytesUntil('\n', buf, sizeof(buf) - 1);
                if (n <= 0)
                    continue;

                buf[n] = '\0';
                if (!strncmp("RAD/PONG", buf, 8))
                {
                    rdy = true;
                    return true;
                }
            }

            s.println("RAD/PING");
            delay(100);
        }

        rdy = false;
        return false;
    }
}
