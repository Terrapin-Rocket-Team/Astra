#ifndef Astra_SYSTEM_H
#define Astra_SYSTEM_H

#include "AstraConfig.h"

namespace astra
{
    class Astra
    {
    public:
        Astra(AstraConfig *config);
        void init();
        bool update(double ms = -1); // returns true if state update occurred

    private:
        bool ready = false;
        AstraConfig *config = nullptr;
        double lastStateUpdate = 0, lastLoggingUpdate = 0;
    };
}
#endif