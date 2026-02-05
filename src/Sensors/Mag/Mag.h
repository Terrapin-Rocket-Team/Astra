#ifndef MAG_H
#define MAG_H

#include "Sensors/RotatableSensor.h"
#include "../../Math/Vector.h"

namespace astra
{
    class Mag : public RotatableSensor
    {
    public:
        virtual ~Mag();
        virtual Vector<3> getMag() const;

        // Override update to add health tracking
        int update(double currentTime = -1) override
        {
            if (!initialized)
                return -1;

            int err = read();

            if (err != 0)
            {
                healthy = false;
                consecutiveGoodReads = 0;
                return err;
            }

            updateHealthTracking();
            return 0;
        }

    protected:
        Mag(const char *name = "Magnetometer");
        Vector<3> mag = Vector<3>(0, 0, 0);

        // Health tracking for stuck-reading detection
        static constexpr uint8_t HEALTH_BUFFER_SIZE = 3;
        Vector<3> lastReadings[HEALTH_BUFFER_SIZE];
        uint8_t readingIndex = 0;
        uint8_t consecutiveGoodReads = 0;

        void updateHealthTracking()
        {
            lastReadings[readingIndex] = mag;
            readingIndex = (readingIndex + 1) % HEALTH_BUFFER_SIZE;

            if (consecutiveGoodReads < HEALTH_BUFFER_SIZE - 1)
            {
                consecutiveGoodReads++;
                return;
            }

            bool allIdentical = true;
            for (uint8_t i = 1; i < HEALTH_BUFFER_SIZE; i++)
            {
                if (lastReadings[i].x() != lastReadings[0].x() ||
                    lastReadings[i].y() != lastReadings[0].y() ||
                    lastReadings[i].z() != lastReadings[0].z())
                {
                    allIdentical = false;
                    break;
                }
            }

            if (allIdentical)
            {
                if (healthy)
                    LOGW("Mag '%s' became unhealthy: stuck readings detected", getName());
                healthy = false;
                consecutiveGoodReads = 0;
            }
            else
            {
                if (!healthy)
                    LOGI("Mag '%s' recovered: readings varying normally", getName());
                healthy = true;
            }
        }
    };
}
#endif // MAG_H
