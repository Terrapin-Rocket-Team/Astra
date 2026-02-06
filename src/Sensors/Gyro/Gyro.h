#ifndef GYRO_H
#define GYRO_H

#include "Sensors/RotatableSensor.h"
#include "../../Math/Vector.h"

namespace astra
{
    class Gyro : public RotatableSensor
    {
    public:
        virtual ~Gyro();
        virtual Vector<3> getAngVel() const;

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
        Gyro(const char *name = "Gyroscope");
        Vector<3> angVel = Vector<3>(0, 0, 0);

        // Health tracking for stuck-reading detection
        static constexpr uint8_t HEALTH_BUFFER_SIZE = 3;
        CircBuffer<Vector<3>> lastReadings;
        uint8_t consecutiveGoodReads = 0;

        void updateHealthTracking()
        {
            lastReadings.push(angVel);

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
                    LOGW("Gyro '%s' became unhealthy: stuck readings detected", getName());
                healthy = false;
                consecutiveGoodReads = 0;
                lastReadings.clear();
            }
            else
            {
                if (!healthy)
                    LOGI("Gyro '%s' recovered: readings varying normally", getName());
                healthy = true;
            }
        }
    };
}
#endif // GYRO_H
