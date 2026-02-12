#ifndef ACCEL_H
#define ACCEL_H

#include "Sensors/RotatableSensor.h"
#include "../../Math/Vector.h"

namespace astra
{
    class Accel : public RotatableSensor
    {
    public:
        virtual ~Accel();
        virtual Vector<3> getAccel() const;

        // Override update to add health tracking
        int update(double currentTime = -1) override
        {
            if (!initialized)
                return -1;  // Don't attempt read if not initialized

            int err = read();  // Calls derived hardware class read()

            if (err != 0)
            {
                healthy = false;  // Immediate invalidation on read failure
                consecutiveGoodReads = 0;
                return err;
            }

            // Check for stuck readings after successful read
            updateHealth();
            return 0;
        }

    protected:
        Accel(const char *name = "Accelerometer");
        Vector<3> acc = Vector<3>(0, 0, 0);

        // Health tracking for stuck-reading detection
        static constexpr uint8_t HEALTH_BUFFER_SIZE = 3;
        CircBuffer<Vector<3>> lastReadings;
        uint8_t consecutiveGoodReads = 0;

        virtual void updateHealth() override
        {
            // Store current reading in circular buffer
            lastReadings.push(acc);

            // Only check for stuck readings after buffer is full
            if (consecutiveGoodReads < HEALTH_BUFFER_SIZE - 1)
            {
                consecutiveGoodReads++;
                return;  // Still filling buffer
            }

            // Check if all readings are identical (stuck sensor)
            // Compare each component of the vectors
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
                if (healthy)  // Only log on transition
                    LOGW("Accel '%s' became unhealthy: stuck readings detected", getName());
                healthy = false;
                consecutiveGoodReads = 0;
                lastReadings.clear();
            }
            else
            {
                // Good readings - sensor is healthy
                if (!healthy)  // Only log on transition
                    LOGI("Accel '%s' recovered: readings varying normally", getName());
                healthy = true;
            }
        }
    };
}
#endif // ACCEL_H
