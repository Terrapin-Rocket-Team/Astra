#ifndef SENSOR_H
#define SENSOR_H

#include "../Utils/CircBuffer.h"
#include "../BlinkBuzz/BlinkBuzz.h"
#include <algorithm>
#include "../RecordData/DataReporter/DataReporter.h"
#include "Utils/Hash.h"
#include "RecordData/Logging/EventLogger.h"

namespace astra
{
    using SensorType = uint32_t;

    class Sensor : public DataReporter
    {
    public:
        virtual ~Sensor() {};

        void setUpdateRate(double hz) { updateInterval = 1.0 / hz; }

        // currentTime in S. returns if enough time has passed that more data should be ready.
        bool shouldUpdate(double currentTime)
        {
            if (currentTime - lastUpdateTime >= updateInterval)
            {
                lastUpdateTime = currentTime;
                return true;
            }
            return false;
        }

        // Check if sensor is healthy and data can be trusted
        // Returns false if init failed, read errors occurred, or stuck readings detected
        virtual bool isHealthy() const { return healthy; }

        // Returns 0 on success, library-specific error code on failure
        virtual int begin() override
        {
            int err = init();
            initialized = (err == 0);
            healthy = initialized;  // Healthy if init succeeded
            return err;
        }

        // Returns 0 on success, library-specific error code on failure
        virtual int update(double currentTime = -1) override
        {
            return read();
        }
    protected:
        // --------------------------------- HARDWARE IMPLEMENTATION -----------------------------------------------

        Sensor(const char *name = nullptr) : DataReporter(name)
        {
            autoUpdate = false; //stop the logging system from automatically udpating sensors before logging data
        }

        // Sets up the sensor and stores any critical parameters. Needs to reset the sensor if it is already initialized.
        // Called by begin(). Returns 0 on success, library-specific error code on failure.
        virtual int init() = 0;

        // Physically reads the outputs from the sensor hardware. Called by update()
        // Returns 0 on success, library-specific error code on failure.
        virtual int read() = 0;

        // Health tracking state
        bool initialized = false;  // Did begin() succeed? (internal, never changes)
        bool healthy = false;      // Can we trust the data? (dynamic, updated by read())

        double updateInterval = 0.1; // default to 10 hz
        double lastUpdateTime = 0;
    };
}; // namespace astra

#endif // SENSOR_H