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
        // ------------------------------- SENSOR TYPE IMPLEMENTATION ---------------------------------------------

        virtual const SensorType getType() const { return type; }        // Returns the type of the sensor
        virtual const char *getTypeString() const { return typeString; } // Returns the type of the sensor as a string

        // ----------------------------------------------------------------------------------------------------------

    protected:
        // --------------------------------- HARDWARE IMPLEMENTATION -----------------------------------------------

        Sensor(const char *type, const char *name = nullptr) : DataReporter(name)
        {
            this->type = fnv1a_32(type, strlen(type));
            typeString = type;
            autoUpdate = false;
        }

        // Sets up the sensor and stores any critical parameters. Needs to reset the sensor if it is already initialized. Called by begin()
        virtual bool init() = 0;

        // Physically reads the outputs from the sensor hardware. Called by update()
        virtual bool read() = 0;

        SensorType type;
        const char *typeString;
    };
}; // namespace astra

#endif // SENSOR_H