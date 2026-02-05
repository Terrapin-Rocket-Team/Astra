#ifndef HITL_GPS_H
#define HITL_GPS_H

#include "../GPS/GPS.h"
#include "HITLSensorBuffer.h"

namespace astra
{
    /**
     * HITLGPS: Hardware-In-The-Loop GPS sensor
     *
     * Simulated GPS that reads position data from the HITLSensorBuffer
     * instead of physical hardware.
     *
     * Usage:
     *   HITLGPS* gps = new HITLGPS();
     *   gps->begin();
     *   // ... in update loop ...
     *   gps->update();  // Reads from HITLSensorBuffer
     *   Vector<3> pos = gps->getPos();  // lat, lon, alt
     */
    class HITLGPS : public GPS
    {
    public:
        HITLGPS(const char *name = "HITL_GPS")
            : GPS(name)
        {
            updateInterval = 0;
        }

        virtual ~HITLGPS() {}

    protected:
        int init() override
        {
            // No hardware initialization needed
            // Initialize GPS base class parameters
            hasFirstFix = false;
            hasFix = false;
            fixQual = 0;
            return 0;
        }

        int read() override
        {
            // Read from HITL sensor buffer
            HITLSensorBuffer &buffer = HITLSensorBuffer::instance();

            // Update position (lat, lon, alt)
            position.x() = buffer.data.gps_lat;
            position.y() = buffer.data.gps_lon;
            position.z() = buffer.data.gps_alt;

            // Update fix status
            hasFix = buffer.data.gps_fix;
            fixQual = buffer.data.gps_fix_quality;
            heading = buffer.data.gps_heading;

            // Set first fix flag if we just got a fix
            if (hasFix && !hasFirstFix)
            {
                hasFirstFix = true;
                calcInitialValuesForDistance();
            }

            return 0;
        }
    };

} // namespace astra

#endif // HITL_GPS_H
