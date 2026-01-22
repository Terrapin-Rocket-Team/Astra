#ifndef BODY_FRAME_DATA_H
#define BODY_FRAME_DATA_H

#include "../../Math/Vector.h"

namespace astra
{
    /**
     * BodyFrameData - Output from SensorManager
     *
     * Contains all sensor data transformed to the rocket's body frame.
     * Body frame convention:
     *   - X: Forward (through nosecone)
     *   - Y: Right (starboard)
     *   - Z: Down (towards fins)
     *
     * This is the standard aerospace body frame (NED-like but attached to vehicle).
     * State will transform this to inertial frame using AHRS orientation.
     */
    struct BodyFrameData
    {
        // Inertial measurements (body frame)
        Vector<3> accel;    // Body-frame acceleration (m/s²)
        Vector<3> gyro;     // Body-frame angular velocity (rad/s)
        Vector<3> mag;      // Body-frame magnetic field (μT)

        // Barometric measurements (scalar, no frame)
        double pressure;    // Barometric pressure (hPa)
        double temperature; // Temperature (°C)
        double baroAltASL;  // Barometric altitude ASL (m)

        // Availability flags
        bool hasAccel;
        bool hasGyro;
        bool hasMag;
        bool hasBaro;

        // Timestamp
        double timestamp;   // Time of measurement (s since boot)

        BodyFrameData()
            : accel(), gyro(), mag(),
              pressure(0), temperature(0), baroAltASL(0),
              hasAccel(false), hasGyro(false), hasMag(false), hasBaro(false),
              timestamp(0)
        {
        }
    };

    /**
     * SensorHealth - Health status for a sensor
     */
    enum class SensorHealth
    {
        UNKNOWN,      // Not yet evaluated
        HEALTHY,      // Operating normally
        DEGRADED,     // Operating but with reduced accuracy
        FAILED,       // Not responding or producing invalid data
        DISABLED      // Manually disabled
    };

    /**
     * SensorHealthInfo - Detailed health information for a sensor
     */
    struct SensorHealthInfo
    {
        SensorHealth status;
        uint32_t lastUpdateTime;    // Last successful read (ms)
        uint32_t failureCount;      // Consecutive failures
        uint32_t totalFailures;     // Total failures since init
        const char* failureReason;  // Human-readable failure reason

        SensorHealthInfo()
            : status(SensorHealth::UNKNOWN),
              lastUpdateTime(0),
              failureCount(0),
              totalFailures(0),
              failureReason(nullptr)
        {
        }
    };

} // namespace astra

#endif // BODY_FRAME_DATA_H
