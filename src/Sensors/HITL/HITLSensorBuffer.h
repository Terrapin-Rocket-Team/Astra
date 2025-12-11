#ifndef HITL_SENSOR_BUFFER_H
#define HITL_SENSOR_BUFFER_H

#include "../../Math/Vector.h"

namespace astra
{
    /**
     * HITLSensorBuffer: Singleton buffer for Hardware-In-The-Loop sensor data
     *
     * This buffer stores simulated sensor data received from a desktop simulation
     * via the HITL protocol. HITL sensors read from this shared buffer instead of
     * querying physical hardware.
     *
     * Usage:
     *   // Desktop sends: HITL/timestamp,ax,ay,az,gx,gy,gz,...
     *   HITLParser::parseAndInject(line);
     *
     *   // HITL sensors read:
     *   HITLSensorBuffer& buf = HITLSensorBuffer::instance();
     *   Vector<3> accel = buf.data.accel;
     */
    class HITLSensorBuffer
    {
    public:
        // Sensor data packet structure
        struct SensorData
        {
            double timestamp;           // Simulation time (seconds)

            // IMU data
            Vector<3> accel;           // Acceleration (m/s^2)
            Vector<3> gyro;            // Angular velocity (rad/s)
            Vector<3> mag;             // Magnetic field (uT)

            // Barometer data
            double pressure;           // Pressure (hPa)
            double temperature;        // Temperature (C)

            // GPS data
            double gps_lat;            // Latitude (decimal degrees)
            double gps_lon;            // Longitude (decimal degrees)
            double gps_alt;            // Altitude MSL (m)
            bool gps_fix;              // GPS has valid fix
            int gps_fix_quality;       // Number of satellites
            double gps_heading;        // Heading (degrees)
        } data;

        bool dataReady;                // True if data has been updated

        // Get singleton instance
        static HITLSensorBuffer& instance()
        {
            static HITLSensorBuffer buffer;
            return buffer;
        }

        // Mark data as consumed (optional, for synchronization)
        void markConsumed() { dataReady = false; }

    private:
        // Singleton - private constructor
        HITLSensorBuffer() : dataReady(false)
        {
            // Initialize with default values
            data.timestamp = 0.0;
            data.accel = Vector<3>(0, 0, -9.81);  // Default: 1G down
            data.gyro = Vector<3>(0, 0, 0);
            data.mag = Vector<3>(0, 0, 0);
            data.pressure = 1013.25;              // Sea level
            data.temperature = 25.0;
            data.gps_lat = 0.0;
            data.gps_lon = 0.0;
            data.gps_alt = 0.0;
            data.gps_fix = false;
            data.gps_fix_quality = 0;
            data.gps_heading = 0.0;
        }

        // Delete copy and assignment
        HITLSensorBuffer(const HITLSensorBuffer&) = delete;
        HITLSensorBuffer& operator=(const HITLSensorBuffer&) = delete;
    };

} // namespace astra

#endif // HITL_SENSOR_BUFFER_H
