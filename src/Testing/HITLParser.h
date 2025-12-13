#ifndef HITL_PARSER_H
#define HITL_PARSER_H

#include "../Sensors/HITL/HITLSensorBuffer.h"
#include "../RecordData/Logging/EventLogger.h"
#include <Arduino.h>

namespace astra
{
    /**
     * HITLParser: Parser for Hardware-In-The-Loop protocol messages
     *
     * Parses incoming "HITL/" prefixed messages from desktop simulation
     * and populates the HITLSensorBuffer with sensor data.
     *
     * Protocol format:
     * HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
     *
     * Where:
     * - timestamp: simulation time (s)
     * - ax,ay,az: acceleration (m/s^2)
     * - gx,gy,gz: angular velocity (rad/s)
     * - mx,my,mz: magnetic field (uT)
     * - pressure: pressure (hPa / mbar)
     * - temp: temperature (C)
     * - lat,lon: GPS coordinates (decimal degrees)
     * - alt: GPS altitude MSL (m)
     * - fix: GPS fix status (0 or 1)
     * - fixqual: GPS fix quality (number of satellites)
     * - heading: GPS heading (degrees)
     *
     * Usage:
     *   if (Serial.available()) {
     *       String line = Serial.readStringUntil('\n');
     *       if (line.startsWith("HITL/")) {
     *           double simTime;
     *           if (HITLParser::parseAndInject(line.c_str(), simTime)) {
     *               astraSys->update(simTime);
     *           }
     *       }
     *   }
     */
    class HITLParser
    {
    public:
        /**
         * Parse a HITL message and inject data into HITLSensorBuffer
         * @param line Full message line including "HITL/" prefix
         * @param timestamp Output parameter for extracted simulation time
         * @return true if parsing successful, false otherwise
         */
        static bool parseAndInject(const char* line, double& timestamp)
        {
            // Verify prefix
            if (strncmp(line, "HITL/", 5) != 0) {
                LOGE("HITL: Invalid prefix");
                return false;
            }

            // Skip prefix
            const char* data = line + 5;

            // Get buffer instance
            HITLSensorBuffer& buffer = HITLSensorBuffer::instance();

            // Parse CSV data
            // Format: timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
            int itemsParsed = sscanf(data, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf",
                &buffer.data.timestamp,
                &buffer.data.accel.x(),
                &buffer.data.accel.y(),
                &buffer.data.accel.z(),
                &buffer.data.gyro.x(),
                &buffer.data.gyro.y(),
                &buffer.data.gyro.z(),
                &buffer.data.mag.x(),
                &buffer.data.mag.y(),
                &buffer.data.mag.z(),
                &buffer.data.pressure,
                &buffer.data.temperature,
                &buffer.data.gps_lat,
                &buffer.data.gps_lon,
                &buffer.data.gps_alt,
                (int*)&buffer.data.gps_fix,
                &buffer.data.gps_fix_quality,
                &buffer.data.gps_heading
            );

            if (itemsParsed != 18) {
                LOGE("HITL: Parse error, got %d items (expected 18)", itemsParsed);
                return false;
            }

            // Mark buffer as ready
            buffer.dataReady = true;

            // Return timestamp
            timestamp = buffer.data.timestamp;

            return true;
        }

        /**
         * Simplified version without timestamp extraction
         */
        static bool parseAndInject(const char* line)
        {
            double timestamp;
            return parseAndInject(line, timestamp);
        }
    };

} // namespace astra

#endif // HITL_PARSER_H
