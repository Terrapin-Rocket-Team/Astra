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
     * Parses incoming HITL messages from desktop simulation and populates
     * the HITLSensorBuffer with sensor data.
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
     * Usage with SerialMessageRouter (recommended):
     *   SerialMessageRouter router;
     *   router.withInterface(&Serial)
     *         .withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
     *             double simTime;
     *             if (HITLParser::parse(msg, simTime)) {
     *                 astraSys->update(simTime);
     *             }
     *         });
     *
     * Legacy manual usage:
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
         * Parse HITL data (without prefix) and inject into HITLSensorBuffer
         * Use this with SerialMessageRouter which automatically strips the prefix.
         *
         * @param data CSV data without "HITL/" prefix
         * @param timestamp Output parameter for extracted simulation time
         * @return true if parsing successful, false otherwise
         *
         * Example: parse("1.234,0.0,0.0,9.81,...", simTime)
         */
        static bool parse(const char *data, double &timestamp)
        {
            if (!data)
            {
                LOGE("HITL: Null data");
                return false;
            }

            // Get buffer instance
            HITLSensorBuffer &buffer = HITLSensorBuffer::instance();

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
                                     (int *)&buffer.data.gps_fix,
                                     &buffer.data.gps_fix_quality,
                                     &buffer.data.gps_heading);

            if (itemsParsed != 18)
            {
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
         * Parse HITL data without timestamp extraction
         */
        static bool parse(const char *data)
        {
            double timestamp;
            return parse(data, timestamp);
        }

        /**
         * Parse a HITL message and inject data into HITLSensorBuffer
         * Legacy method that expects full line with "HITL/" prefix.
         *
         * @param line Full message line including "HITL/" prefix
         * @param timestamp Output parameter for extracted simulation time
         * @return true if parsing successful, false otherwise
         */
        static bool parseAndInject(const char *line, double &timestamp)
        {
            if (!line)
            {
                LOGE("HITL: Null line");
                return false;
            }

            // Verify prefix
            if (strncmp(line, "HITL/", 5) != 0)
            {
                LOGE("HITL: Invalid prefix");
                return false;
            }

            // Skip prefix and use the new parse method
            const char *data = line + 5;
            return parse(data, timestamp);
        }

        /**
         * Simplified version without timestamp extraction
         */
        static bool parseAndInject(const char *line)
        {
            double timestamp;
            return parseAndInject(line, timestamp);
        }
    };

} // namespace astra

#endif // HITL_PARSER_H
