#ifndef HITL_PARSER_H
#define HITL_PARSER_H

#include "../Sensors/HITL/HITLSensorBuffer.h"
#include "../RecordData/Logging/EventLogger.h"
#include <Arduino.h>
#include <cstdlib>

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
            // Parse CSV data with strtod/strtol for robust embedded behavior.
            // Format: timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
            const char *p = data;
            char *end = nullptr;

            auto parseDouble = [&](double &out) -> bool
            {
                out = strtod(p, &end);
                if (end == p)
                    return false;
                p = end;
                return true;
            };

            auto consumeComma = [&]() -> bool
            {
                if (*p != ',')
                    return false;
                ++p;
                return true;
            };

            auto parseInt = [&](int &out) -> bool
            {
                long v = strtol(p, &end, 10);
                if (end == p)
                    return false;
                out = static_cast<int>(v);
                p = end;
                return true;
            };

            int gpsFixInt = 0;
            int gpsFixQuality = 0;

            if (!parseDouble(buffer.data.timestamp) || !consumeComma() ||
                !parseDouble(buffer.data.accel.x()) || !consumeComma() ||
                !parseDouble(buffer.data.accel.y()) || !consumeComma() ||
                !parseDouble(buffer.data.accel.z()) || !consumeComma() ||
                !parseDouble(buffer.data.gyro.x()) || !consumeComma() ||
                !parseDouble(buffer.data.gyro.y()) || !consumeComma() ||
                !parseDouble(buffer.data.gyro.z()) || !consumeComma() ||
                !parseDouble(buffer.data.mag.x()) || !consumeComma() ||
                !parseDouble(buffer.data.mag.y()) || !consumeComma() ||
                !parseDouble(buffer.data.mag.z()) || !consumeComma() ||
                !parseDouble(buffer.data.pressure) || !consumeComma() ||
                !parseDouble(buffer.data.temperature) || !consumeComma() ||
                !parseDouble(buffer.data.gps_lat) || !consumeComma() ||
                !parseDouble(buffer.data.gps_lon) || !consumeComma() ||
                !parseDouble(buffer.data.gps_alt) || !consumeComma() ||
                !parseInt(gpsFixInt) || !consumeComma() ||
                !parseInt(gpsFixQuality) || !consumeComma() ||
                !parseDouble(buffer.data.gps_heading))
            {
                LOGE("HITL: Parse error");
                return false;
            }

            while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
                ++p;
            // Preserve the original protocol's forward-compatible behavior:
            // explicitly comma-delimited extension fields may follow the
            // required 18 fields. Reject text attached to the heading itself.
            if (*p != '\0' && *p != ',')
            {
                LOGE("HITL: Parse error trailing data");
                return false;
            }

            buffer.data.gps_fix = (gpsFixInt != 0);
            buffer.data.gps_fix_quality = gpsFixQuality;

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
