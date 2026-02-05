#include "Barometer.h"
#include <cmath>
#include <Arduino.h>
#define MEAN_SEA_LEVEL_PRESSURE_HPA 1013.25
namespace astra
{

#pragma region Barometer Specific Functions
    Barometer::Barometer(const char *name) : Sensor(name)
    {
        addColumn("%0.3f", &pressure, "Pres (hPa)");
        addColumn("%0.3f", &temp, "Temp (C)");
        addColumn("%0.3f", &altitudeASL, "Alt ASL (m)");
        setUpdateRate(20);
    }

    Barometer::~Barometer() {}

    double Barometer::getPressure() const { return pressure; }

    double Barometer::getTemp() const { return temp; }

    double Barometer::getTempF() const { return (temp * 9.0 / 5.0) + 32.0; }

    double Barometer::getPressureAtm() const { return pressure / MEAN_SEA_LEVEL_PRESSURE_HPA; }

    double Barometer::getASLAltFt() const { return altitudeASL * 3.28084; }

    double Barometer::getASLAltM() const { return altitudeASL; }

    double Barometer::calcAltitude(double pressure)
    {
        // Equation from NOAA, but for meters: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
        return 44307.69 * (1.0 - pow(pressure / MEAN_SEA_LEVEL_PRESSURE_HPA, 0.190284));
    }

#pragma endregion // Barometer Specific Functions

#pragma region Sensor Virtual Function Implementations

    int Barometer::update(double currentTime)
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

        altitudeASL = calcAltitude(pressure);

        // Health tracking - check for stuck pressure readings
        lastReadings[readingIndex] = pressure;
        readingIndex = (readingIndex + 1) % HEALTH_BUFFER_SIZE;

        if (consecutiveGoodReads < HEALTH_BUFFER_SIZE - 1)
        {
            consecutiveGoodReads++;
            return 0;
        }

        bool allIdentical = true;
        for (uint8_t i = 1; i < HEALTH_BUFFER_SIZE; i++)
        {
            if (lastReadings[i] != lastReadings[0])
            {
                allIdentical = false;
                break;
            }
        }

        if (allIdentical)
        {
            if (healthy)
                LOGW("Baro '%s' became unhealthy: stuck readings detected", getName());
            healthy = false;
            consecutiveGoodReads = 0;
        }
        else
        {
            if (!healthy)
                LOGI("Baro '%s' recovered: readings varying normally", getName());
            healthy = true;
        }

        return 0;
    }

    int Barometer::begin()
    {
        pressure = 0;
        temp = 0;
        altitudeASL = 0;
        int err = init();
        initialized = (err == 0);
        if (initialized)
        {
            read();
            altitudeASL = calcAltitude(pressure);
        }
        return err;
    }
}