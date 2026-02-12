#ifndef BAROMETER_H
#define BAROMETER_H

#include "../Sensor.h"

namespace astra
{
    class Barometer : public Sensor
    {
    public:
        virtual ~Barometer();
        virtual double getPressure() const;    // hPa
        virtual double getTemp() const;        // deg C
        virtual double getTempF() const;       // deg F
        virtual double getPressureAtm() const; // atmospheres
        virtual double getASLAltFt() const;    // ft
        virtual double getASLAltM() const;     // meters

        static double calcAltitude(double pressure);

        // Sensor virtual functions - return 0 on success, error code on failure
        virtual int update(double currentTime = -1) override;
        virtual int begin() override;

    protected:
        Barometer(const char *name = "Barometer");
        double pressure = 0; // hPa
        double temp = 0; // deg c

        // Altitude-related data
        double altitudeASL = 0; // m

        // Health tracking for stuck-reading detection
        static constexpr uint8_t HEALTH_BUFFER_SIZE = 3;
        CircBuffer<double> lastReadings;
        uint8_t consecutiveGoodReads = 0;

        virtual void updateHealth() override;
    };
}
#endif // BAROMETER_H
