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

        // Sensor virtual functions
        virtual bool update() override;
        virtual bool begin() override;

    protected:
        Barometer(const char *name = "Barometer");
        double pressure = 0;
        double temp = 0;

        // Altitude-related data
        double altitudeASL = 0;

        double calcAltitude(double pressure);
    };
}
#endif // BAROMETER_H
