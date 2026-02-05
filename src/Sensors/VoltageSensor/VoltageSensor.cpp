#include "VoltageSensor.h"
#include <Arduino.h>

namespace astra
{
    VoltageSensor::VoltageSensor(int pin, const char *name)
        : Sensor(name),
          pin(pin),
          r1(0),
          r2(0),
          refVoltage(PLATFORM_DEFAULT_REF_VOLTAGE),
          voltage(0.0),
          rawValue(0)
    {
        addColumn("%0.3f", &voltage, "Voltage (V)");
        setUpdateRate(10);
    }

    VoltageSensor::VoltageSensor(int pin, int r1, int r2, const char *name, double refVoltage)
        : Sensor(name),
          pin(pin),
          r1(r1),
          r2(r2),
          refVoltage(refVoltage),
          voltage(0.0),
          rawValue(0)
    {
        addColumn("%0.3f", &voltage, "Voltage (V)");
    }

    int VoltageSensor::init()
    {
        pinMode(pin, INPUT);
        return 0;
    }

    int VoltageSensor::read()
    {
        rawValue = analogRead(pin);

        // Calculate ADC resolution
        int maxAdcValue = (1 << PLATFORM_ADC_BITS) - 1;

        // Convert raw ADC value to voltage
        voltage = (rawValue / (double)maxAdcValue) * refVoltage;

        // Apply voltage divider calculation if r1 and r2 are set
        if (r1 > 0 && r2 > 0)
        {
            voltage = voltage * (r1 + r2) / (double)r2;
        }

        return 0;
    }

    double VoltageSensor::getVoltage() const
    {
        return voltage;
    }

    int VoltageSensor::getRawValue() const
    {
        return rawValue;
    }
}
