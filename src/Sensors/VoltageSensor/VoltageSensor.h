#ifndef VOLTAGESENSOR_H
#define VOLTAGE_SENSOR_H

#include "../Sensor.h"

// TODO: Update
#if defined(ENV_TEENSY)
#define PLATFORM_DEFAULT_REF_VOLTAGE 3.3
#define PLATFORM_ADC_BITS 10
#elif defined(ENV_STM)
#define PLATFORM_DEFAULT_REF_VOLTAGE 3.3
#define PLATFORM_ADC_BITS 16
#elif defined(ENV_ESP)
#define PLATFORM_DEFAULT_REF_VOLTAGE 3.1
#define PLATFORM_ADC_BITS 12
#else
#define PLATFORM_DEFAULT_REF_VOLTAGE 0
#define PLATFORM_ADC_BITS 0
#endif

namespace astra
{
    class VoltageSensor : public Sensor
    {
    public:
        VoltageSensor(int pin, const char *name = "Voltage Sensor");
        VoltageSensor(int pin, int r1, int r2, const char *name = "Voltage Sensor", double refVoltage = PLATFORM_DEFAULT_REF_VOLTAGE);

        double getVoltage() const;
        int getRawValue() const;

    protected:
        bool init() override;
        bool read() override;

    private:
        int pin;
        int r1;
        int r2;
        double refVoltage;
        double voltage;
        int rawValue;
    };
}

#endif