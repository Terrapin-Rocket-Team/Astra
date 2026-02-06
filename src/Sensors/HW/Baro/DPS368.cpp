#include "DPS368.h"

using namespace astra;

DPS368::DPS368(const char *name, TwoWire *bus, uint8_t addr) : Barometer(name), addr(addr), bus(bus) {}

DPS368::DPS368(TwoWire *bus, uint8_t addr) : DPS368("DPS368", bus, addr) {}

int DPS368::init()
{
    if (!dps.begin_I2C(addr, bus))
    {
        return -1;
    }

    // Set up sampling rate and oversampling
    dps.configurePressure(DPS310_64HZ, DPS310_32SAMPLES);
    dps.configureTemperature(DPS310_32HZ, DPS310_8SAMPLES);

    // Operation mode of the sensor. See section 8.5 of the datasheet.
    dps.setMode(DPS310_CONT_PRESTEMP);

    return 0;
}

int DPS368::read()
{
    sensors_event_t temp_event, pressure_event;

    /* getEvents returns true or false depending on whether the sensors were successfully read or not */
    if (dps.getEvents(&temp_event, &pressure_event))
    {
        this->temp = temp_event.temperature;
        this->pressure = pressure_event.pressure;
        return 0;
    }
    else
    {
        LOGE("Failed to read data from DPS368 sensor");
        return -1;
    }
}
