#include "LightSensor.h"

namespace astra
{
#pragma region LightSensor Specific Functions

    LightSensor::LightSensor(const char *name) : Sensor("Light Sensor", name), lux(0), initialLux(0)
    {
        addColumn("%0.3f", &lux, "Lux (Lux)");
    }

    LightSensor::~LightSensor() {}

    const double LightSensor::getLux() const { return lux; }

#pragma endregion // LightSensor Specific Functions

#pragma region Sensor Virtual Function Implementations

#pragma endregion // Sensor Virtual Function Implementations
}