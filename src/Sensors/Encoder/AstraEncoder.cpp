#include "AstraEncoder.h"

namespace astra
{

#pragma region Encoder Specific Functions

    Encoder_Astra::~Encoder_Astra() {}

    Encoder_Astra::Encoder_Astra(const char *name) : Sensor("Encoder", name)
    {
        addColumn("%d", &currentRelativeSteps, "Rel Steps");
    }

    int Encoder_Astra::getSteps() const { return currentRelativeSteps; }

#pragma endregion // Encoder Specific Functions

#pragma region Sensor Virtual Function Implementations

    void Encoder_Astra::setInitialSteps(int step)
    {
        initialSteps = step;
        LOGI("[Encoder]: Initial Steps set to: %d", step);
    }

#pragma region Data Reporting

#pragma endregion // Data Reporting

#pragma endregion // Sensor Virtual Function Implementations
}