#ifndef ENCODER_Astra_H
#define ENCODER_Astra_H

#include "../Sensor.h"

namespace astra
{
    class Encoder_Astra : public Sensor
    {
    public:
        virtual ~Encoder_Astra();
        virtual int getSteps() const;
        virtual void setInitialSteps(int step);

    protected:
        Encoder_Astra(const char *name = "Encoder");
        int currentRelativeSteps;
        int initialSteps;
    };
}
#endif // ENCODER_Astra_H
