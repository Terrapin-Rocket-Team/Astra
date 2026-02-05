#ifndef DUAL_RANGE_ACCEL_H
#define DUAL_RANGE_ACCEL_H

#include "Accel.h"

namespace astra
{
    class DualRangeAccel : public Accel
    {

    public:
        DualRangeAccel(Accel *lowG, Accel *highG, double maxLowG, double minHighG, const char *name = "DualRangeAccel")
            : Accel(name), lowGAccel(lowG), highGAccel(highG), maxLowG(maxLowG), minHighG(minHighG)
        {
            lowGAccel->setAutoUpdate(false);
            highGAccel->setAutoUpdate(false);
        }
        virtual ~DualRangeAccel() {}

        // Because this class deals with two separate sensors, setting one orientation for both of them does not make sense.
        void setMountingOrientation(MountingOrientation orientation) override
        {
            (void)orientation;
            LOGW("DualRangeAccel does not support mounting orientation! Set each sensor's mounting orientation separately.");
        }

        Vector<3> getAccel() const override
        {
            return acc;
        }

    protected:
        int init() override
        {
            int low = lowGAccel->begin();
            int high = highGAccel->begin();
            if (low == 0 && high == 0)
                return 0;
            if (low != 0)
                return low;
            return high;
        }

        int read() override
        {
            if (lowGAccel->update() && highGAccel->update())
            {
                Vector<3> lowVal = lowGAccel->getAccel();
                Vector<3> highVal = highGAccel->getAccel();

                double magLow = lowVal.magnitude();
                double magHigh = highVal.magnitude();

                // 1. Pure Low-G Range
                if (magLow <= minHighG)
                {
                    alpha = 0.0;
                }

                // 2. Pure High-G Range
                else if (magHigh >= maxLowG)
                {
                    alpha = 1.0;
                }

                // 3. Transition Range (Linear Interpolation)
                // Calculate alpha: 0.0 at minHighG, 1.0 at maxLowG
                else
                {
                    alpha = (magHigh - maxLowG) / (minHighG - maxLowG);
                }
                acc = lowVal * (1.0 - alpha) + highVal * alpha;
                return true;
            }
            return false;
        }

    private:
        Accel *lowGAccel;
        Accel *highGAccel;
        double maxLowG;
        double minHighG;
        double alpha;
    };
}

#endif // DUAL_RANGE_ACCEL_H