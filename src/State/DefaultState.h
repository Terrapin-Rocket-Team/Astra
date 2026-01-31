#ifndef DEFAULT_STATE_H
#define DEFAULT_STATE_H

#include "State.h"
#include "../Filters/DefaultKalmanFilter.h"

namespace astra
{
    /**
     * DefaultState - Out-of-the-box state estimation
     *
     * This class provides a ready-to-use state estimator for most aerial vehicle
     * applications. It creates and manages its own DefaultKalmanFilter and MahonyAHRS.
     *
     * Perfect for:
     * - Beginners who want state estimation to "just work"
     * - Standard drones, model aircraft, or basic rockets
     * - Prototyping before creating custom implementations
     *
     * Features:
     * - Asynchronous sensor updates (GPS, baro update independently)
     * - Auto-calibration of orientation filter
     * - Tunable filter parameters for different applications
     *
     * For advanced use cases (custom dynamics, rocket-specific models):
     * - Use State base class with custom LinearKalmanFilter implementation
     * - Or derive from DefaultState and override specific behaviors
     */
    class DefaultState : public State
    {
    public:
        /**
         * Constructor - creates all necessary filters internally
         * @param processNoise - KF process noise tuning (default: 1.0)
         * @param gpsNoise - GPS horizontal noise in meters (default: 5.0)
         * @param baroNoise - Barometer vertical noise in meters (default: 2.0)
         * @param mahonyKp - Mahony proportional gain (default: 0.5)
         * @param mahonyKi - Mahony integral gain (default: 0.0)
         */
        DefaultState(double processNoise = 1.0,
                     double gpsNoise = 5.0,
                     double baroNoise = 2.0,
                     double mahonyKp = 0.5,
                     double mahonyKi = 0.0);

        virtual ~DefaultState();

        // Override update to handle asynchronous sensor updates
        virtual bool update(double currentTime = -1) override;

        // Access to the default filter (for advanced tuning)
        DefaultKalmanFilter *getDefaultKalmanFilter() { return ownedKalmanFilter; }

    private:
        // Owned filter instances (created and managed by this class)
        DefaultKalmanFilter *ownedKalmanFilter;
        MahonyAHRS *ownedOrientationFilter;
    };

} // namespace astra

#endif // DEFAULT_STATE_H
