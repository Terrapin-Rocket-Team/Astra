#ifndef DEFAULT_KALMAN_FILTER_H
#define DEFAULT_KALMAN_FILTER_H

#include "LinearKalmanFilter.h"

namespace astra
{
    /**
     * DefaultKalmanFilter - A 6-state position/velocity Kalman filter
     *
     * State vector: [px, py, pz, vx, vy, vz]
     * Control input: [ax, ay, az] (earth-frame acceleration from orientation filter)
     * Measurements: GPS horizontal [px, py], Barometer vertical [pz]
     *
     * This is a general-purpose filter suitable for most aerial vehicle applications
     * (drones, rockets, model aircraft) using standard IMU + baro + GPS sensor suite.
     *
     * Supports asynchronous sensor updates:
     * - updateGPS() - horizontal position from GPS (when GPS has fix)
     * - updateBaro() - vertical position from barometer (high rate)
     * - updateFull() - all measurements together (if available)
     *
     * For custom applications (e.g., rocket-specific drag/thrust models),
     * derive from LinearKalmanFilter and override the matrix methods.
     */
    class DefaultKalmanFilter : public LinearKalmanFilter
    {
    public:
        /**
         * Constructor with tunable noise parameters
         * @param processNoise - Process noise scaling (higher = trust model less)
         * @param gpsNoise - GPS horizontal measurement noise in meters (default: 5m std dev)
         * @param baroNoise - Barometer vertical measurement noise in meters (default: 2m std dev)
         */
        DefaultKalmanFilter(double processNoise = 1.0, double gpsNoise = 5.0, double baroNoise = 2.0);
        virtual ~DefaultKalmanFilter() = default;

        // Initialize state and covariance matrices
        virtual void initialize() override;

        // Matrix getters (define the Kalman filter dynamics)
        virtual Matrix getF(double dt) override; // State transition matrix
        virtual Matrix getG(double dt) override; // Control input matrix
        virtual Matrix getH() override;          // Measurement matrix (full)
        virtual Matrix getR() override;          // Measurement noise covariance (full)
        virtual Matrix getQ(double dt) override; // Process noise covariance

        // Sensor-specific update methods (asynchronous)
        void updateGPS(double px, double py);      // Update with GPS horizontal position
        void updateBaro(double pz);                // Update with barometer altitude
        void updateFull(double px, double py, double pz); // Update with all measurements

    private:
        double processNoise;
        double gpsNoise;
        double baroNoise;

        // Matrices for partial measurements (initialized in constructor)
        Matrix H_gps;   // 2x6 measurement matrix for GPS horizontal
        Matrix R_gps;   // 2x2 noise covariance for GPS
        Matrix H_baro;  // 1x6 measurement matrix for baro vertical
        Matrix R_baro;  // 1x1 noise covariance for baro
    };

} // namespace astra

#endif // DEFAULT_KALMAN_FILTER_H
