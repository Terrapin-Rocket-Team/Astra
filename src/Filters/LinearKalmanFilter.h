#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

#include "../Math/Matrix.h"

namespace astra {

class LinearKalmanFilter {
public:

    int measurementSize;
    int controlSize;
    int stateSize;

    // Constructors
    LinearKalmanFilter(int measurementSize, int controlSize, int stateSize);
    LinearKalmanFilter(Matrix X, Matrix P);
    virtual ~LinearKalmanFilter() = default;

    // Virtual getter methods for matrices, to be overridden by subclasses
    virtual void initialize() = 0;
    virtual Matrix getF(double dt) = 0;
    virtual Matrix getG(double dt) = 0;
    virtual Matrix getH() = 0;
    virtual Matrix getR() = 0;
    virtual Matrix getQ(double dt) = 0;

    // Query filter dimensions
    virtual int getMeasurementSize() const { return measurementSize; }
    virtual int getInputSize() const { return controlSize; }
    virtual int getStateSize() const { return stateSize; }

    // Split predict/update for different update rates
    virtual void predict(double dt, Matrix control);
    virtual void update(Matrix measurement);

    // Flexible update with custom measurement matrix and noise
    // Allows sensor-specific updates (e.g., GPS-only, baro-only)
    virtual void update(Matrix z, Matrix H, Matrix R);

    // Sensor-specific measurement updates for standard flight sensors
    // These work with any 6-state [px, py, pz, vx, vy, vz] LKF
    // Override in subclass if using different state representation
    // If noise parameter is -1, uses the value set in constructor (via getGPSNoise()/getBaroNoise())
    virtual void updateGPS(double px, double py, double gpsNoise = -1.0);
    virtual void updateBaro(double pz, double baroNoise = -1.0);
    virtual void updateGPSBaro(double px, double py, double pz, double gpsNoise = -1.0, double baroNoise = -1.0);

    // Get current state estimate
    virtual Matrix getState() const { return X; }

    // Get noise parameters (used as defaults when updateGPS/updateBaro called with -1)
    // Subclasses should override these if they store noise parameters
    virtual double getGPSNoise() const { return 5.0; }   // Default GPS noise
    virtual double getBaroNoise() const { return 2.0; }  // Default baro noise

protected:
    // Instance variables to store matrices
    Matrix X; // State vector
    Matrix P; // Error covariance matrix
    Matrix K; // Kalman gain

    virtual void predictState(double dt, Matrix control);
    virtual void estimateState(Matrix measurement);
    virtual void calculateKalmanGain();
    virtual void covarianceUpdate();
    virtual void covarianceExtrapolate(double dt);
};

} // namespace astra

#endif // LINEAR_KALMAN_FILTER_H
