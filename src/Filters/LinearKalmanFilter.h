#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

#include "Filter.h"
#include "../Math/Matrix.h"

namespace astra {

class LinearKalmanFilter : public Filter {
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

    virtual Matrix iterate(double dt, Matrix measurement, Matrix control);

    // Override core interface methods
    virtual int getMeasurementSize() const override { return measurementSize; }
    virtual int getInputSize() const override { return controlSize; }
    virtual int getStateSize() const override { return stateSize; }
    virtual double *iterate(double dt, double *state, double *measurements, double *controlVars) override;

    // Split predict/update for different update rates
    virtual void predict(double dt, Matrix control);
    virtual void predict(double dt, double *controlVars);
    virtual void update(Matrix measurement);
    virtual void update(double *measurements);

    // Get current state estimate
    virtual Matrix getState() const { return X; }
    virtual void getState(double *state) const;
    virtual void setState(double *state);

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
