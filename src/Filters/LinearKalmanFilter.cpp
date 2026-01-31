#include "LinearKalmanFilter.h"

namespace astra
{
    LinearKalmanFilter::LinearKalmanFilter(int measurementSize, int controlSize, int stateSize)
    {

        X = Matrix(stateSize, 1, new double[stateSize]());                                 // State vector
        P = Matrix(stateSize, stateSize, new double[stateSize * stateSize]());             // Error covariance matrix
        K = Matrix(stateSize, measurementSize, new double[stateSize * measurementSize]()); // Kalman gain

        this->measurementSize = measurementSize;
        this->controlSize = controlSize;
        this->stateSize = stateSize;
    }

    LinearKalmanFilter::LinearKalmanFilter(Matrix X, Matrix P)
    {
        this->X = X;
        this->P = P;
        this->K = Matrix(X.getRows(), P.getRows(), new double[X.getRows() * P.getRows()]());

        this->measurementSize = K.getCols();
        this->controlSize = K.getRows();
    }

    void LinearKalmanFilter::predictState(double dt, Matrix U)
    {
        X = getF(dt) * X + getG(dt) * U;
    }

    void LinearKalmanFilter::estimateState(Matrix measurement)
    {
        X = X + K * (measurement - getH() * X);
    }

    void LinearKalmanFilter::calculateKalmanGain()
    {
        K = P * getH().transpose() * (getH() * P * getH().transpose() + getR()).inverse();
    }

    void LinearKalmanFilter::covarianceUpdate()
    {
        int n = X.getRows();
        P = (Matrix::ident(n) - K * getH()) * P * (Matrix::ident(n) - K * getH()).transpose() + K * getR() * K.transpose();
    }

    void LinearKalmanFilter::covarianceExtrapolate(double dt)
    {
        P = getF(dt) * P * getF(dt).transpose() + getQ(dt);
    }

    // Split predict/update methods for different update rates
    void LinearKalmanFilter::predict(double dt, Matrix control)
    {
        predictState(dt, control);
        covarianceExtrapolate(dt);
    }

    void LinearKalmanFilter::update(Matrix measurement)
    {
        calculateKalmanGain();
        estimateState(measurement);
        covarianceUpdate();
    }

    // Flexible update with custom H and R matrices
    // Allows partial measurements (e.g., GPS horizontal only, baro vertical only)
    void LinearKalmanFilter::update(Matrix z, Matrix H, Matrix R)
    {
        // Calculate Kalman gain for this specific measurement
        Matrix K_custom = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // Update state estimate
        X = X + K_custom * (z - H * X);

        // Update covariance (Joseph form for numerical stability)
        int n = X.getRows();
        Matrix I_KH = Matrix::ident(n) - K_custom * H;
        P = I_KH * P * I_KH.transpose() + K_custom * R * K_custom.transpose();
    }

} // namespace astra
