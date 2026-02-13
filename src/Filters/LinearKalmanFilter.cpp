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

        this->stateSize = X.getRows();
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

    // =================== Standard Flight Sensor Updates ===================
    // These work with any 6-state [px, py, pz, vx, vy, vz] LKF

    void LinearKalmanFilter::updateGPS(double px, double py, double gpsNoise)
    {
        // Use member variable if not specified
        if (gpsNoise < 0) {
            gpsNoise = getGPSNoise();
        }

        // GPS measures horizontal position [px, py]
        // H = [1 0 0 0 0 0]
        //     [0 1 0 0 0 0]
        double h_data[12] = {
            1, 0, 0, 0, 0, 0,  // px measurement
            0, 1, 0, 0, 0, 0   // py measurement
        };
        Matrix H(2, 6, h_data);

        double gpsVar = gpsNoise * gpsNoise;
        double r_data[4] = {
            gpsVar, 0,
            0, gpsVar
        };
        Matrix R(2, 2, r_data);

        double z_data[2] = {px, py};
        Matrix z(2, 1, z_data);

        update(z, H, R);
    }

    void LinearKalmanFilter::updateBaro(double pz, double baroNoise)
    {
        // Use member variable if not specified
        if (baroNoise < 0) {
            baroNoise = getBaroNoise();
        }

        // Barometer measures vertical position [pz]
        // H = [0 0 1 0 0 0]
        double h_data[6] = {
            0, 0, 1, 0, 0, 0  // pz measurement
        };
        Matrix H(1, 6, h_data);

        double baroVar = baroNoise * baroNoise;
        double r_data[1] = {baroVar};
        Matrix R(1, 1, r_data);

        double z_data[1] = {pz};
        Matrix z(1, 1, z_data);

        update(z, H, R);
    }

    void LinearKalmanFilter::updateGPSBaro(double px, double py, double pz, double gpsNoise, double baroNoise)
    {
        // Use member variables if not specified
        if (gpsNoise < 0) {
            gpsNoise = getGPSNoise();
        }
        if (baroNoise < 0) {
            baroNoise = getBaroNoise();
        }

        // Combined GPS + Baro measurement [px, py, pz]
        // H = [1 0 0 0 0 0]
        //     [0 1 0 0 0 0]
        //     [0 0 1 0 0 0]
        double h_data[18] = {
            1, 0, 0, 0, 0, 0,  // px measurement
            0, 1, 0, 0, 0, 0,  // py measurement
            0, 0, 1, 0, 0, 0   // pz measurement
        };
        Matrix H(3, 6, h_data);

        double gpsVar = gpsNoise * gpsNoise;
        double baroVar = baroNoise * baroNoise;
        double r_data[9] = {
            gpsVar, 0, 0,
            0, gpsVar, 0,
            0, 0, baroVar
        };
        Matrix R(3, 3, r_data);

        double z_data[3] = {px, py, pz};
        Matrix z(3, 1, z_data);

        update(z, H, R);
    }

} // namespace astra
