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

    //In this function, we predict the next state of the system based on the current state and the dynamics of the system
    //ex. using F = ma to predict where position will be. We use a constant velcity model (getF) and propogate it some 
    //dt into the future thus geting position. The getG is for the acceleration. Acceleration is an input with matrix U. 
    //Comes straight from the IM. 
    void LinearKalmanFilter::predictState(double dt, Matrix U)
    {
        X = getF(dt) * X + getG(dt) * U;
    }
    //surprise value; get hte diff between measured and predicted state. Produces the corrected state estimate. 
    void LinearKalmanFilter::estimateState(Matrix measurement)
    {
        X = X + K * (measurement - getH() * X);
    }
    //Kalman ratio = predicted uncertainty / (predicted uncertainty + measurement uncertainty)
    //Larger P = trust measruements more, larger R = trust prediction more. R is the measruement noise (gps + 
    //baro noise). GPS noise defaults to 5 and baro noise defaults to 1. No data backs this. 
    void LinearKalmanFilter::calculateKalmanGain()
    {
        K = P * getH().transpose() * (getH() * P * getH().transpose() + getR()).inverse();
    }
    //updates state uncertainity after a measruement. After incorporating the measrumeent to calcualte how uncertain 
    //the state is. Part f the final part of update step. Done very last
    void LinearKalmanFilter::covarianceUpdate()
    {
        int n = X.getRows();
        P = (Matrix::ident(n) - K * getH()) * P * (Matrix::ident(n) - K * getH()).transpose() + K * getR() * K.transpose();
    }

    // P represents the uncertainity in our state estimation (i.e how confident I am that the predicted state is correct)
    //We start by propogating the constant velocity (getF) and getting position. The F*P*Ft is the propogation of the old state
    //uncertainity into the new state uncertainity using ideal dynamics. So if I have P1 at t1 and then I want P2 at t2 I would
    //this. Again, P is the uncertainity of our state estimate. Q is the uncertainity from our system evolving (i.e. the model is
    //not perfect and we know that). The Q is taken from 1D model --> 3D model. Q is the uncertainity of the current model (model isn't
    //perfect). The matrix is a given dt^4/4 via online, but the scaling factor q = process noise shoudl be caculated based on the system.
    //Right now, q is 0.01 for position (we assume position dynamic is good) and 1.0 for velocity (we assume velocity model is bad)
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
    //update, now this is where we taking into account actual measurements. Takes in GPS and baro measuremenets 
    //1 to use the measurement (GPS gives px, py. Baro gives pz) and 0 to not use the measurement.
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
