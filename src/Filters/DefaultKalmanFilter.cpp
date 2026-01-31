#include "DefaultKalmanFilter.h"
#include <Arduino.h>

namespace astra
{
    DefaultKalmanFilter::DefaultKalmanFilter(double processNoise, double gpsNoise, double baroNoise)
        : LinearKalmanFilter(3, 3, 6), // 3 measurements, 3 control inputs, 6 states
          processNoise(processNoise),
          gpsNoise(gpsNoise),
          baroNoise(baroNoise)
    {
        // Initialize partial measurement matrices (these are constant)
        // GPS measures [px, py]
        double h_gps_data[12] = {
            1, 0, 0, 0, 0, 0,  // px measurement
            0, 1, 0, 0, 0, 0   // py measurement
        };
        H_gps = Matrix(2, 6, h_gps_data);

        double r_gps_data[4] = {
            gpsNoise * gpsNoise, 0,
            0, gpsNoise * gpsNoise
        };
        R_gps = Matrix(2, 2, r_gps_data);

        // Baro measures [pz]
        double h_baro_data[6] = {
            0, 0, 1, 0, 0, 0  // pz measurement
        };
        H_baro = Matrix(1, 6, h_baro_data);

        double r_baro_data[1] = {baroNoise * baroNoise};
        R_baro = Matrix(1, 1, r_baro_data);
    }

    void DefaultKalmanFilter::initialize()
    {
        // Initialize state to zero
        double stateData[6] = {0, 0, 0, 0, 0, 0};
        X = Matrix(6, 1, stateData);

        // Initialize covariance with reasonable uncertainty
        double covData[36] = {
            100, 0, 0, 0, 0, 0,  // px uncertainty: 10m
            0, 100, 0, 0, 0, 0,  // py uncertainty: 10m
            0, 0, 100, 0, 0, 0,  // pz uncertainty: 10m
            0, 0, 0, 25, 0, 0,   // vx uncertainty: 5 m/s
            0, 0, 0, 0, 25, 0,   // vy uncertainty: 5 m/s
            0, 0, 0, 0, 0, 25    // vz uncertainty: 5 m/s
        };
        P = Matrix(6, 6, covData);

        LOGI("DefaultKalmanFilter initialized with 6-state position/velocity model");
    }

    Matrix DefaultKalmanFilter::getF(double dt)
    {
        // State transition matrix (constant velocity model)
        // x(k+1) = F*x(k) + G*u(k)
        double data[36] = {
            1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1
        };
        return Matrix(6, 6, data);
    }

    Matrix DefaultKalmanFilter::getG(double dt)
    {
        // Control input matrix (acceleration affects velocity and position)
        double dt2 = 0.5 * dt * dt;
        double data[18] = {
            dt2, 0, 0,
            0, dt2, 0,
            0, 0, dt2,
            dt, 0, 0,
            0, dt, 0,
            0, 0, dt
        };
        return Matrix(6, 3, data);
    }

    Matrix DefaultKalmanFilter::getH()
    {
        // Full measurement matrix (we measure position directly from GPS/baro)
        double data[18] = {
            1, 0, 0, 0, 0, 0,  // px measurement
            0, 1, 0, 0, 0, 0,  // py measurement
            0, 0, 1, 0, 0, 0   // pz measurement
        };
        return Matrix(3, 6, data);
    }

    Matrix DefaultKalmanFilter::getR()
    {
        // Full measurement noise covariance
        double gpsVar = gpsNoise * gpsNoise;
        double baroVar = baroNoise * baroNoise;

        double data[9] = {
            gpsVar, 0, 0,
            0, gpsVar, 0,
            0, 0, baroVar
        };
        return Matrix(3, 3, data);
    }

    Matrix DefaultKalmanFilter::getQ(double dt)
    {
        // Process noise covariance
        // Models uncertainty in the constant-velocity assumption
        double dt2 = dt * dt;
        double dt3 = dt2 * dt / 3.0;
        double dt4 = dt2 * dt2 / 4.0;

        double q = processNoise; // Noise scaling factor

        // Continuous white noise acceleration model
        double data[36] = {
            dt4 * q, 0, 0, dt3 * q, 0, 0,
            0, dt4 * q, 0, 0, dt3 * q, 0,
            0, 0, dt4 * q, 0, 0, dt3 * q,
            dt3 * q, 0, 0, dt2 * q, 0, 0,
            0, dt3 * q, 0, 0, dt2 * q, 0,
            0, 0, dt3 * q, 0, 0, dt2 * q
        };
        return Matrix(6, 6, data);
    }

    void DefaultKalmanFilter::updateGPS(double px, double py)
    {
        // GPS horizontal position update
        double z_data[2] = {px, py};
        Matrix z(2, 1, z_data);

        // Use flexible update method
        LinearKalmanFilter::update(z, H_gps, R_gps);
    }

    void DefaultKalmanFilter::updateBaro(double pz)
    {
        // Barometer altitude update
        double z_data[1] = {pz};
        Matrix z(1, 1, z_data);

        // Use flexible update method
        LinearKalmanFilter::update(z, H_baro, R_baro);
    }

    void DefaultKalmanFilter::updateFull(double px, double py, double pz)
    {
        // Full position update (if all sensors available simultaneously)
        double z_data[3] = {px, py, pz};
        Matrix z(3, 1, z_data);

        // Use standard update method with full H and R
        LinearKalmanFilter::update(z);
    }

} // namespace astra
