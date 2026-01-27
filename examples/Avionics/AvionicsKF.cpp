#include "AvionicsKF.h"

namespace astra {

// Define the measurement size, control size, and state size
AvionicsKF::AvionicsKF() : LinearKalmanFilter(3, 3, 6) {}

Matrix AvionicsKF::getF(double dt) {
    double *data = new double[36]{
        1.0, 0, 0, dt, 0, 0,
        0, 1.0, 0, 0, dt, 0,
        0, 0, 1.0, 0, 0, dt,
        0, 0, 0, 1.0, 0, 0,
        0, 0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 0, 1.0
    };
    return Matrix(6, 6, data);
}

Matrix AvionicsKF::getG(double dt) {
    double *data = new double[18]{
        0.5 * dt * dt, 0, 0,
        0, 0.5 * dt * dt, 0,
        0, 0, 0.5 * dt * dt,
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt
    };
    return Matrix(6, 3, data);
}

Matrix AvionicsKF::getH() {
    double *data = new double[18]{
        1.0, 0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0
    };
    return Matrix(3, 6, data);
}

Matrix AvionicsKF::getR() {
    // Measurement noise covariance - how much we trust sensors
    // Lower values = trust sensors more (good for simulation with low/no noise)
    // Higher values = trust sensors less (good for real hardware with noisy sensors)

    // For simulation: use smaller values since sim sensors are typically accurate
    double r_gps_xy = 0.25;  // GPS horizontal: ~0.5m std dev (good GPS)
    double r_alt = 0.1;      // Barometric altitude: ~0.3m std dev (good baro)

    double *data = new double[9]{
        r_gps_xy, 0, 0,
        0, r_gps_xy, 0,
        0, 0, r_alt
    };
    return Matrix(3, 3, data);
}

Matrix AvionicsKF::getQ(double dt) {
    // Process noise covariance - accounts for model uncertainty
    // Higher values = less trust in the constant-acceleration model
    // For rockets: acceleration changes rapidly (thrust curves, drag, gravity turn)

    // Position process noise: small (model is good for position prediction)
    double q_pos = 0.01;

    // Velocity process noise: moderate (velocity can change due to drag, thrust variations)
    double q_vel = 1.0;

    double *data = new double[36]{
        q_pos, 0, 0, 0, 0, 0,
        0, q_pos, 0, 0, 0, 0,
        0, 0, q_pos, 0, 0, 0,
        0, 0, 0, q_vel, 0, 0,
        0, 0, 0, 0, q_vel, 0,
        0, 0, 0, 0, 0, q_vel
    };
    return Matrix(6, 6, data);
}

} // namespace astra
