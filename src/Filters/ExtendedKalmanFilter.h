//Created by Divyansh Srivastava on 11/11/2026
#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H
#include <ArduinoEigen.h>

extern "C" { //needed since numx is a C library
#include <numx/autodiff.h>
}

namespace astra {

class ExtendedKalmanFilter {
public:
    using DualVector = Eigen::Matrix<numx_dual_t, Eigen::Dynamic, 1>;
    using JacobianFunction = numx_status_t (*)(const DualVector &input,
                                             DualVector &output, void *context);

    int measurementSize;
    int controlSize;
    int stateSize;

    ExtendedKalmanFilter(int measurementSize, int controlSize, int stateSize);
    virtual ~ExtendedKalmanFilter() = default;

    /**
     * Initialize the state, covariance, Jacobians, and noise covariances.
     * @param A_in Continuous-time dynamics Jacobian.
     * @param Q_in Continuous-time process-noise intensity Q_c.
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &A_in,
              Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

    /** Predict the state and state covariance using the process model. */
    void Predict();

    /**
     * Update the state using a linear measurement model.
     * @param z The new measurement.
     */
    void Update(const Eigen::VectorXd &z);

    /**
     * Update the state using a nonlinear measurement model.
     * @param z The new measurement.
     */
    void UpdateEKF(const Eigen::VectorXd &z);

    /**
     * Calculate J(i,j) = d function_i / d input_j using numx forward-mode AD.
     * The callback fills outputSize dual results using numx_dual_* operations.
     * context can hold inputs/time that stay fixed during differentiation.
     * Uses numx_real_t precision (float by default), even though J stores doubles.
     * Returns NUMX_OK, a callback error, NUMX_ERR_NULL_PTR for a null callback,
     * or NUMX_ERR_INVALID_ARG for invalid sizes/nonfinite values. J is unchanged
     * on failure. Scratch buffers are reused when dimensions stay unchanged.
     * Coordinates are defined by the callback; quaternion errors need their own
     * error-coordinate model, rather than four independent quaternion entries.
     */
    numx_status_t CalculateJacobian(JacobianFunction function,
                                   const Eigen::VectorXd &input, int outputSize,
                                   Eigen::MatrixXd &jacobian, void *context = nullptr);

protected:
    // Sized and initialized by the model; nominal and error-state sizes may differ.
    Eigen::VectorXd X; // Nominal state vector
    Eigen::MatrixXd P; // State-error covariance
    Eigen::MatrixXd Q; // Continuous-time process-noise intensity Q_c
    Eigen::MatrixXd R; // Measurement-noise covariance
    Eigen::MatrixXd A; // Continuous dynamics Jacobian in error coordinates
    Eigen::MatrixXd H; // Measurement Jacobian in error coordinates
    Eigen::MatrixXd L; // Process-noise mapping
    Eigen::MatrixXd M; // Measurement-noise mapping
    Eigen::MatrixXd K; // Kalman gain

private:
    DualVector jacobianInput;
    DualVector jacobianOutput;
    Eigen::MatrixXd jacobianResult;
};

} // namespace astra

#endif // EXTENDED_KALMAN_FILTER_H
