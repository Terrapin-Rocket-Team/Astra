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
    using JacobianEvaluator = numx_status_t (*)(const Eigen::VectorXd &nominalState,
                                              Eigen::MatrixXd &jacobian,
                                              void *context);

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

    //function to evaluate teh jacobian numerically at that specific
    //nominal state that we currently are at
    numx_status_t EvaluateJacobian(JacobianEvaluator evaluator,
                                  const Eigen::VectorXd &nominalState,
                                  int outputSize, int errorSize,
                                  Eigen::MatrixXd &jacobian,
                                  void *context = nullptr);

    /**
     * Store an evaluator for the continuous error-dynamics Jacobian A.
     * A has errorSize rows and columns; stateSize is the nominal state size.
     * The callback and borrowed context must remain valid while configured.
     * Configuration does not run the callback; A contains NaNs until updated.
     * Invalid configuration leaves the evaluator and A unchanged.
     */
    numx_status_t SetDynamicsJacobianEvaluator(JacobianEvaluator evaluator,
                                             int errorSize,
                                             void *context = nullptr);

    /**
     * Store an evaluator for the measurement Jacobian H in error coordinates.
     * H has measurementSize rows and errorSize columns. Context is borrowed.
     * Configuration does not run the callback; H contains NaNs until updated.
     * Invalid configuration leaves the evaluator and H unchanged.
     */
    numx_status_t SetMeasurementJacobianEvaluator(JacobianEvaluator evaluator,
                                                int errorSize,
                                                void *context = nullptr);

    /** Update persistent A from a stateSize-component nominal state. */
    numx_status_t UpdateDynamicsJacobian(const Eigen::VectorXd &nominalState);

    /** Update persistent H from a stateSize-component nominal state. */
    numx_status_t UpdateMeasurementJacobian(const Eigen::VectorXd &nominalState);

    /** Latest evaluated matrix; check the update status before using it. */
    const Eigen::MatrixXd &GetDynamicsJacobian() const { return A; }
    const Eigen::MatrixXd &GetMeasurementJacobian() const { return H; }

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

    /**
     * Mappings for the 11-component [p_ENU, v_ENU, q_wxyz, c] nominal state.
     * Error order: [dp_ENU(3), dv_ENU(3), dtheta_body(3), dc].
     * Use right-local attitude errors: q_true = q_est * Exp(dtheta_body).
     * These helpers do not resize or initialize the filter state/covariance.
     */
    /** Relative barometric altitude in metres: H selects the Up position error. */
    static Eigen::Matrix<double, 1, 10> BarometerJacobian();

    /** Local GPS [East, North] position in metres, matching State's current use. */
    static Eigen::Matrix<double, 2, 10> GPSHorizontalJacobian();

    /**
     * Differentiate a sensor model h(x) on the 11-component nominal state and
     * convert its Jacobian to the 10-component right-local error coordinates.
     * The callback must use numx dual operations, as in CalculateJacobian.
     * nominalState's quaternion (indices 6..9, w first) must have unit length.
     * Returns an outputSize-by-10 H; H is unchanged on failure.
     */
    numx_status_t CalculateNavigationMeasurementJacobian(
        JacobianFunction measurementModel, const Eigen::VectorXd &nominalState,
        int outputSize, Eigen::MatrixXd &measurementJacobian, void *context = nullptr);

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
    static numx_status_t EvaluateJacobianWithScratch(
        JacobianEvaluator evaluator, const Eigen::VectorXd &nominalState,
        int outputSize, int errorSize, Eigen::MatrixXd &jacobian,
        void *context, Eigen::MatrixXd &scratch);

    JacobianEvaluator dynamicsJacobianEvaluator = nullptr; //symbolic jacobian of A
    JacobianEvaluator measurementJacobianEvaluator = nullptr; //symbolic jacobian of H
    void *dynamicsJacobianContext = nullptr;
    void *measurementJacobianContext = nullptr;
    int dynamicsErrorSize = 0;
    int measurementErrorSize = 0;
    Eigen::MatrixXd evaluatedJacobian;
    Eigen::MatrixXd dynamicsJacobianCandidate;
    Eigen::MatrixXd measurementJacobianCandidate;
    DualVector jacobianInput;
    DualVector jacobianOutput;
    Eigen::MatrixXd jacobianResult;
    Eigen::MatrixXd nominalMeasurementJacobian;
    Eigen::MatrixXd errorMeasurementJacobian;
};

} // namespace astra

#endif // EXTENDED_KALMAN_FILTER_H
