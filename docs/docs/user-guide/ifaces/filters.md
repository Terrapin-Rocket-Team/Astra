# Filters

Astra uses two filter layers:

- **Orientation**: `MahonyAHRS`
- **Position/Velocity**: `LinearKalmanFilter` (usually `DefaultKalmanFilter`)

`State` consumes both.

---

## LinearKalmanFilter

`LinearKalmanFilter` is an abstract base class. You override the matrix methods:

```cpp
class LinearKalmanFilter {
public:
    virtual void initialize() = 0;
    virtual Matrix getF(double dt) = 0;
    virtual Matrix getG(double dt) = 0;
    virtual Matrix getH() = 0;
    virtual Matrix getR() = 0;
    virtual Matrix getQ(double dt) = 0;

    void predict(double dt, Matrix control);
    void update(Matrix measurement);
    void update(Matrix z, Matrix H, Matrix R);
    void updateGPS(double px, double py, double gpsNoise = -1.0);
    void updateBaro(double pz, double baroNoise = -1.0);
    void updateGPSBaro(double px, double py, double pz,
                       double gpsNoise = -1.0, double baroNoise = -1.0);
};
```

---

## DefaultKalmanFilter

`DefaultKalmanFilter` is a 6‑state position/velocity filter:

```
[px, py, pz, vx, vy, vz]
```

It expects:

- **Control**: acceleration (ENU)
- **Measurements**: GPS horizontal position + baro altitude

```cpp
DefaultKalmanFilter kf(1.0, 5.0, 2.0);
MahonyAHRS ahrs;
State state(&kf, &ahrs);
```

---

## ExtendedKalmanFilter Jacobian Helper

`ExtendedKalmanFilter` is an experimental scaffold. Its constructor, analytic
Jacobian evaluators, AD helper, and measurement-mapping helpers are implemented;
`Init()`, `Predict()`, `Update()`, and
`UpdateEKF()` are declarations awaiting implementation. It is not connected to
`State` or `DefaultState`.

### Persistent Analytic Jacobians

Derive the Jacobian formulas before building firmware, then evaluate those
formulas at the current nominal state during flight. The filter stores the
compiled evaluator and the numerical matrix separately: the evaluator stays
registered, while each successful update refreshes the existing `A` or `H` field.

`SetDynamicsJacobianEvaluator(evaluator, errorSize, context)` registers the
dynamics evaluator once. `UpdateDynamicsJacobian(nominalState)` evaluates it, and
`GetDynamicsJacobian()` returns a const reference to the saved `A`. The matching
measurement methods are `SetMeasurementJacobianEvaluator`,
`UpdateMeasurementJacobian`, and `GetMeasurementJacobian`; their matrix has
`measurementSize` rows. These methods only maintain Jacobians; the scaffold's
prediction and correction methods remain unimplemented.

The evaluator has this signature and uses ordinary `double` arithmetic:

```cpp
numx_status_t evaluator(const Eigen::VectorXd& nominalState,
                        Eigen::MatrixXd& jacobian, void* context);
```

It fills every entry of the supplied matrix, including zeros, without changing
its size. Return `NUMX_OK` on success or a numx error code on failure. The filter
calls it once per update, validates the result, and preserves the last successful
matrix if evaluation fails. Scratch storage is reused when dimensions stay the
same. Registration initializes the saved matrix with NaNs; use it only after a
successful update. Check the returned status on every call.

The constructor's `stateSize` is the **nominal input size**. The separately
configured `errorSize` is the covariance coordinate count: navigation uses 11
nominal components and 10 error components. Keep the constructor dimension fields
unchanged while evaluators are registered. Dynamics evaluators must directly
return the continuous **error-dynamics** Jacobian; differentiating four independent
quaternion components does not produce the required attitude-error dynamics.
Measurement evaluators must likewise return `H` in error coordinates. Derive
those error-coordinate models before generating their evaluators.

Context pointers are borrowed and must stay valid while registered. Update their
time, controls, atmosphere, table slopes, and other changing parameters before
evaluation. Treating a state-dependent lookup value as an independent constant
omits its state derivative; include the interpolation law or its derivative in
the model when that sensitivity is required.

For a checked evaluation into a caller-owned matrix, use
`EvaluateJacobian(evaluator, input, outputSize, errorSize, J, context)`.

#### Derive Formulas Offline

The optional host tool `tools/generate_ekf_jacobian.py` separates symbolic
derivation from evaluation. `derive_jacobian(model, state_symbols)` computes the
symbolic derivatives with SymPy; `generate_jacobian_header(...)` writes their C++
evaluator. Symbolic algebra runs when regenerating the header, and the compiled
firmware evaluates its formulas using plain numbers.

From the repository root:

```sh
python -m pip install -r tools/requirements-jacobian.txt
```

Import `derive_jacobian` and `generate_jacobian_header` from
`tools.generate_ekf_jacobian` in your host-side model script. Supply symbolic
equations and their independent variables to `derive_jacobian`, then pass the
resulting matrix and its nominal input symbols to `generate_jacobian_header`.
Write the returned header text to your application's source tree and compile it
into firmware. Its generated callback can be registered once and evaluated
throughout flight:

```cpp
// During configuration, using your generated evaluator and parameter context:
if (filter.SetDynamicsJacobianEvaluator(dynamicsJacobian, 10, &physics) != NUMX_OK)
    return;

// During propagation, using the current 11-component nominal state:
if (filter.UpdateDynamicsJacobian(nominalState) != NUMX_OK) return;
const Eigen::MatrixXd& A = filter.GetDynamicsJacobian();
```

Keep the configured filter and its context alive throughout use. Regenerate and
rebuild the header when the equations change. SymPy is an optional development
dependency; firmware builds from generated headers do not need it. The full
flight dynamics, attitude-error model, and sensor models still need implementation.

### Automatic Differentiation Helper

The existing AD path remains available for prototyping and checking analytic
evaluators against their original models.

`CalculateJacobian()` uses numx forward-mode automatic differentiation to return
an output-by-input matrix, with entry `J(i,j)` equal to the partial derivative
of output `i` with respect to input `j`. The callback receives dual numbers,
fills every element of the supplied output vector, and returns `NUMX_OK` or an
error code. Use `numx_dual_*` operations inside the callback so derivatives pass
through the calculation; ordinary `double` arithmetic or lookup values alone do
not carry derivatives.

```cpp
#include <Filters/ExtendedKalmanFilter.h>

using EKF = astra::ExtendedKalmanFilter;

numx_status_t model(const EKF::DualVector& x, EKF::DualVector& y, void* context) {
    (void)context;
    y(0) = numx_dual_mul(x(0), x(1));
    y(1) = numx_dual_sin(x(0));
    return NUMX_OK;
}

void example() {
    EKF filter(2, 0, 2);
    Eigen::VectorXd point(2);
    point << 0.5, 3.0;
    Eigen::MatrixXd J;
    const numx_status_t status = filter.CalculateJacobian(model, point, 2, J);
    if (status != NUMX_OK) return;
    // J is approximately [[3, 0.5], [cos(0.5), 0]].
}
```

The optional `context` pointer can hold time, controls, and model parameters that
remain fixed while differentiating. The helper evaluates the callback once per
input component and reuses its scratch buffers when dimensions stay unchanged.
It rejects invalid sizes, nonfinite inputs/results, and callbacks that change the
output size or leave outputs unwritten. Errors leave the caller's matrix unchanged.

Calculations use `numx_real_t` (`float` by default); storing the Jacobian in
`Eigen::MatrixXd` does not increase derivative precision. For quaternion attitude,
the callback must express the model in the chosen attitude-error coordinates
before its Jacobian can be used with the covariance. Lookup-table interpolation
must also carry the appropriate derivatives; converting a dual input to a plain
number discards that derivative path.

### Sensor Measurement Mappings

The navigation helpers use nominal state `[p_ENU(3), v_ENU(3), q_wxyz(4), c]`
and error state `[dp_ENU(3), dv_ENU(3), dtheta_body(3), dc]`. Attitude errors
are right-local: `q_true = q_est * Exp(dtheta_body)`. Their measurement
Jacobians have **10 columns**, matching the error covariance.

| Sensor data | Measurement mapping |
| --- | --- |
| Relative barometric altitude, metres | `BarometerJacobian()` returns a 1-by-10 H selecting Up position (column 2). |
| GPS East/North position, metres | `GPSHorizontalJacobian()` returns a 2-by-10 H selecting columns 0 and 1, matching current `State` fusion. Convert latitude/longitude first. |
| Body-frame accelerometer specific force | A nonlinear measurement model is required; the proposed free-flight model predicts body-frame non-gravitational acceleration from the forces. Use the navigation Jacobian helper with that model. |
| Body-frame gyro angular velocity | Prediction input; there is no gyro measurement selector for this state, which contains no angular-velocity state. |
| Magnetometer | Currently used by the optional Mahony path. A future direct EKF vector update needs a calibrated body-vector model and a navigation-frame magnetic-field reference. |

```cpp
auto H_baro = astra::ExtendedKalmanFilter::BarometerJacobian();
auto H_gps = astra::ExtendedKalmanFilter::GPSHorizontalJacobian();
```

For nonlinear sensor models, call
`CalculateNavigationMeasurementJacobian(model, nominalState, outputSize, H, context)`.
The callback evaluates a noise-free sensor prediction on the **11-component
nominal state**, using numx dual operations. The helper differentiates it and
maps the quaternion's four nominal components onto the three attitude-error
components, returning an `outputSize`-by-10 H. The nominal quaternion must be
unit length. Invalid states or callback failures leave H unchanged.

This conversion is for measurement Jacobians; continuous error-dynamics
Jacobians require their own derivation. These helpers do not initialize or wire
the filter into `State`, enable new sensor fusion, or choose numerical Q/R values.

---

## MahonyAHRS

`MahonyAHRS` estimates orientation from accel + gyro (and optional mag).  
It outputs a quaternion and earth-frame acceleration.

```cpp
MahonyAHRS ahrs(0.1, 0.0005);
```

If magnetometer calibration is performed, yaw stabilization improves.

---

## Recommended Path

Most projects should start with:

```cpp
DefaultState state;
```

This wires `DefaultKalmanFilter` + `MahonyAHRS` automatically.

