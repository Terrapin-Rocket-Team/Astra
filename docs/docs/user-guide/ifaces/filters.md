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

`ExtendedKalmanFilter` is an experimental scaffold. Its constructor, Jacobian
helper, and measurement-mapping helpers are implemented; `Init()`, `Predict()`, `Update()`, and
`UpdateEKF()` are declarations awaiting implementation. It is not connected to
`State` or `DefaultState`.

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

