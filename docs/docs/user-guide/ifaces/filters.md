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

