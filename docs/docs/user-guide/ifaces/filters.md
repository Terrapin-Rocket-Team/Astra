# Filters

The `Filter` interface provides a standardized way to implement state estimation algorithms like Kalman filters for sensor fusion. Filters can be passed to the `State` object to improve position, velocity, and acceleration estimates by combining data from multiple sensors.

---

## Overview

The `Filter` interface is a pure virtual class that defines the contract for state estimation algorithms. It provides:

1. **Initialization** - Set up filter parameters and initial state
2. **Iteration** - Process new measurements and update state estimate
3. **State dimensionality** - Query filter configuration

Implementing a filter requires defining these core methods to integrate with Astra's sensor fusion system.

---

## Interface Definition

```cpp
class Filter {
public:
    virtual ~Filter() {}

    // Core interface
    virtual void initialize() = 0;
    virtual double* iterate(double dt, double* state,
                          double* measurements, double* controlVars) = 0;

    // Query filter dimensions
    virtual int getMeasurementSize() const = 0;
    virtual int getInputSize() const = 0;
    virtual int getStateSize() const = 0;
};
```

---

## Required Methods

### initialize()

```cpp
virtual void initialize() = 0;
```

Called once during setup to initialize filter matrices and parameters. Use this to:
- Set initial state covariance
- Configure process noise
- Configure measurement noise
- Allocate any required memory

### iterate()

```cpp
virtual double* iterate(double dt, double* state,
                       double* measurements, double* controlVars) = 0;
```

Called every update cycle to process new sensor data and update the state estimate.

**Parameters:**
- `dt`: Time step since last update (seconds)
- `state`: Current state vector
- `measurements`: New sensor measurements
- `controlVars`: Control inputs (optional, can be nullptr)

**Returns:** Pointer to updated state estimate

### Dimension Queries

```cpp
virtual int getMeasurementSize() const = 0;   // Number of measurements
virtual int getInputSize() const = 0;          // Number of control inputs
virtual int getStateSize() const = 0;          // Number of state variables
```

These methods tell the `State` object how to allocate arrays and pass data correctly.

---

## Integration with State

Filters are passed to the `State` constructor:

```cpp
MyKalmanFilter filter;
State vehicleState(sensors, numSensors, &filter);
```

During `State::update()`:
1. Sensors are read
2. Measurement vector is assembled from sensor data
3. `filter->iterate()` is called with measurements
4. Updated state is used for position, velocity, acceleration

---

## Implementing a Custom Filter

### Basic Kalman Filter Structure

```cpp
#ifndef MY_KALMAN_FILTER_H
#define MY_KALMAN_FILTER_H

#include "Filters/Filter.h"
#include "Math/Matrix.h"
#include "Math/Vector.h"

class MyKalmanFilter : public Filter {
public:
    MyKalmanFilter();
    ~MyKalmanFilter();

    void initialize() override;
    double* iterate(double dt, double* state,
                   double* measurements, double* controlVars) override;

    int getMeasurementSize() const override { return 3; }  // x, y, z
    int getInputSize() const override { return 0; }        // No control
    int getStateSize() const override { return 9; }        // [pos, vel, accel]

private:
    // State vector: [px, py, pz, vx, vy, vz, ax, ay, az]
    Vector<9> x;

    // Covariance matrix
    Matrix<9, 9> P;

    // Process noise covariance
    Matrix<9, 9> Q;

    // Measurement noise covariance
    Matrix<3, 3> R;

    void predict(double dt);
    void update(const Vector<3>& z);
};

#endif
```

### Implementation Example

```cpp
#include "MyKalmanFilter.h"

MyKalmanFilter::MyKalmanFilter() {
    // Initialize state to zero
    x = Vector<9>::zero();

    // Initialize covariances
    P = Matrix<9, 9>::identity() * 1.0;
    Q = Matrix<9, 9>::identity() * 0.01;
    R = Matrix<3, 3>::identity() * 0.1;
}

void MyKalmanFilter::initialize() {
    // Reset filter state
    x = Vector<9>::zero();
    P = Matrix<9, 9>::identity() * 1.0;

    LOGI("Kalman filter initialized");
}

double* MyKalmanFilter::iterate(double dt, double* state,
                                double* measurements, double* controlVars) {
    // Prediction step
    predict(dt);

    // Update step with measurements
    Vector<3> z(measurements[0], measurements[1], measurements[2]);
    update(z);

    // Return updated state
    for (int i = 0; i < 9; i++) {
        state[i] = x[i];
    }

    return state;
}

void MyKalmanFilter::predict(double dt) {
    // State transition: x_k = F * x_{k-1}
    // F models constant acceleration dynamics
    Matrix<9, 9> F = Matrix<9, 9>::identity();

    // Position += velocity * dt + 0.5 * acceleration * dt^2
    F(0, 3) = dt; F(0, 6) = 0.5 * dt * dt;
    F(1, 4) = dt; F(1, 7) = 0.5 * dt * dt;
    F(2, 5) = dt; F(2, 8) = 0.5 * dt * dt;

    // Velocity += acceleration * dt
    F(3, 6) = dt;
    F(4, 7) = dt;
    F(5, 8) = dt;

    // Update state
    x = F * x;

    // Update covariance
    P = F * P * F.transpose() + Q;
}

void MyKalmanFilter::update(const Vector<3>& z) {
    // Measurement model: z = H * x
    // We measure position directly
    Matrix<3, 9> H = Matrix<3, 9>::zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;

    // Innovation
    Vector<3> y = z - (H * x);

    // Innovation covariance
    Matrix<3, 3> S = H * P * H.transpose() + R;

    // Kalman gain
    Matrix<9, 3> K = P * H.transpose() * S.inverse();

    // Update state
    x = x + K * y;

    // Update covariance
    Matrix<9, 9> I = Matrix<9, 9>::identity();
    P = (I - K * H) * P;
}
```

---

## Example: 6DOF Kalman Filter

Astra includes a 6DOF (degrees of freedom) Kalman filter example in the `examples/Avionics` folder. This filter estimates:

- Position (3D)
- Velocity (3D)
- Acceleration (3D)

From:
- GPS position measurements
- Barometer altitude
- IMU acceleration

---

## Usage Example

```cpp
#include "MyKalmanFilter.h"
#include "State/State.h"

// Create sensors
MAX_M10S gps;
DPS368 baro;
BMI088andLIS3MDL imu;
Sensor *sensors[] = {&gps, &baro, &imu};

// Create filter
MyKalmanFilter filter;

// Pass to State
State vehicleState(sensors, 3, &filter);

AstraConfig config = AstraConfig()
                        .withState(&vehicleState)
                        .withUpdateRate(50);  // 50Hz for good filtering

Astra system(&config);

void loop() {
    system.update();

    // Filtered state is available from State
    Vector<3> pos = vehicleState.getPosition();
    Vector<3> vel = vehicleState.getVelocity();

    // Position and velocity are now filtered estimates
}
```

---

## Filter Design Considerations

### State Vector Design

Choose state variables based on what you need to estimate:
- **Minimal**: Position, velocity
- **Standard**: Position, velocity, acceleration
- **Extended**: Add orientation, angular rates, biases

### Process Noise (Q)

Models uncertainty in your dynamics model:
- **High Q**: Trust measurements more, faster response
- **Low Q**: Trust model more, smoother output
- Tune based on vehicle dynamics

### Measurement Noise (R)

Models sensor uncertainty:
- Use sensor datasheets for initial values
- **High R**: Trust measurements less
- **Low R**: Trust measurements more
- Can be adaptive based on sensor status

### Update Rate

- **Higher rates** (50-100Hz): Better tracking, more computation
- **Lower rates** (10-20Hz): Adequate for most applications
- Match or exceed sensor update rates

---

## Best Practices

1. **Initialize properly**: Set realistic initial covariance

2. **Tune systematically**: Start conservative, then optimize
   ```cpp
   // Conservative initial values
   Q = Matrix<9, 9>::identity() * 0.1;  // Low process noise
   R = Matrix<3, 3>::identity() * 1.0;  // High measurement noise
   ```

3. **Handle missing data**: Check sensor validity before measurements
   ```cpp
   if (!gps.getHasFix()) {
       // Skip GPS measurements or use higher R
   }
   ```

4. **Monitor filter health**: Check covariance growth
   ```cpp
   if (P.trace() > threshold) {
       LOGW("Filter covariance growing - reinitialize?");
   }
   ```

5. **Use appropriate math libraries**: Astra provides `Vector` and `Matrix` classes

---

## Common Filter Types

### Kalman Filter
- Linear systems
- Gaussian noise
- Optimal for linear dynamics

### Extended Kalman Filter (EKF)
- Nonlinear systems
- Linearizes around current estimate
- Common for IMU orientation

### Unscented Kalman Filter (UKF)
- Nonlinear systems
- Better nonlinear handling than EKF
- More computationally expensive

### Complementary Filter
- Simple sensor fusion
- No matrix operations
- Good for IMU orientation

---

## Limitations

!!! warning "Current Implementation"
    The filter system is currently evolving. The interface is stable, but:
    - No built-in filters provided yet
    - Matrix operations may be dense/unoptimized
    - See examples for current best practices

---

## Summary

- `Filter` provides interface for state estimation algorithms
- Implement `initialize()` and `iterate()` for custom filters
- Pass filter to `State` constructor for sensor fusion
- Tune process and measurement noise for your application
- Higher update rates improve filter performance

For integration examples, see `examples/Avionics`. For sensor details, see the [Sensor Interface](sensor.md) and [State](state.md) documentation.

---
