# Math

Astra provides three mathematical classes for vector and matrix operations: `Vector`, `Matrix`, and `Quaternion`. These are used throughout the library for sensor data representation, coordinate transformations, and state estimation.

---

## Vector

The `Vector` class represents a fixed-size mathematical vector (not to be confused with `std::vector`). It supports standard vector operations and is templated on size.

### Overview

`Vector<N>` stores `N` double values and provides operations like addition, subtraction, dot products, cross products, and normalization. Adapted from Adafruit's IMU vector class.

**Common sizes:**
- `Vector<2>` - 2D coordinates (latitude/longitude, horizontal position)
- `Vector<3>` - 3D coordinates (position, velocity, acceleration, orientation)
- `Vector<4>` - Quaternion components, homogeneous coordinates

### Basic Usage

```cpp
#include <Math/Vector.h>

using namespace astra;

// Create vectors
Vector<3> v1(1.0, 2.0, 3.0);
Vector<3> v2(4.0, 5.0, 6.0);

// Access components
double x = v1.x();  // 1.0
double y = v1.y();  // 2.0
double z = v1.z();  // 3.0

// Array-style access
double val = v1[0];  // 1.0 (same as x())
v1[1] = 7.0;         // Set y to 7.0
```

### Vector Operations

**Arithmetic:**

```cpp
Vector<3> a(1.0, 2.0, 3.0);
Vector<3> b(4.0, 5.0, 6.0);

Vector<3> sum = a + b;           // (5.0, 7.0, 9.0)
Vector<3> diff = a - b;          // (-3.0, -3.0, -3.0)
Vector<3> scaled = a * 2.0;      // (2.0, 4.0, 6.0)
Vector<3> divided = a / 2.0;     // (0.5, 1.0, 1.5)

// In-place operations
a += b;  // a becomes (5.0, 7.0, 9.0)
```

**Vector Products:**

```cpp
Vector<3> a(1.0, 0.0, 0.0);
Vector<3> b(0.0, 1.0, 0.0);

// Dot product (scalar result)
double dot = a.dot(b);           // 0.0

// Cross product (3D only, vector result)
Vector<3> cross = a.cross(b);    // (0.0, 0.0, 1.0)
```

**Magnitude and Normalization:**

```cpp
Vector<3> v(3.0, 4.0, 0.0);

double mag = v.magnitude();      // 5.0

v.normalize();                   // v becomes (0.6, 0.8, 0.0)
double newMag = v.magnitude();   // 1.0
```

**Utility Methods:**

```cpp
Vector<3> v(1.0, 2.0, 3.0);

Vector<3> inverted = v.invert(); // (-1.0, -2.0, -3.0)
Vector<3> scaled = v.scale(2.5); // (2.5, 5.0, 7.5)

// Angle conversions
v.toRadians();                   // Convert degrees to radians
v.toDegrees();                   // Convert radians to degrees

// Size query
uint8_t size = v.n();            // 3
```

### Practical Examples

**Position displacement:**

```cpp
Vector<3> startPos = state.getPosition();   // capture at t0
Vector<3> currentPos = state.getPosition(); // later in time

Vector<3> displacement = currentPos - startPos;
double distance = displacement.magnitude();

LOGI("Traveled %.2f meters", distance);
```

**Velocity from acceleration:**

```cpp
Vector<3> velocity(0, 0, 0);
Vector<3> acceleration = state.getAcceleration(); // ENU linear accel
double dt = 0.01;  // 10ms

velocity += acceleration * dt;
```

**Unit direction vectors:**

```cpp
Vector<3> direction(1.0, 1.0, 0.0);
direction.normalize();  // (0.707, 0.707, 0.0)

// Use as unit direction
Vector<3> target = position + direction * 10.0;  // 10 meters in that direction
```

**Angle between vectors:**

```cpp
Vector<3> v1(1.0, 0.0, 0.0);
Vector<3> v2(0.0, 1.0, 0.0);

double cosAngle = v1.dot(v2) / (v1.magnitude() * v2.magnitude());
double angleRad = acos(cosAngle);
double angleDeg = angleRad * 57.2957795;  // 90 degrees
```

---

## Matrix

The `Matrix` class provides dense matrix operations for Kalman filters and linear algebra. It supports basic operations like addition, subtraction, multiplication, transposition, and inversion.

### Overview

Matrix operations use dynamically allocated arrays. Matrices are passed ownership of their data arrays (shallow copy), so arrays must be heap-allocated and should not be modified or deleted after passing to the constructor.

### Basic Usage

```cpp
#include <Math/Matrix.h>

using namespace astra;

// Create 3x3 identity matrix
double* data = new double[9]{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
};

Matrix m(3, 3, data);

// Access elements (row, col)
double val = m(0, 0);    // 1.0
m(1, 2) = 5.0;           // Set element at row 1, col 2

// Query dimensions
uint8_t rows = m.getRows();  // 3
uint8_t cols = m.getCols();  // 3
```

!!! warning "Memory Management"
    The `Matrix` class takes **ownership** of the array passed to its constructor:
    - The array **must** be heap-allocated (`new double[]`)
    - Do **not** modify or delete the array after passing it to `Matrix`
    - The array is automatically deleted when the matrix is destroyed
    - Shallow copy only—matrix operations create new arrays as needed

### Matrix Operations

**Arithmetic:**

```cpp
Matrix A(3, 3, dataA);
Matrix B(3, 3, dataB);

Matrix C = A + B;        // Addition
Matrix D = A - B;        // Subtraction
Matrix E = A * B;        // Matrix multiplication
Matrix F = A * 2.0;      // Scalar multiplication
```

**Transpose and Inverse:**

```cpp
Matrix A(3, 3, data);

Matrix AT = A.T();           // Transpose
Matrix AT_alt = A.transpose(); // Same as T()

Matrix Ainv = A.inv();       // Inverse
Matrix Ainv_alt = A.inverse(); // Same as inv()
```

**Special Matrices:**

```cpp
// Identity matrix
Matrix I = Matrix::ident(3);  // 3x3 identity

// Trace (sum of diagonal elements)
double tr = A.trace();
```

**Display:**

```cpp
Matrix A(3, 3, data);
A.disp();  // Print to serial/console
```

### Practical Examples

**Kalman Filter State Transition:**

```cpp
// State transition matrix (constant velocity model)
double* F_data = new double[9]{
    1.0, dt, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
};
Matrix F(3, 3, F_data);

// Previous state
double* x_data = new double[3]{position, velocity, 1.0};
Matrix x(3, 1, x_data);

// Predict new state
Matrix x_pred = F * x;
```

**Covariance Update:**

```cpp
Matrix P = covariance;
Matrix Q = processNoise;

// Predict covariance: P' = F*P*F^T + Q
Matrix P_pred = F * P * F.transpose() + Q;
```

**Measurement Update:**

```cpp
// Kalman gain: K = P*H^T * (H*P*H^T + R)^-1
Matrix K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

// State update: x = x + K*y
Matrix x_new = x + K * innovation;

// Covariance update: P = (I - K*H) * P
Matrix I = Matrix::ident(stateSize);
Matrix P_new = (I - K * H) * P;
```

---

## Quaternion

The `Quaternion` class represents 3D rotations using quaternion algebra. It's more stable than Euler angles and avoids gimbal lock. Adapted from Adafruit's BNO055 library.

### Overview

Quaternions store rotation as four components: `w` (scalar) and `x, y, z` (vector). They provide efficient composition, interpolation, and conversion to/from other rotation representations.

**Common uses:**
- IMU orientation representation
- Coordinate frame transformations
- Smooth rotation interpolation

### Basic Usage

```cpp
#include <Math/Quaternion.h>

using namespace astra;

// Identity quaternion (no rotation)
Quaternion q;  // (w=1, x=0, y=0, z=0)

// Create from components
Quaternion q1(1.0, 0.0, 0.0, 0.0);

// Create from scalar + vector
Vector<3> axis(0.0, 0.0, 1.0);
Quaternion q2(0.707, axis * 0.707);

// Access components
double w = q.w();
double x = q.x();
double y = q.y();
double z = q.z();
```

### Quaternion Operations

**Basic Operations:**

```cpp
Quaternion q1(1.0, 0.0, 0.0, 0.0);
Quaternion q2(0.707, 0.0, 0.0, 0.707);

// Quaternion multiplication (rotation composition)
Quaternion q3 = q1 * q2;

// Addition/subtraction
Quaternion sum = q1 + q2;
Quaternion diff = q1 - q2;

// Scalar operations
Quaternion scaled = q1 * 2.0;
Quaternion divided = q1 / 2.0;

// Conjugate (inverse rotation for unit quaternions)
Quaternion conj = q.conjugate();
```

**Normalization:**

```cpp
Quaternion q(1.0, 2.0, 3.0, 4.0);

double mag = q.magnitude();  // sqrt(1^2 + 2^2 + 3^2 + 4^2) = 5.477
q.normalize();               // Unit quaternion
```

### Rotation Representations

**From Axis-Angle:**

```cpp
Quaternion q;
Vector<3> axis(0.0, 0.0, 1.0);  // Z-axis
double angle = M_PI / 2;         // 90 degrees

q.fromAxisAngle(axis, angle);
```

**To Axis-Angle:**

```cpp
Vector<3> axis;
double angle;

q.toAxisAngle(axis, angle);
LOGI("Rotation: %.2f rad about (%.2f, %.2f, %.2f)",
     angle, axis.x(), axis.y(), axis.z());
```

**From Rotation Matrix:**

```cpp
Matrix R(3, 3, rotationMatrixData);
Quaternion q;
q.fromMatrix(R);
```

**To Rotation Matrix:**

```cpp
Matrix R = q.toMatrix();  // 3x3 Direction Cosine Matrix
```

**To Euler Angles (Yaw-Pitch-Roll):**

```cpp
Vector<3> euler = q.toEuler321();

double yaw = euler[0];    // Rotation about Z
double pitch = euler[1];  // Rotation about Y
double roll = euler[2];   // Rotation about X

// Convert to degrees
euler.toDegrees();
LOGI("Orientation: yaw=%.1f, pitch=%.1f, roll=%.1f",
     euler[0], euler[1], euler[2]);
```

### Rotating Vectors

Apply rotation to a vector:

```cpp
Quaternion orientation = state.getOrientation();
Vector<3> localAccel(0.0, 0.0, 9.81);  // Gravity in body frame

// Rotate to global frame
Vector<3> globalAccel = orientation.rotateVector(localAccel);
```

### Interpolation

Smoothly interpolate between orientations:

```cpp
Quaternion current = state.getOrientation();
Quaternion identity(1.0, 0.0, 0.0, 0.0);

double alpha = 0.9;  // Weighting (0 = identity, 1 = current)

// SLERP/LERP interpolation
Quaternion smoothed = current.interpolation(identity, alpha);
```

**Parameters:**
- `q` - Target quaternion to interpolate toward
- `alpha` - Weight (0-1): higher = closer to current quaternion
- `epsilon` - Threshold (default 0.9): below uses SLERP, above uses LERP

### Practical Examples

**IMU Orientation:**

```cpp
Quaternion orientation = state.getOrientation();
Vector<3> euler = orientation.toEuler321();
euler.toDegrees();

LOGI("Yaw: %.1f, Pitch: %.1f, Roll: %.1f",
     euler.x(), euler.y(), euler.z());
```

**Transform Sensor Readings to Global Frame:**

```cpp
// Acceleration in body frame
Vector<3> accelBody = imu.getAccel();  // body-frame accel (m/s^2)

// Orientation of body relative to global
Quaternion orientation = state.getOrientation();

// Transform to global frame
Vector<3> accelGlobal = orientation.rotateVector(accelBody);
```

**Composing Rotations:**

```cpp
// Rotate 90° about Z, then 45° about X
Quaternion rotZ, rotX;
rotZ.fromAxisAngle(Vector<3>(0, 0, 1), M_PI / 2);
rotX.fromAxisAngle(Vector<3>(1, 0, 0), M_PI / 4);

// Combined rotation (order matters!)
Quaternion combined = rotZ * rotX;
```

**Orientation Rate (Angular Velocity):**

```cpp
Quaternion orientation = state.getOrientation();
double dt = 0.01;  // 10ms

Vector<3> angularVel = orientation.toAngularVelocity(dt);
LOGI("Angular rates: %.2f, %.2f, %.2f rad/s",
     angularVel.x(), angularVel.y(), angularVel.z());
```

---

## Integration with Astra

These math classes are used throughout Astra's sensor and state systems:

**Sensors:**
```cpp
Vector<3> accelBody = imu.getAccel();
Vector<3> positionLLA = gps.getPos();
```

**State:**
```cpp
Vector<3> velocity = state.getVelocity();
Vector<3> acceleration = state.getAcceleration();
Quaternion orientation = state.getOrientation();
```

**Filters:**
```cpp
class MyKalmanFilter : public LinearKalmanFilter {
    Matrix F;  // State transition
    Matrix H;  // Measurement model
    Matrix Q;  // Process noise
    Matrix R;  // Measurement noise
    // ...
};
```

---

## Best Practices

1. **Use stack allocation for Vectors**: Vectors are lightweight and can be allocated on the stack.

2. **Heap-allocate Matrix data**: Always use `new double[]` for matrix arrays:
   ```cpp
   double* data = new double[9]{...};
   Matrix m(3, 3, data);
   ```

3. **Normalize quaternions regularly**: Floating-point errors accumulate:
   ```cpp
   orientation.normalize();  // Maintain unit magnitude
   ```

4. **Prefer quaternions for rotations**: More stable than Euler angles, no gimbal lock.

5. **Check vector magnitude before normalization**: Avoid division by zero:
   ```cpp
   double mag = v.magnitude();
   if (mag > 1e-6) {
       v.normalize();
   }
   ```

6. **Use matrix inversion carefully**: Inversion is expensive and can fail for singular matrices.

---

## Limitations

**Vector:**
- Fixed size at compile time
- Cross product only defined for 3D vectors

**Matrix:**
- Shallow copy of data array (ownership transfer)
- Dense matrices only (no sparse matrix support)
- Limited to 255×255 size (uses `uint8_t` for dimensions)
- Matrix inversion uses LU decomposition (expensive for large matrices)

**Quaternion:**
- Quaternion multiplication is non-commutative (order matters)
- Floating-point drift requires periodic normalization
- Euler angle conversion has gimbal lock at ±90° pitch

---

## Summary

- **Vector**: Fixed-size vector for 2D/3D math, supports arithmetic and products
- **Matrix**: Dense matrix for linear algebra, used in Kalman filters
- **Quaternion**: Rotation representation, preferred for 3D orientation
- All classes adapted from proven libraries (Adafruit IMU)
- Integrated throughout Astra's sensor and state systems

For usage in state estimation, see the [Filter](../ifaces/filters.md) and [State](../ifaces/state.md) documentation.

---
