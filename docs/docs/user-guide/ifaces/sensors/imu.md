# IMU

The `IMU` class provides a standardized interface for inertial measurement units in Astra. It extends the [`Sensor`](../sensor.md) interface to handle 9DOF (degrees of freedom) sensors combining accelerometers, gyroscopes, and magnetometers for complete orientation and motion tracking.

---

## Overview

IMUs measure linear acceleration, angular velocity, and magnetic field to determine orientation and motion. The `IMU` base class provides:

1. **Accelerometer data** - Linear acceleration in 3 axes
2. **Gyroscope data** - Angular velocity in 3 axes
3. **Magnetometer data** - Magnetic field in 3 axes
4. **Orientation estimation** - Quaternion representation of attitude
5. **Euler angles** - Roll, pitch, yaw for intuitive understanding

Implementing a new IMU requires only defining `init()` and `read()` methods—orientation fusion can be handled automatically or by hardware.

---

## Available Methods

### Acceleration

```cpp
virtual Vector<3> getAcceleration();           // Body frame acceleration (m/s²)
```

Returns acceleration in the sensor's local (body) frame.

**Example:**
```cpp
BMI088andLIS3MDL imu;
Vector<3> accel = imu.getAcceleration();
// accel[0] = X-axis acceleration
// accel[1] = Y-axis acceleration
// accel[2] = Z-axis acceleration (includes gravity!)
```

### Angular Velocity

```cpp
virtual Vector<3> getAngularVelocity();        // Rotation rates (rad/s)
```

Returns rotation rates around each axis (gyroscope data).

**Example:**
```cpp
Vector<3> gyro = imu.getAngularVelocity();
// gyro[0] = Roll rate (rad/s)
// gyro[1] = Pitch rate (rad/s)
// gyro[2] = Yaw rate (rad/s)
```

### Magnetic Field

```cpp
virtual Vector<3> getMagField();               // Magnetic field (μT)
```

Returns magnetic field strength in microteslas.

**Example:**
```cpp
Vector<3> mag = imu.getMagField();
// mag[0] = X-axis field strength
// mag[1] = Y-axis field strength
// mag[2] = Z-axis field strength
```

### Orientation (from State or Filter)

While the `IMU` class stores orientation data internally, the best way to access vehicle orientation is through the `State` object, which may apply additional filtering:

```cpp
// In State or custom state estimation
Quaternion orient = vehicleState.getOrientation();
```

---

## Logged Data Columns

The `IMU` class automatically registers these columns for telemetry:

| Column | Format | Units | Description |
|--------|--------|-------|-------------|
| `accel_x` | `%.3f` | m/s² | X-axis acceleration |
| `accel_y` | `%.3f` | m/s² | Y-axis acceleration |
| `accel_z` | `%.3f` | m/s² | Z-axis acceleration |
| `gyro_x` | `%.3f` | rad/s | X-axis rotation rate |
| `gyro_y` | `%.3f` | rad/s | Y-axis rotation rate |
| `gyro_z` | `%.3f` | rad/s | Z-axis rotation rate |
| `mag_x` | `%.2f` | μT | X-axis magnetic field |
| `mag_y` | `%.2f` | μT | Y-axis magnetic field |
| `mag_z` | `%.2f` | μT | Z-axis magnetic field |
| `quat_w` | `%.4f` | - | Quaternion W component |
| `quat_x` | `%.4f` | - | Quaternion X component |
| `quat_y` | `%.4f` | - | Quaternion Y component |
| `quat_z` | `%.4f` | - | Quaternion Z component |

---

## Implementing a Custom IMU

To add support for a new IMU sensor, inherit from `IMU` and implement `init()` and `read()`:

### Basic Structure

```cpp
#ifndef MY_IMU_H
#define MY_IMU_H

#include "Sensors/IMU/IMU.h"
#include <SomeIMULibrary.h>

namespace astra {

class MyIMU : public IMU {
public:
    MyIMU(const char *name = "MyIMU");

protected:
    bool init() override;
    bool read() override;

private:
    SomeIMUDriver hardware;
};

}

#endif
```

### Implementation Example

```cpp
#include "MyIMU.h"

using namespace astra;

MyIMU::MyIMU(const char *name) : IMU(name) {
    // IMU columns are registered by parent
}

bool MyIMU::init() {
    // Initialize I2C communication
    if (!hardware.begin()) {
        LOGE("MyIMU: Failed to initialize");
        return false;
    }

    // Configure sensor settings
    hardware.setAccelRange(16);      // ±16g
    hardware.setGyroRange(2000);     // ±2000°/s
    hardware.setFilterBandwidth(50); // 50Hz lowpass

    LOGI("MyIMU: Initialized successfully");
    return true;
}

bool MyIMU::read() {
    // Read from hardware
    if (!hardware.dataReady()) {
        return false;
    }

    // Update protected member variables
    measuredAcc[0] = hardware.getAccelX();
    measuredAcc[1] = hardware.getAccelY();
    measuredAcc[2] = hardware.getAccelZ();

    measuredGyro[0] = hardware.getGyroX();
    measuredGyro[1] = hardware.getGyroY();
    measuredGyro[2] = hardware.getGyroZ();

    measuredMag[0] = hardware.getMagX();
    measuredMag[1] = hardware.getMagY();
    measuredMag[2] = hardware.getMagZ();

    // If your sensor provides orientation, update it
    orientation = hardware.getQuaternion();

    return true;
}
```

### Important Notes

1. **Units**: Acceleration in m/s², gyro in rad/s, mag in μT
2. **Protected members**: Set `measuredAcc`, `measuredGyro`, `measuredMag`, and `orientation`
3. **Coordinate frames**: Body frame (sensor-relative) for raw data
4. **Return false on error**: Signal communication failures

---

## Available Implementations

Astra includes drivers for common IMUs:

### BMI088andLIS3MDL

Combines Bosch BMI088 (6DOF) with ST LIS3MDL (magnetometer) for 9DOF.

```cpp
#include "Sensors/IMU/BMI088andLIS3MDL.h"

BMI088andLIS3MDL imu;
```

**Specifications:**
- Accelerometer: ±3g to ±24g
- Gyroscope: ±125°/s to ±2000°/s
- Magnetometer: ±4/±8/±12/±16 gauss
- Separate accelerometer and gyroscope for better performance

### BNO055

Bosch BNO055 with hardware sensor fusion.

```cpp
#include "Sensors/IMU/BNO055.h"

BNO055 imu;
```

**Specifications:**
- 9DOF with onboard ARM Cortex-M0 processor
- Hardware quaternion output
- Automatic calibration
- Lower cost but single-source integration

### MockIMU

For testing without hardware:

```cpp
#include "Sensors/IMU/MockIMU.h"

MockIMU imu;
```

Generates simulated IMU data for testing orientation algorithms.

---

## Usage Examples

### Basic Usage

```cpp
#include "Sensors/IMU/BMI088andLIS3MDL.h"

BMI088andLIS3MDL imu;

void setup() {
    Serial.begin(115200);

    if (!imu.begin()) {
        Serial.println("IMU init failed!");
    }
}

void loop() {
    imu.update();

    Vector<3> accel = imu.getAcceleration();
    Vector<3> gyro = imu.getAngularVelocity();

    Serial.print("Accel: ");
    Serial.print(accel[0], 3); Serial.print(", ");
    Serial.print(accel[1], 3); Serial.print(", ");
    Serial.print(accel[2], 3); Serial.println();

    Serial.print("Gyro: ");
    Serial.print(gyro[0], 3); Serial.print(", ");
    Serial.print(gyro[1], 3); Serial.print(", ");
    Serial.print(gyro[2], 3); Serial.println();

    delay(100);
}
```

### Launch Detection

```cpp
void detectLaunch() {
    Vector<3> accel = imu.getAcceleration();
    double totalAccel = accel.magnitude();

    // Detect launch when acceleration exceeds threshold
    if (totalAccel > 20.0) {  // 20 m/s² = ~2g
        LOGI("Launch detected! Accel: %.2f m/s²", totalAccel);
        launched = true;
    }
}
```

---

## Integration with State

When you pass an IMU to `State`, it contributes to orientation and acceleration estimation:

```cpp
BMI088andLIS3MDL imu;
DPS368 baro;
MAX_M10S gps;

Sensor *sensors[] = {&imu, &baro, &gps};
State vehicleState(sensors, 3, nullptr);
```

The `State` class uses IMU data to:
- Estimate vehicle orientation (quaternion)
- Provide body-frame acceleration
- Detect attitude changes (roll, pitch, yaw)
- Contribute to velocity estimation

---

## Coordinate Frames

### Body Frame (Sensor Frame)

IMU measurements are in the sensor's local coordinate system:

- **X**: Forward (nose direction)
- **Y**: Right wing
- **Z**: Down (through bottom of vehicle)

This is a right-handed coordinate system commonly used in aerospace.

### Inertial Frame (World Frame)

After sensor fusion, orientation relates body frame to world frame:

- **X**: East
- **Y**: North
- **Z**: Up

The `State` object transforms IMU data into this frame for navigation.

---

## Orientation Representation

### Quaternions

Quaternions are a 4-element representation of 3D rotation:

```cpp
Quaternion q = vehicleState.getOrientation();
// q = [w, x, y, z]
```

Advantages:
- No gimbal lock
- Smooth interpolation
- Efficient computation
- Used internally for sensor fusion

### Euler Angles

For human-readable output, convert to Euler angles:

```cpp
// Roll, pitch, yaw in radians
Vector<3> euler = quaternionToEuler(orientation);
float roll = euler[0];
float pitch = euler[1];
float yaw = euler[2];
```

**Conventions:**
- **Roll**: Rotation around X (wing dip)
- **Pitch**: Rotation around Y (nose up/down)
- **Yaw**: Rotation around Z (heading)

---

## Calibration

### Gyroscope Bias

Gyroscopes drift over time. The IMU can estimate and remove this bias:

1. Keep vehicle stationary
2. Let IMU average gyro readings
3. Calculated bias is subtracted automatically

### Accelerometer Calibration

Accelerometers measure gravity when stationary:
- X and Y should read near 0 m/s²
- Z should read near 9.81 m/s² (or -9.81 depending on orientation)

### Magnetometer Calibration

Magnetometers need hard-iron and soft-iron calibration:
- **Hard-iron**: Constant offset from nearby ferromagnetic materials
- **Soft-iron**: Distortion from ferromagnetic materials

Many IMU libraries include calibration routines. Run these before flight.

---

## Best Practices

1. **Mount rigidly**: IMUs are sensitive to vibration
   - Use vibration dampening if needed
   - Avoid flexible mounting

2. **Orient consistently**: Define your body frame clearly
   - Document X/Y/Z directions
   - Maintain orientation across sensor swaps

3. **Check data rates**: IMUs typically run at 100-1000Hz
   - Match update rate to sensor capability
   - Don't poll faster than sensor updates

4. **Validate orientation**: Test orientation before flight
   ```cpp
   Vector<3> accel = imu.getAcceleration();
   if (abs(accel.magnitude() - 9.81) > 2.0) {
       LOGW("IMU may not be calibrated correctly");
   }
   ```

5. **Handle saturation**: High-g events can saturate accelerometers
   - Choose appropriate range (±16g vs ±24g)
   - Log saturation events

6. **Keep magnetometers away from current**: Large currents create magnetic fields
   - Separate mag from power traces
   - Test with motors running

---

## Common Issues

### "Orientation drifts over time"

**Causes:**
- Gyroscope bias accumulation
- No magnetometer correction
- Poor sensor fusion

**Solutions:**
- Use magnetometer for yaw correction
- Implement better sensor fusion (Kalman filter)
- Recalibrate gyroscope regularly

### "Accelerometer reads wrong when moving"

**Cause:** Accelerometers measure specific force (gravity + motion)

**Solution:** This is normal. Use sensor fusion to separate gravity from motion acceleration.

### "Magnetometer gives bad heading"

**Causes:**
- Local magnetic disturbances
- Nearby ferromagnetic materials
- Uncalibrated magnetometer

**Solutions:**
- Calibrate magnetometer properly
- Mount away from batteries/motors
- Use GPS heading when available

### "High-frequency noise in data"

**Causes:**
- Vibration
- Electrical noise
- High sensor bandwidth

**Solutions:**
- Add mechanical damping
- Enable onboard digital filtering
- Implement software lowpass filter

---

## Advanced: Sensor Fusion

For best results, implement sensor fusion in your `State` extension or use a `Filter`:

```cpp
class FusedState : public State {
    void update(double currentTime) override {
        State::update(currentTime);

        // Get IMU data
        IMU *imu = dynamic_cast<IMU*>(getSensor(imuInstance.getType()));
        if (imu) {
            Vector<3> accel = imu->getAcceleration();
            Vector<3> gyro = imu->getAngularVelocity();

            // Fuse with other sensors
            // ... your fusion algorithm ...
        }
    }
};
```

Or use a Kalman filter—see [Filter documentation](../filters.md).

---

## Summary

- `IMU` provides standardized interface for 9DOF sensors
- Measures acceleration, angular velocity, and magnetic field
- Orientation can be estimated via sensor fusion
- Implement only `init()` and `read()` for new sensors
- Integrates with `State` for vehicle attitude estimation
- Several implementations included (BMI088+LIS3MDL, BNO055)

For general sensor information, see the [Sensor Interface](../sensor.md) documentation.

---
