# Mahony AHRS Filter

## Overview

The Mahony filter is a **pure math** attitude estimation algorithm using complementary filtering. It fuses gyroscope, accelerometer, and optionally magnetometer data to produce orientation estimates.

**Key Philosophy:** This implementation is **application-agnostic**. Rocket-specific logic (snap-to-vertical, mode switching, etc.) belongs in higher layers (State or RocketState), not in the filter itself.

## Architecture Layers

```
Application Logic (RocketState)          ← Rocket-specific: snap-to-vertical, launch detection
    ↓
State (Common AHRS management)           ← Common logic: high-G switching, sensor trust
    ↓
MahonyAHRS (Pure Math Filter)            ← Pure math: quaternion integration, no modes
```

## Usage

### Basic Setup

```cpp
// Create filter with tunable gains
MahonyAHRS orientationFilter(Kp, Ki);

// Initialize with initial orientation (optional)
orientationFilter.setQuaternion(initialQuat);

// Update loop
orientationFilter.update(accel, gyro, mag, dt);  // With mag
orientationFilter.update(accel, gyro, dt);       // Without mag

// Get results
Quaternion orientation = orientationFilter.getQuaternion();
Vector<3> earthAccel = orientationFilter.getEarthAcceleration(accel);
```

### Tuning Parameters

- **Kp** (Proportional gain): How quickly to correct orientation errors
  - Higher = faster correction, more sensitive to noise
  - Typical: 0.1 - 2.0
  - Rocket default: 0.1 (slower correction during high-G)

- **Ki** (Integral gain): Gyro bias correction over time
  - Higher = faster bias removal, can introduce drift
  - Typical: 0.0 - 0.01
  - Rocket default: 0.0005

### Magnetometer Usage

**When to use mag:**
- Provides absolute heading reference (yaw relative to magnetic north)
- Useful for navigation, tracking orientation over long durations
- Prevents yaw drift in gyro-only mode

**When NOT to use mag:**
- Near electric motors (drones, quadcopters) - massive magnetic interference
- Near ferromagnetic materials that move relative to sensor
- If absolute heading not required (gyro-only works fine for relative orientation)

**Rockets:** Solid/liquid rocket motors don't create significant magnetic fields (unlike electric motors). Mag should work fine, but adds calibration complexity. Decision: user configurable.

## Sensor Behavior

| Sensor | Measures | Good For | Bad For | Trust When |
|--------|----------|----------|---------|------------|
| **Gyro** | Angular velocity | Short-term rotation | Long-term (drifts) | Always (with bias correction) |
| **Accel** | Specific force (gravity + linear accel) | Long-term "up" direction | During acceleration | ~1g (stationary or coast) |
| **Mag** | Magnetic field | Absolute heading (yaw) | Near electric motors/metal | Always (if calibrated) |

## Frames of Reference

- **Sensor/Board Frame:** Raw sensor readings (IMU chip orientation)
- **Body Frame:** Vehicle-fixed frame (rocket/drone body)
  - X: Forward, Y: Right, Z: Down (NED convention)
- **Inertial Frame:** Earth-fixed frame relative to launch/start point
  - NED (North, East, Down) for rockets
  - ENU (East, North, Up) for some applications

The filter handles **Body → Inertial** transformation via quaternion `q`.

## Removed Features (Application-Specific)

The following features were **removed** from Mahony and moved to application layers:

### 1. MahonyMode Enum (REMOVED)
Previously had `CALIBRATING`, `CORRECTING`, `GYRO_ONLY` modes.

**Why removed:** These are application-specific behaviors, not math operations.

**Where it went:**
- **Calibration logic** → User calls `reset()` or `setQuaternion()` as needed
- **High-G switching** → State::updateOrientation() checks accel magnitude
- **Snap-to-vertical (rocket)** → RocketState::updateOrientation() override

### 2. lockFrame() (REMOVED)
Previously locked reference frame at liftoff.

**Why removed:** Launch detection is rocket-specific.

**Where it went:**
- RocketState detects launch and calls `setQuaternion()` to inject orientation

### 3. computeBoardToBodyRotation() (REMOVED)
Previously auto-detected board mounting orientation.

**Why removed:** Board mounting is application-specific.

**Where it went:**
- User configures board→body transformation externally
- Pass pre-rotated vectors to Mahony, or apply rotation after getting quaternion

### 4. Automatic Mode Switching (REMOVED)
Previously switched modes based on accel magnitude.

**Why removed:** Application-specific logic.

**Where it went:**
- State checks accel magnitude and decides whether to pass accel to Mahony
- Example: `if (accelMag ~= 1g) { filter.update(accel, gyro, dt); } else { filter.update(gyro, dt); }`

## New API

### Core Methods

```cpp
// Update with all sensors
void update(const Vector<3> &accel, const Vector<3> &gyro, const Vector<3> &mag, double dt);

// Update without magnetometer (6-DOF)
void update(const Vector<3> &accel, const Vector<3> &gyro, double dt);

// Update gyro-only (dead reckoning)
void update(const Vector<3> &gyro, double dt);

// Get current orientation (returns identity if not ready)
Quaternion getQuaternion() const;

// Transform body-frame accel to inertial frame (removes gravity)
Vector<3> getEarthAcceleration(const Vector<3> &accel) const;

// Inject external orientation (for initial alignment, snap-to-vertical, etc.)
void setQuaternion(const Quaternion &q);

// Reset filter to initial state
void reset();

// Check if filter has been initialized
bool isReady() const;
```

### Magnetometer Calibration (Optional)

If using magnetometer:

```cpp
// Collect samples during "mag dance" (rotate board in all axes)
for (int i = 0; i < 500; i++) {
    filter.collectMagCalibrationSample(mag);
    delay(20);
}

// Compute hard/soft iron calibration
filter.finalizeCalibration();

// Check if mag is calibrated
if (filter.isMagCalibrated()) {
    // Can now use 9-DOF update
}
```

## Example: Rocket Application

```cpp
// In RocketState::updateOrientation() override:
void RocketState::updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt) {
    // Rocket-specific: Snap to vertical while on pad
    if (onPad) {
        // Find body axis closest to gravity
        Vector<3> closestAxis = findClosestAxisToGravity(accel);

        // Create quaternion aligning that axis with "up"
        Quaternion snapQuat = createSnapToVerticalQuat(closestAxis, accel);

        // Inject into filter
        orientationFilter->setQuaternion(snapQuat);
    }

    // Common: High-G switching (could also be in base State)
    double accelMag = accel.magnitude();
    if (abs(accelMag - 9.81) < 1.0) {
        // Trust accel - low acceleration environment
        orientationFilter->update(accel, gyro, dt);
    } else {
        // Don't trust accel - high-G or freefall
        orientationFilter->update(gyro, dt);
    }

    // Update state variables
    orientation = orientationFilter->getQuaternion();
    acceleration = orientationFilter->getEarthAcceleration(accel);
}
```

## Algorithm Details

### Complementary Filtering

The Mahony filter is a **complementary filter**:
- **Gyro** provides short-term orientation (integrates angular velocity)
- **Accel** corrects long-term drift (provides gravity reference)
- **Mag** corrects yaw drift (provides north reference)

### Error Correction

1. **Estimate gravity/north in body frame** from current quaternion
2. **Compare to measured accel/mag** → compute error vector (cross product)
3. **Apply proportional + integral feedback** to gyro rate
4. **Integrate corrected gyro** to update quaternion

### Bias Estimation

The integral term `Ki` accumulates gyro bias over time:
```cpp
bias += Ki * error * dt
correctedGyro = gyro - bias + Kp * error
```

## References

- [Mahony et al. (2008)](https://ieeexplore.ieee.org/document/4608934) - Original paper
- [Madgwick's IMU Guide](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/) - Comparison with Madgwick filter

## Migration from Old API

If you had code using `MahonyMode`:

**Before:**
```cpp
filter.setMode(MahonyMode::CALIBRATING);
filter.update(accel, gyro, dt);
filter.finalizeCalibration();
filter.lockFrame();
filter.setMode(MahonyMode::CORRECTING);
```

**After:**
```cpp
// Option 1: Let user handle calibration
filter.reset();
// ... collect samples ...
filter.setQuaternion(initialOrientation);

// Option 2: High-G switching in State layer
if (abs(accel.magnitude() - 9.81) < 1.0) {
    filter.update(accel, gyro, dt);  // Trust accel
} else {
    filter.update(gyro, dt);  // Gyro-only
}
```
