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
## Filter Tuning

### Tuning Parameters

- **Kp** (Proportional gain): How quickly to correct orientation errors
  - Higher = faster correction, more sensitive to noise
  - Lower = Relies too much on gyroscope, can accumulate drift
  - Typical: 0.1 - 2.0
  - Current Rocket default: 0.1 (slower correction during high-G)

- **Ki** (Integral gain): Gyro bias correction over time
  - Higher = faster bias removal, can introduce drift
  - Lower = Gyroscope bias won't be accurately tracked
  - Typical: 0.0 - 0.01
  - Current Rocket default: 0.0005

### Bench Testing
#### Overview
 Tune $K_p$ and $K_i$ individually. Set $K_i = 0$ initially. Increase $K_p$ until the filter is responsive to changes in orientation but not noisy. Then, slowly increase $K_i$ until the filter is able to correct for gyro bias. The avionics bay should be fully assembled before tuning filters, especially if the magnetometer is being used. The magnotemeter, as mentioned before, is sensitive to magnetic interference from components.


#### Procedures
Near hover/pad equilibrium, the small angle linearized dynamics of this Mahony implementation behave as a standard second order system

$$
s^2 + K_p s + K_i = 0 \rightarrow \omega_n = \sqrt{K_i}, \quad \zeta = \frac{K_p}{2 \sqrt{K_i}}
$$

With $K_i = 0$ Keep the IMU motionless at a known orientation, and deliberately introduce a 10 degree tilt error to the filter. Compare a few values of $K_p$ such as $K_p = \{0.1, 0.5, 1.0, 2.0\}$. For small tilt errors, $\tilde\theta(t)\approx\tilde\theta_0e^{-K_pt}$. The time to reduce an error from 10 degrees to 1 is given by $\tau_{10 \rightarrow 1} = \frac{\ln 10}{K_p}$. Measure the recovery time and stationary noise, and choose the **lowest** gain that meets the recovery requirement while keeping noise/disturbance to acceptable levels.

To avoid oscillatory overshoot when recovering from disturbances, aim for a critically damped to slightly overdamped response ($\zeta∈[1.0,2.0]$).

$$
K_i = \left(\frac{K_p}{2\zeta}\right)^2
$$


 Inject a synthetic gyro bias of +0.02 rad/s (1.15˚/s) into the raw gyro vector before passing it to update(). Verify that `_biasX/Y/Z` converges to +0.02 rad/s within 15–20 seconds with zero steady-state tilt error

### Monte Carlo

A Monte Carlo validation requires a 6DOF truth trajectory pared with a stochastic IMU error model. Reference truth can be generated from OpenRocket trajectories spanning Pad Standy -> Boost -> Coast -> Apogee. 

Run $N = 500, 1000$ trials and randomize the following parameters:

| Parameter Category | Parameter | Distribution | Dispersion Range (3σ or Bounds) | What It Stress-Tests in `Mahony.h` |
| :--- | :--- | :--- | :--- | :--- |
| **Gyroscope** | Turn-on Bias (b_g0) | Uniform / Normal | ±0.035 rad/s (±2 deg/s) | Whether Pad Standby `Ki` converges `_biasX/Y/Z` before T-0. |
| **Gyroscope** | Angle Random Walk (White Noise) | Normal (per step) | σ_g = 0.005 to 0.015 rad/s/√Hz | Dead-reckoning drift during 3-DOF Boost/Coast (`update(gyro, dt)`). |
| **Gyroscope** | G-Sensitivity (Linear Accel Bias) | Normal | ±0.001 rad/s/g | High-G motor burn inducing unmodeled roll/pitch drift. |
| **Accelerometer** | Pad Rail Wind Vibration | Colored/Sine Noise | 2–8 Hz, amplitude ±0.15g | Whether Pad Standby `Kp` is too high (tracking wind sway as tilt). |
| **Accelerometer** | Boost Vibro-Acoustic Noise | Normal (during burn) | σ_a = 0.5 to 2.0g RMS | False launch triggers or premature re-entry into 6-DOF mode. |
| **Magnetometer** | Residual Hard/Soft Iron Error | Uniform | ±5% scale, ±10° heading offset | Sensitivity of `magWeight` to launch-rail magnetic distortion. |
| **Flight Dynamics** | Roll Rate during Boost ($ω_z$) | Uniform | 0.5 to 5.0 rev/s (3 to 31 rad/s) | First-order Euler integration error in `integrateQuaternion()`. |
| **Flight Software** | Pad Dwell Time (T_pad) | Uniform | 15 s to 120 s | Minimum pad wait time needed for `_biasX/Y/Z` to settle. |
| **Flight Software** | Loop Δt Jitter | Normal (clipped) | Δt_nom ± 20% (with occasional 2× spike) | Robustness of Euler step and PI accumulator to RTOS/loop jitter. |

Evaluate four specific metrics using quaternion error $\bold{q}_{err} = \bold{q}_{true}^* \otimes \bold{q}_{est}$. 

**Pre launch bias estimation error (T-0.1s)**
$$
e_{\text{bias}} = ||\bold{b}_\text{est}(T_0) - \bold{b}_{\text{g,true}}(T_0)||_2
$$
Ensure $e_\text{bias} < 0.00015 \text{ rad/s}$. Any residual bias error will integrates linearly into attitude drift over time.

**Liftoff Attitude Error (T + 0.05s)**
$$
\theta_{\text{err}} = 2 \arccos(|\text{clamp}(q_\text{err}, -1, 1)|)
$$
Target < 1.0˚.

**Apogee Tilt Error**
$$
\theta_{\text{apogee}} = 2 \arccos(|\text{clamp}(q_\text{err,apogee}, -1, 1)|)
$$
Target < 3.0˚.

**Inertial Vertical Velocity Error at Apogee**
$$
v_{z, err}(T_\text{apogee}) = \left| \int_{T_{0}}^{T_\text{apogee}}\hat{a}_\text{earth,z}(t) dt - v_{z, \text{true}}(T_\text{apogee}) \right|
$$

### HIL playback

 After tuning, run the Mahony AHRS through existing IMU flight logs to verify performance
 - Plot `_biasX`, `_biasY` and `_biasZ` (will need to create a getter function to expose the gyro bias) as a function of time during the pre-launch window and confirm they flatten out before T-0
 - Inspect `getEarthAcceleration(accel).z()` at liftoff. Verify that switching to `update(gyro, dt)` at $∥a∥>1.3g$ prevents any pitch/roll spike during motor burn.
 - Integrate `getEarthAcceleration(accel)` during boost and coast to compare vertical velocity against your barometric altimeter's derivative. If tilt drift during the 3-DOF coast phase is excessive, increase the Pad Standby $K_i$ slightly so the pre-launch gyro bias estimate is tighter before liftoff