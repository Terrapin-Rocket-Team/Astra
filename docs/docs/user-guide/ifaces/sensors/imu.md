# IMU (6‑DoF and 9‑DoF)

Astra models IMUs as composite sensors:

- `IMU6DoF` → accelerometer + gyroscope
- `IMU9DoF` → accelerometer + gyroscope + magnetometer

Each IMU exposes *component sensors* so Astra can use them for state estimation.

---

## Component Access

```cpp
IMU6DoF* imu = ...;
Accel* accel = imu->getAccelSensor();
Gyro* gyro = imu->getGyroSensor();
```

```cpp
IMU9DoF* imu = ...;
Accel* accel = imu->getAccelSensor();
Gyro* gyro = imu->getGyroSensor();
Mag* mag = imu->getMagSensor();
```

---

## Using with AstraConfig

```cpp
AstraConfig config = AstraConfig()
    .with6DoFIMU(&imu)  // or with9DoFIMU(&imu)
    .withState(&state);
```

This automatically extracts the component sensors for `SensorManager`, and logs the IMU as a misc sensor.

---

## Implementations

| Class | Type |
|------|------|
| `BMI088` | 6‑DoF |
| `BNO055` | 9‑DoF |

Example:

```cpp
#include <Sensors/HW/IMU/BMI088.h>

BMI088 imu("BMI088", Wire);
```

---

## Mounting Orientation

IMUs are `RotatableSensor`s. If the sensor is rotated on the board:

```cpp
imu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
```

This orientation is propagated to the component sensors.

---

## Orientation Note

IMU classes only report **raw sensor data**.  
Orientation is estimated in `State` via `MahonyAHRS`.
