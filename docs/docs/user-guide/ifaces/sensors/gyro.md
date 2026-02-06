# Gyroscope

`Gyro` is the base class for angular velocity sensors.

---

## Key Methods

```cpp
Vector<3> getAngVel() const; // rad/s
bool isHealthy() const;
```

Health tracking detects stuck readings similarly to `Accel`.

---

## Implementations

Standalone gyro drivers are not currently included. Most applications use the gyro component from an IMU:

```cpp
IMU6DoF* imu = ...;
Gyro* gyro = imu->getGyroSensor();
```

