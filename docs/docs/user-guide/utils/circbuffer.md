# CircBuffer

The `CircBuffer` (circular buffer) is a templated queue implementation that provides fixed-size FIFO (First In, First Out) storage with random access. It's designed for storing recent sensor readings, implementing moving averages, and buffering time-series data without dynamic memory allocation after initialization.

---

## Overview

A circular buffer maintains a fixed-size array that wraps around when full. New elements overwrite the oldest data automatically. This makes it ideal for:

1. **Moving averages** - Store recent sensor readings to calculate averages
2. **Ground calibration** - Buffer sensor data during pre-launch to establish baselines
3. **Signal filtering** - Smooth noisy sensor data
4. **Time-series analysis** - Keep a sliding window of recent measurements

The `CircBuffer` supports both queue operations (push/pop) and array-like random access via the `[]` operator.

---

## Template Definition

```cpp
template <typename T>
class CircBuffer {
public:
    CircBuffer(int size);

    void push(T item);
    T pop();
    T peek();

    int getCount();
    int getSize();
    bool isFull();
    bool isEmpty();
    void clear();

    T& operator[](int index);
    T operator[](int index) const;
};
```

**Type parameter `T`:** Can be any type (`double`, `int`, `Vector<3>`, custom structs, etc.)

---

## Basic Usage

### Creating a Buffer

```cpp
#include <CircBuffer.h>

// Buffer for 100 double values
CircBuffer<double> pressureBuffer(100);

// Buffer for 50 3D vectors
CircBuffer<Vector<3>> positionBuffer(50);

// Buffer for custom types
struct SensorReading {
    double timestamp;
    double value;
};
CircBuffer<SensorReading> dataBuffer(200);
```

### Adding and Removing Elements

```cpp
CircBuffer<double> buffer(5);

// Add elements
buffer.push(1.5);
buffer.push(2.3);
buffer.push(3.7);

// Remove oldest element
double oldest = buffer.pop();  // Returns 1.5

// View oldest without removing
double next = buffer.peek();   // Returns 2.3 (doesn't remove)
```

### Random Access

```cpp
CircBuffer<double> buffer(10);
buffer.push(10.0);
buffer.push(20.0);
buffer.push(30.0);

// Access by index (0 = oldest element)
double oldest = buffer[0];     // 10.0
double newest = buffer[buffer.getCount() - 1];  // 30.0

// Iterate over all elements
for (int i = 0; i < buffer.getCount(); i++) {
    Serial.println(buffer[i]);
}
```

---

## Core Methods

### push(item)

Add an item to the buffer. If full, overwrites the oldest element.

```cpp
CircBuffer<double> buffer(3);
buffer.push(1.0);
buffer.push(2.0);
buffer.push(3.0);
buffer.push(4.0);  // Overwrites 1.0

// Buffer now contains: [2.0, 3.0, 4.0]
```

### pop()

Remove and return the oldest element. Returns default-constructed value if empty.

```cpp
CircBuffer<int> buffer(5);
buffer.push(10);
buffer.push(20);

int val = buffer.pop();  // Returns 10
// Buffer now contains: [20]
```

### peek()

View the oldest element without removing it. Returns default-constructed value if empty.

```cpp
CircBuffer<double> buffer(5);
buffer.push(1.5);

double val = buffer.peek();  // Returns 1.5
// Buffer still contains: [1.5]
```

### getCount()

Returns the number of elements currently in the buffer.

```cpp
CircBuffer<int> buffer(10);
buffer.push(1);
buffer.push(2);

int count = buffer.getCount();  // Returns 2
```

### getSize()

Returns the maximum capacity of the buffer.

```cpp
CircBuffer<int> buffer(100);
int capacity = buffer.getSize();  // Returns 100
```

### isFull()

Returns `true` if the buffer is at maximum capacity.

```cpp
CircBuffer<int> buffer(3);
buffer.push(1);
buffer.push(2);
buffer.push(3);

if (buffer.isFull()) {
    // Buffer is full, next push will overwrite oldest
}
```

### isEmpty()

Returns `true` if the buffer contains no elements.

```cpp
CircBuffer<int> buffer(10);

if (buffer.isEmpty()) {
    // No elements to pop
}
```

### clear()

Remove all elements from the buffer without changing capacity.

```cpp
CircBuffer<int> buffer(10);
buffer.push(1);
buffer.push(2);
buffer.clear();

// Buffer is now empty, count = 0
```

---

## Practical Examples

### Moving Average Filter

Calculate the average of the last N sensor readings:

```cpp
CircBuffer<double> pressureBuffer(100);

void loop() {
    double pressure = baro.getPressure();
    pressureBuffer.push(pressure);

    // Calculate average
    double sum = 0;
    int count = pressureBuffer.getCount();
    for (int i = 0; i < count; i++) {
        sum += pressureBuffer[i];
    }
    double avgPressure = sum / count;

    LOGI("Pressure: %.2f Pa (avg: %.2f)", pressure, avgPressure);
}
```

### Ground Level Calibration

Buffer sensor data before launch to establish a ground reference:

```cpp
class CustomBarometer : public Barometer {
private:
    CircBuffer<double> pressureBuffer{100};  // Buffer for calibration
    bool calibrated = false;
    double groundPressure = 0;

protected:
    bool read() override {
        pressure = hardware.readPressure();

        if (!calibrated) {
            pressureBuffer.push(pressure);

            // Use first 50 readings for calibration
            if (pressureBuffer.getCount() >= 50) {
                double sum = 0;
                for (int i = 0; i < 50; i++) {
                    sum += pressureBuffer[i];
                }
                groundPressure = sum / 50.0;
                calibrated = true;

                LOGI("Ground pressure: %.2f Pa", groundPressure);
            }
        }

        return true;
    }
};
```

### Velocity Estimation from Position

Calculate velocity using recent position history:

```cpp
struct PositionTimestamp {
    Vector<3> position;
    double timestamp;
};

CircBuffer<PositionTimestamp> posHistory(10);

Vector<3> estimateVelocity() {
    if (posHistory.getCount() < 2) {
        return Vector<3>(0, 0, 0);
    }

    // Use oldest and newest positions
    PositionTimestamp oldest = posHistory[0];
    PositionTimestamp newest = posHistory[posHistory.getCount() - 1];

    double dt = newest.timestamp - oldest.timestamp;
    if (dt < 0.001) return Vector<3>(0, 0, 0);

    Vector<3> displacement = newest.position - oldest.position;
    return displacement / dt;
}

void loop() {
    PositionTimestamp pt;
    pt.position = state.getPosition();
    pt.timestamp = millis() / 1000.0;

    posHistory.push(pt);

    Vector<3> velocity = estimateVelocity();
}
```

### Peak Detection

Detect local maxima in sensor data:

```cpp
CircBuffer<double> altitudeBuffer(20);

bool detectApogee() {
    if (altitudeBuffer.getCount() < 20) return false;

    // Check if middle element is a local maximum
    double center = altitudeBuffer[10];

    // Check all before are lower
    for (int i = 0; i < 10; i++) {
        if (altitudeBuffer[i] >= center) return false;
    }

    // Check all after are lower
    for (int i = 11; i < 20; i++) {
        if (altitudeBuffer[i] >= center) return false;
    }

    return true;  // Center is a local maximum
}
```

### Outlier Rejection

Reject sensor readings that deviate significantly from recent history:

```cpp
CircBuffer<double> gyroBuffer(50);

bool isOutlier(double newReading, double threshold = 3.0) {
    if (gyroBuffer.getCount() < 10) return false;

    // Calculate mean
    double sum = 0;
    int count = gyroBuffer.getCount();
    for (int i = 0; i < count; i++) {
        sum += gyroBuffer[i];
    }
    double mean = sum / count;

    // Calculate standard deviation
    double sqSum = 0;
    for (int i = 0; i < count; i++) {
        double diff = gyroBuffer[i] - mean;
        sqSum += diff * diff;
    }
    double stdDev = sqrt(sqSum / count);

    // Check if reading is beyond threshold
    return abs(newReading - mean) > (threshold * stdDev);
}

void loop() {
    double gyroX = imu.getGyroscopeGlobal().x();

    if (!isOutlier(gyroX)) {
        gyroBuffer.push(gyroX);
        // Use reading
    } else {
        LOGW("Gyro outlier rejected: %.2f", gyroX);
    }
}
```

---

## Integration with Sensors

Many sensor classes use `CircBuffer` internally for calibration and filtering. You can access these patterns in custom sensor implementations:

```cpp
class SmartBarometer : public Barometer {
private:
    static const int BUFFER_SIZE = 100;
    static const int CALIBRATION_SAMPLES = 50;

    CircBuffer<double> pressureBuffer{BUFFER_SIZE};
    bool calibrated = false;

protected:
    bool read() override {
        pressure = hardware.readPressure();
        pressureBuffer.push(pressure);

        if (!calibrated && pressureBuffer.getCount() >= CALIBRATION_SAMPLES) {
            calibrateGround();
        }

        return true;
    }

private:
    void calibrateGround() {
        double sum = 0;

        // Use only first half to avoid launch transients
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            sum += pressureBuffer[i];
        }

        double groundPressure = sum / CALIBRATION_SAMPLES;
        altitudeASL = calcAltitude(groundPressure);
        calibrated = true;

        LOGI("Ground altitude calibrated: %.2f m", altitudeASL);
    }
};
```

---

## Copy Semantics

`CircBuffer` supports both copy construction and copy assignment:

```cpp
CircBuffer<double> buffer1(10);
buffer1.push(1.0);
buffer1.push(2.0);

// Copy constructor
CircBuffer<double> buffer2 = buffer1;

// Copy assignment
CircBuffer<double> buffer3(10);
buffer3 = buffer1;

// All three buffers now contain [1.0, 2.0]
```

Copies create independent buffers—modifying one doesn't affect the others.

---

## Memory Considerations

- **Fixed allocation**: Memory is allocated once during construction and never grows
- **Heap storage**: Buffer uses dynamic allocation internally (`new T[size]`)
- **Copy overhead**: Copying a buffer allocates new memory and copies all elements
- **Template instantiation**: Each `CircBuffer<T>` type creates a separate class

**Memory usage:**
```
Bytes = sizeof(T) * size + overhead
```

For example:
- `CircBuffer<double>(100)` ≈ 800 bytes (8 bytes/double × 100)
- `CircBuffer<Vector<3>>(50)` ≈ 1200 bytes (24 bytes/vector × 50)

---

## Best Practices

1. **Size buffers appropriately**: Larger buffers provide better filtering but consume more memory. Typical sizes: 20-200 elements.

2. **Check before accessing**: Always verify count before array access:
   ```cpp
   if (buffer.getCount() > 0) {
       double val = buffer[0];
   }
   ```

3. **Avoid frequent copies**: Copying large buffers is expensive. Pass by reference when possible:
   ```cpp
   void processBuffer(const CircBuffer<double>& buffer) {
       // Use buffer without copying
   }
   ```

4. **Clear on reset**: Clear buffers when transitioning stages to avoid stale data:
   ```cpp
   if (stageChanged) {
       gyroBuffer.clear();
   }
   ```

5. **Use const access**: Use the const `operator[]` for read-only access to enable compiler optimizations.

6. **Calibration patterns**: Collect buffer data during stable periods (pre-launch, steady flight) for reliable baselines.

---

## Common Patterns

### Exponential Moving Average

```cpp
double ema = 0;
double alpha = 0.1;

void loop() {
    double reading = sensor.read();
    ema = alpha * reading + (1 - alpha) * ema;
}
```

### Median Filter

```cpp
CircBuffer<double> buffer(9);

double getMedian() {
    if (buffer.getCount() < 9) return 0;

    double sorted[9];
    for (int i = 0; i < 9; i++) {
        sorted[i] = buffer[i];
    }

    // Simple bubble sort
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8 - i; j++) {
            if (sorted[j] > sorted[j + 1]) {
                double tmp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = tmp;
            }
        }
    }

    return sorted[4];  // Middle element
}
```

---

## Limitations

- **Fixed size**: Cannot grow or shrink after construction
- **No bounds checking in release**: Array access via `[]` doesn't validate indices in production builds
- **Overwrites on full**: Pushing to a full buffer silently overwrites oldest data
- **No iterators**: No STL-style iterators; use index-based loops instead

---

## Summary

- `CircBuffer` provides fixed-size FIFO queue with random access
- Ideal for moving averages, calibration, and filtering
- Automatically overwrites oldest data when full
- Supports both queue operations (push/pop) and array access (`[]`)
- Templated for any data type
- Fixed memory allocation—size set at construction

For examples of `CircBuffer` in real sensors, see the barometer implementation in [src/Sensors/Baro/Barometer.cpp](../Sensors/Baro/Barometer.cpp).

---
