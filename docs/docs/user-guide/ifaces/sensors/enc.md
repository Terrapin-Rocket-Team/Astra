# Encoder

The `Encoder_Astra` class provides an interface for rotary encoders in Astra. It extends the [`Sensor`](../sensor.md) interface to track rotational position through step counting, useful for motor position feedback, wheel odometry, or any application requiring rotation tracking.

---

## Overview

Encoders convert rotational motion into discrete digital pulses (steps). The `Encoder_Astra` class:

1. **Tracks step count** - Cumulative rotation measurement
2. **Provides relative position** - Steps from initialization
3. **Integrates with telemetry** - Automatic logging

Implementing a new encoder requires defining `init()` and `read()` methods to interface with your specific hardware.

---

## Available Methods

### getSteps()

```cpp
virtual int getSteps() const;
```

Returns the current relative step count (steps since initialization or since last reset).

**Example:**
```cpp
Encoder_Astra encoder;
int steps = encoder.getSteps();
// Positive = clockwise rotation
// Negative = counter-clockwise rotation
```

### setInitialSteps()

```cpp
virtual void setInitialSteps(int step);
```

Sets the reference point for relative step counting. Use this to zero the encoder or set a known position.

**Example:**
```cpp
encoder.setInitialSteps(0);  // Zero the encoder
// or
encoder.setInitialSteps(encoder.getSteps());  // Set current position as reference
```

---

## Logged Data Columns

The `Encoder_Astra` class automatically registers:

| Column | Format | Units | Description |
|--------|--------|-------|-------------|
| `steps` | `%d` | steps | Relative step count |

---

## Implementing a Custom Encoder

### Basic Structure

```cpp
#ifndef MY_ENCODER_H
#define MY_ENCODER_H

#include "Sensors/Encoder/AstraEncoder.h"

namespace astra {

class MyEncoder : public Encoder_Astra {
public:
    MyEncoder(const char *name = "MyEncoder");

protected:
    bool init() override;
    bool read() override;

private:
    int pinA;
    int pinB;
    volatile int rawSteps;
};

}

#endif
```

### Implementation Example

```cpp
#include "MyEncoder.h"

using namespace astra;

MyEncoder::MyEncoder(const char *name)
    : Encoder_Astra(name), pinA(2), pinB(3), rawSteps(0) {}

bool MyEncoder::init() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    // Attach interrupts for quadrature decoding
    attachInterrupt(digitalPinToInterrupt(pinA),
                    [this](){ handleInterrupt(); }, CHANGE);

    initialSteps = 0;
    LOGI("MyEncoder: Initialized");
    return true;
}

bool MyEncoder::read() {
    // Update relative steps
    currentRelativeSteps = rawSteps - initialSteps;
    return true;
}

void MyEncoder::handleInterrupt() {
    // Quadrature decoding logic
    bool stateA = digitalRead(pinA);
    bool stateB = digitalRead(pinB);

    if (stateA == stateB) {
        rawSteps++;
    } else {
        rawSteps--;
    }
}
```

---

## Usage Example

```cpp
#include "Sensors/Encoder/AstraEncoder.h"

MyEncoder wheelEncoder("Wheel");

void setup() {
    if (!wheelEncoder.begin()) {
        Serial.println("Encoder init failed!");
    }

    // Zero the encoder
    wheelEncoder.setInitialSteps(0);
}

void loop() {
    wheelEncoder.update();

    int steps = wheelEncoder.getSteps();
    float rotations = steps / 2400.0;  // For 2400 PPR encoder

    Serial.print("Steps: ");
    Serial.print(steps);
    Serial.print(" (");
    Serial.print(rotations);
    Serial.println(" rotations)");

    delay(100);
}
```

---

## Integration with State

Encoders can be used for odometry or actuation feedback:

```cpp
MyEncoder leftWheel("LeftWheel");
MyEncoder rightWheel("RightWheel");

Sensor *sensors[] = {&leftWheel, &rightWheel};
State vehicleState(sensors, 2, nullptr);

void loop() {
    system.update();

    int leftSteps = leftWheel.getSteps();
    int rightSteps = rightWheel.getSteps();

    // Calculate differential drive odometry
    float avgSteps = (leftSteps + rightSteps) / 2.0;
    float heading = (rightSteps - leftSteps) / wheelbaseSteps;
}
```

---

## Common Applications

1. **Wheel Odometry**: Track distance traveled
2. **Motor Position Control**: Closed-loop position feedback
3. **Gimbal Tracking**: Measure actuator position
4. **Rotation Counting**: Count revolutions

---

## Summary

- `Encoder_Astra` tracks rotational position via step counting
- Implement `init()` and `read()` for hardware-specific encoders
- Use `setInitialSteps()` to zero or re-reference position
- Useful for odometry, position control, and rotation tracking

For general sensor information, see the [Sensor Interface](../sensor.md) documentation.

---
