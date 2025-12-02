# BlinkBuzz

The `BlinkBuzz` utility provides asynchronous LED and buzzer pattern control without blocking your main loop. It handles simple patterns like beeping a fixed number of times, complex patterns like SOS in Morse code, and repeating patterns that run indefinitely. The utility is designed to be used as-is without modification or subclassing.

---

## Overview

BlinkBuzz operates in two modes:

1. **Synchronous** - Blocking calls that hold execution until the pattern completes
2. **Asynchronous** - Non-blocking patterns that execute in the background while your code continues

The asynchronous mode is the primary use case, allowing status indication (stage changes, GPS lock, errors) without interrupting critical sensor updates or state estimation.

---

## Setup

BlinkBuzz can be used standalone or with `AstraConfig`. The only difference is initialization—all other functions work identically.

### With AstraConfig (Recommended)

```cpp
#include <Astra.h>

int GPS_STATUS_PIN = 25;

AstraConfig config = AstraConfig()
                    .withBuzzerPin(13)              // Buzzer on pin 13
                    .withBBPin(LED_BUILTIN)         // LED on built-in pin
                    .withBBPin(GPS_STATUS_PIN)      // Add as many pins as needed
                    .withBBAsync(true, 50);         // Enable async, queue size 50

Astra system(&config);

void setup() {
    system.init();  // Initializes BlinkBuzz automatically
}

void loop() {
    system.update();  // Updates async patterns automatically
}
```

**Configuration options:**

| Method | Description | Default |
|--------|-------------|---------|
| `withBuzzerPin(int)` | Set buzzer pin | None (required) |
| `withBBPin(int)` | Add LED/buzzer pin | None (add as many as needed) |
| `withBBAsync(bool, int)` | Enable async mode with queue size | `(false, 0)` |

### Without AstraConfig

If you prefer standalone usage, initialize manually:

```cpp
#include <BlinkBuzz.h>

int allowedPins[] = {LED_BUILTIN, 33};
BlinkBuzz bb;  // Global variable for easy access

double lastTime = 0;

void setup() {
    bb.init(allowedPins, 2, true, 50);
    // Args: pin array, pin count, enable async, queue size per pin
}

void loop() {
    bb.update();  // Call frequently for accurate timing

    // Avoid delay() when using async patterns
    double time = millis();
    if (time - lastTime < 100)  // 100ms loop interval
        return;
    lastTime = time;

    // Rest of your loop code
}
```

!!! warning "Memory Overhead"
    Async patterns use queues to store ON/OFF toggle events. Each pattern adds to the queue (e.g., 3 beeps = 6 toggles: ON, OFF, ON, OFF, ON, OFF). Keep queue sizes reasonable to avoid excessive memory usage.

---

## Synchronous Usage

Synchronous calls block execution until the pattern completes. Use these in `setup()` for initialization feedback, or when async mode is disabled.

```cpp
#include <BlinkBuzz.h>

void setup() {
    // Hold pin on/off
    bb.on(BUZZER);   // Turn on
    bb.off(BUZZER);  // Turn off

    // Simple patterns
    bb.onoff(BUZZER, 200);          // Single 200ms beep
    bb.onoff(BUZZER, 200, 5);       // 5 beeps: 200ms on, 200ms off
    bb.onoff(BUZZER, 200, 3, 100);  // 3 beeps: 200ms on, 100ms off
}
```

**Parameters:**

| Function | Parameters | Description |
|----------|------------|-------------|
| `on(pin)` | `pin` | Turn pin on indefinitely |
| `off(pin)` | `pin` | Turn pin off |
| `onoff(pin, dur)` | `pin`, `dur` | Single pulse of `dur` ms |
| `onoff(pin, dur, reps)` | `pin`, `dur`, `reps` | Repeat `reps` times, equal on/off |
| `onoff(pin, dur, reps, off)` | `pin`, `dur`, `reps`, `off` | Repeat `reps` times, `off` ms between |

!!! note "Synchronous Limitations"
    `BBPattern` objects are **not** supported in synchronous mode. Use async mode for complex patterns.

---

## Asynchronous Usage

Async mode is where BlinkBuzz shines. Patterns execute in the background without blocking, making them perfect for status indication during flight.

### Basic Async Patterns

```cpp
// Single beep
bb.aonoff(BUZZER, 200);

// 5 beeps, 200ms on/off
bb.aonoff(BUZZER, 200, 5);

// 3 beeps, 200ms on, 100ms off
bb.aonoff(BUZZER, 200, 3, 100);

// Clear pending patterns on a pin
bb.clearQueue(BUZZER);
```

**Parameters match synchronous `onoff()`, but with `a` prefix for async.**

### Complex Patterns with BBPattern

The `BBPattern` class builds complex patterns by combining simpler ones. You can chain patterns together and add "rests" (pauses) between repetitions.

**Basic BBPattern usage:**

```cpp
BBPattern pattern = BBPattern(ON_DURATION, REPEATS, OFF_DURATION);

// Example: SOS in Morse code
BBPattern s(50, 3, 200);   // S: 3 short beeps
BBPattern o(500, 3, 200);  // O: 3 long beeps

// Combine patterns using a() (append)
BBPattern sos;
sos.a(s).a(o).a(s);

// Execute the pattern
bb.aonoff(BUZZER, sos);
```

**Repeating patterns indefinitely:**

```cpp
// Repeat SOS pattern forever
bb.aonoff(BUZZER, sos, true);

// Add 1 second rest between repetitions
bb.aonoff(BUZZER, sos.r(1000), true);
```

!!! warning "Pattern Construction"
    Do **not** use `BBPattern sos = s.a(o).a(s);` — this won't work as expected due to how chaining is implemented. Instead, create an empty pattern and append to it:
    ```cpp
    BBPattern sos;
    sos.a(s).a(o).a(s);  // Correct
    ```

**BBPattern methods:**

| Method | Description |
|--------|-------------|
| `BBPattern(dur, reps, off)` | Create pattern: `dur` ms on, `reps` times, `off` ms between |
| `a(pattern)` | Append another pattern |
| `r(duration)` | Add rest (pause) at the end |

---

## Practical Examples

### GPS Lock Indicator

```cpp
int GPS_PIN = 25;

void loop() {
    system.update();

    if (gps.getHasFix()) {
        // Solid on when locked
        bb.on(GPS_PIN);
    } else {
        // Slow blink when searching
        bb.aonoff(GPS_PIN, 500, 1);  // 500ms on, 500ms off, repeats
    }
}
```

### Stage Change Notifications

```cpp
class CustomState : public State {
    void update(double currentTime) override {
        State::update(currentTime);

        int newStage = determineStage();
        if (newStage != stage) {
            stage = newStage;

            // Beep N times for stage N
            bb.aonoff(BUZZER, 200, stage);

            LOGI("Entered stage %d", stage);
        }
    }
};
```

### Armed Status Pattern

```cpp
void setup() {
    system.init();

    // Armed: rapid beep pattern
    BBPattern armed(100, 3, 50);  // 3 quick beeps
    bb.aonoff(BUZZER, armed.r(2000), true);  // Repeat every 2s

    LOGI("System armed");
}
```

### Complex Morse Code Patterns

```cpp
// Morse: -- --- -.- (M O K)
BBPattern dash(300, 1, 100);
BBPattern dot(100, 1, 100);
BBPattern letterGap(0, 1, 300);

BBPattern m;
m.a(dash).a(dash).a(letterGap);

BBPattern o;
o.a(dash).a(dash).a(dash).a(letterGap);

BBPattern k;
k.a(dash).a(dot).a(dash).a(letterGap);

BBPattern mok;
mok.a(m).a(o).a(k);

bb.aonoff(BUZZER, mok, true);  // Repeat "MOK" indefinitely
```

---

## Queue Management

Each pin has its own queue of pending toggles (ON/OFF transitions). Queue size is set during initialization.

**Queue considerations:**

- Each `aonoff(pin, dur, reps)` adds `2 * reps` events to the queue (ON + OFF per repetition)
- Repeating patterns (`aonoff(pin, pattern, true)`) continuously refill the queue
- Use `clearQueue(pin)` to cancel all pending patterns on a pin
- If the queue fills, new patterns are rejected (logged as warning)

**Example:**

```cpp
bb.aonoff(BUZZER, 200, 3);  // Adds 6 events: ON, OFF, ON, OFF, ON, OFF

// Cancel before completion
bb.clearQueue(BUZZER);  // Remaining events discarded
```

---

## Best Practices

1. **Call `update()` frequently**: The more often `update()` is called, the more accurate pattern timing will be. Avoid long delays in your loop.

2. **Avoid `delay()` with async mode**: Use time-based loop control instead:
   ```cpp
   double lastTime = 0;
   void loop() {
       bb.update();

       double time = millis();
       if (time - lastTime < 100) return;
       lastTime = time;

       // Your loop code
   }
   ```

3. **Size queues appropriately**: Use the minimum queue size needed for your patterns. Typical values: 20-50 per pin.

4. **Clear queues on stage transitions**: Prevent old patterns from interfering with new ones:
   ```cpp
   if (stageChanged) {
       bb.clearQueue(BUZZER);
       bb.aonoff(BUZZER, 200, newStage);  // Beep for new stage
   }
   ```

5. **Test patterns in `setup()`**: Use synchronous patterns during initialization to verify hardware:
   ```cpp
   void setup() {
       system.init();
       bb.onoff(BUZZER, 100, 2);  // Sync beeps confirm buzzer works
   }
   ```

6. **Use multiple pins for different statuses**: Dedicate pins to specific indicators (GPS, stage, errors) for clear status at a glance.

---

## Common Patterns

### Heartbeat

```cpp
// Continuous heartbeat: quick double beep, long pause
BBPattern heartbeat(50, 2, 100);
bb.aonoff(LED_BUILTIN, heartbeat.r(2000), true);
```

### Error Alarm

```cpp
// Rapid continuous beeping
bb.aonoff(BUZZER, 100, -1);  // Infinite repetitions (use clearQueue to stop)
```

### Count-Up Sequence

```cpp
// Beep 1, 2, 3, 4 times with pauses
for (int i = 1; i <= 4; i++) {
    BBPattern count(200, i, 150);
    bb.aonoff(BUZZER, count.r(1000));
}
```

---

## Limitations

- **No pattern modification**: Once a pattern enters the queue, it cannot be changed. Use `clearQueue()` and resubmit.
- **Fixed queue sizes**: Queue size is set at initialization and cannot grow dynamically.
- **Synchronous patterns don't support BBPattern**: Only async mode supports `BBPattern` objects.
- **No priority system**: Patterns execute in submission order. Use `clearQueue()` for urgent patterns.

---

## Summary

- `BlinkBuzz` provides non-blocking LED and buzzer control
- Use `aonoff()` for simple async patterns
- Use `BBPattern` to build complex patterns from simpler components
- Call `update()` frequently for accurate timing
- Use `clearQueue()` to cancel pending patterns
- Integrate with `AstraConfig` for automatic initialization and updates

For integration with the main system, see the [Basic Use](../basic-use.md) guide.

---
