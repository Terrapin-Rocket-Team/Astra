# FAQ

## What coordinate frame does State use?

`State` uses **ENU** (East, North, Up).  
If you see references to **NEU** in older docs, that is a documentation error.

## Does the IMU provide orientation?

No. IMU classes return **raw sensor data** only.  
Orientation is estimated by `MahonyAHRS` inside `State`.

## Can I reconfigure logging after init?

Yes. You can call:

```cpp
EventLogger::configure(eventSinks, count);
DataLogger::configure(dataSinks, count);
```

This is useful if a sink becomes available later (e.g., USB or radio connection).

## Do I need to call `SerialMessageRouter::update()`?

Only if you are using it standalone.  
If you use the full Astra system, it updates the router internally.

