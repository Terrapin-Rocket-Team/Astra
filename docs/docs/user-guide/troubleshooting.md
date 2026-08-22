# Troubleshooting

## “No State provided” warning

If you do not call `withState()`, Astra will create a `DefaultState` automatically.  
If you want a custom filter stack, pass your own `State`.

---

## Telemetry header missing or incomplete

Add every standalone `DataReporter` with `AstraConfig::withReporter()` before
`Astra::init()`. Astra automatically registers the sensors and `State` supplied
through its configuration.

---

## No logs coming out

- Ensure you configured sinks:
  - `withEventLogs()` or `EventLogger::configure()` for event logs
  - `withDataLogs()` for telemetry
- Check that your sink returns `ok() == true`

---

## GPS shows no fix

`GPS::getHasFix()` only returns true when fix quality ≥ 4.  
You may need a clear sky view and time to lock.

---

## Coordinate frame confusion

`State` outputs use **ENU** (East, North, Up). Raw GPS velocity from
`GPS::getVel()` is NED and is converted before it reaches the State outputs.

