# Troubleshooting

## “No State provided” warning

If you do not call `withState()`, Astra will create a `DefaultState` automatically.  
If you want a custom filter stack, pass your own `State`.

---

## Telemetry header missing or incomplete

Make sure all `DataReporter` objects are constructed **before** `Astra::init()` so the header includes their columns.

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

`State` uses **ENU** (East, North, Up).  
Some older docs mention NEU or NED; those are outdated.

