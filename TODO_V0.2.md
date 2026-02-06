# TODO - Version 0.2 (Active)

Last updated: 2026-02-06

## Release Goal
- Release ASAP once blockers are complete.

## Release Blockers

### Documentation
None (completed).

### Examples
- Add data-logger-only example (no state estimation).
- Add minimal sensor setup example (accel + gyro).
- Update existing examples to the v0.2 API and add comments.
- Test examples on all platforms.

### Tests
None (completed).

## Completed in V2 (Core)
- Architecture refactor: State is pure math, Astra orchestrates, SensorManager internal.
- Event-driven sensor updates and update ordering.
- Error handling with `int` error codes.
- Status indicators (LED and buzzer) via BlinkBuzz.
- Sensor health monitoring.
- HITL/SITL architecture and SerialMessageRouter.

## Recently Completed
- Removed deprecated State API, tests, and docs.
- Cleaned lingering deprecated markers in code.
- Documentation updates: README architecture + quick start + platform setup, API reference (State vectors, AstraConfig, ownership, SerialMessageRouter), and getting started (step-by-step, patterns, command guide).
- Replaced `updateMeasurements` with per-sensor GPS/baro updates.
- Normalized hardware sensor constructors (`name`, `TwoWire*`, address order).
- Swapped health tracking buffers to `CircBuffer` in core sensors.
- Fixed BlinkBuzz `isUsingAsync()` to return `enableAsync`.
- Added/confirmed native tests for math components (Kalman filter edge cases, Mahony extreme inputs, coordinate transforms).
- Added/confirmed mock sensor tests for State behavior and fusion.
- Added integration tests for HITL scenarios and update ordering.

## Deferred to V3 (Out of V2 Scope)
- Config persistence and validation.
- Calibration system.
- File system commands and retrieval cleanup (FileReader and SerialHandler tasks).
- PWM support in BlinkBuzz.
- Matrix and Vector swap to Gemini versions.
- `dt` vs `currentTime` decision for sensors and state (decision deferred to v3).
- AstraRocket improvements (separate repo).
