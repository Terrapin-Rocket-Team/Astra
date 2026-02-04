# TODO - Version 0.2

## Hardware Integration & Sensors
- [X] Voltage Sensor code
- [X] Add GPS velocity (NED frame)
- [ ] Add Radio Sensor (for GPS eventually)
- [ ] Add STM32 EMMC support
- [ ] Change init/read to int instead of bool?

## User Feedback Systems
- [ ] Proper buzzer/LED feedback systems
- [ ] Teensy CrashReport

## Radio & Communication
- [X] Radio integration (Fix transmit and logging issues)
- [ ] Data retrieval

## HITL (Hardware-in-the-Loop)
- [X] Better HITL initializing
- [X] Better HITL telem trading
- [X] Figure out a way to add noise to OR
- [X] Use real sensors for base in HITL and have OR "add" values to what those sensors read??

## Sensor Fusion & Filtering

### Mahony Filter
- [X] Unit test Mahony (At least Python visual test)
- [X] Mahony magnetometer integration
- [X] Initial Tilt estimation

### Kalman Filter
- [X] KF testing
- [X] KF rewrite for accelerometer

## Flight Computer Features
- [ ] Add apogee time and altitude prediction
- [X] Add drogue deploy detection
- [X] Add main deploy detection

## Data Handling & Logging
- [X] Fix `withOtherDataReporters`
- [X] Add debug log

## Build & Deployment
- [X] Auto-increment version based on build/SHA
- [ ] Reduce init verbosity/double init
- [X] Clean up comments in files
