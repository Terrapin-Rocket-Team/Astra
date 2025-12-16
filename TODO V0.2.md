# TODO - Version 0.2

## Hardware Integration & Sensors
- [X] Voltage Sensor code
- [X] Add GPS velocity (NED frame)
- [ ] Add Radio Sensor (for GPS eventually)
- [ ] Add STM32EMMC support

## User Feedback Systems
- [ ] Proper buzzer/LED feedback systems

## Radio & Communication
- [ ] Radio integration (Fix transmit and logging issues)
- [ ] Data retrieval

## HITL (Hardware-in-the-Loop)
- [ ] Better HITL initializing
- [ ] Better HITL telem trading
- [ ] Figure out a way to add noise to OR
- [ ] Use real sensors for base in HITL and have OR "add" values to what those sensors read??

## Sensor Fusion & Filtering

### Mahony Filter
- [ ] Unit test Mahony (At least Python visual test)
- [ ] Mahony magnetometer integration

### Kalman Filter
- [ ] KF testing
- [ ] KF rewrite for accelerometer

## Flight Computer Features
- [ ] Add apogee time and altitude prediction
- [ ] Add drogue deploy detection
- [ ] Add main deploy detection

## Data Handling & Logging
- [ ] Fix `withOtherDataReporters`
- [ ] Add debug log

## Build & Deployment
- [ ] Auto-increment version based on build/SHA
- [ ] Reduce init verbosity/double init
