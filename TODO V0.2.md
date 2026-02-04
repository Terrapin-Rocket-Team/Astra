# TODO - Version 0.2

## Hardware Integration & Sensors
- [X] Voltage Sensor code
- [X] Add GPS velocity (NED frame)
- [ ] Add Radio Sensor (for GPS eventually)
- [ ] Add STM32 EMMC support
- [ ] Change init/read to int instead of bool?
- [ ] Add servo and motor control interfaces

## User Feedback Systems
- [ ] Proper buzzer/LED feedback systems
- [ ] Teensy CrashReport

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
- [ ] Initial Tilt estimation

### Kalman Filter
- [ ] KF testing
- [ ] KF rewrite for accelerometer

## Flight Computer Features
- [ ] Add apogee time and altitude prediction
- [ ] Add drogue deploy detection
- [ ] Add main deploy detection
- [ ] Add pyro charge deployment
- [ ] Add multi stage state detection

## Data Handling & Logging
- [X] Fix `withOtherDataReporters`
- [X] Add a DataReporter callback interface for single item logging
- [ ] Add debug log

## Build & Deployment
- [X] Auto-increment version based on build/SHA
- [ ] Reduce init verbosity/double init
- [ ] Clean up comments in files

## General
- [ ] Add a scheduler to the update loop to detect hung or crashed tasks
- [ ] Interactive flight data log viewer
- [ ] Create a more comprehensive example set
- [ ] Create a GUI configurator tool (and by extension have Astra be able to read from a config file)