# TODO - Version 0.2

## ✅ JUST COMPLETED (2026-02-04)

**Major architectural refactoring complete!**
- State is now pure math - takes vectors as parameters, no hardware dependencies
- AstraConfig owns SensorManager - user never creates it manually
- Astra orchestrates data flow - fetches from sensors, passes to State
- Added IMU convenience methods: `with6DoFIMU()` and `with9DoFIMU()`
- Removed all `withSensorManager()` calls - breaking change, no backwards compat
- Example code updated to new API
- **Result:** Clean separation of concerns, code compiles successfully

**What's Next:**
- Remove Mahony rocket-specific logic (MahonyMode enum, lockFrame, high-G switching)
- Event-driven sensor updates (remove fixed intervals)
- Error handling overhaul (status enums instead of bools)

---

## 🔥 CRITICAL - Architecture Refactor (Do First)

### 1. Decouple State/SensorManager/Astra - Make Astra the Orchestrator
**Problem:** State requires SensorManager, accesses sensors directly, too tightly coupled
**Goal:** State takes vectors (pure math), Astra orchestrates data flow, SensorManager hidden from user

- [X] **Add convenience methods to SensorManager**
  ```cpp
  Vector<3> getAcceleration() const;
  Vector<3> getAngularVelocity() const;
  Vector<3> getMagneticField() const;
  double getBarometricAltitude() const;
  Vector<3> getGPSPosition() const;
  Vector<3> getGPSVelocity() const;
  ```
  - Cleaner than `getAccelSource()->getAccel()`
  - Returns safe defaults if sensor not present

- [X] **Refactor State to take vectors**
  - Remove `withSensorManager(SensorManager*)`
  - Remove `SensorManager* sensorManager` member
  - Remove all `sensorManager->getXxxSource()->getXxx()` calls
  - Add methods that take vectors:
    ```cpp
    void updateOrientation(Vector<3> gyro, Vector<3> accel, double dt);
    void predict(Vector<3> earthAccel, double dt);
    void updateMeasurement(Vector<3> gpsPos, double baroAlt);
    ```
  - State is now pure math, no hardware knowledge

- [X] **Make Astra own SensorManager internally**
  - Move `SensorManager sensorManager;` into Astra (owned, not pointer)
  - Remove `withSensorManager()` from AstraConfig
  - Add sensor methods to AstraConfig:
    ```cpp
    AstraConfig& withAccel(Accel* a);
    AstraConfig& withGyro(Gyro* g);
    AstraConfig& withBaro(Barometer* b);
    AstraConfig& withGPS(GPS* g);
    AstraConfig& withMiscSensor(Sensor* s);
    ```
  - Astra sets up internal SensorManager from config
  - User never creates SensorManager manually
  - Optional: `Astra::getSensorManager()` for advanced access

- [X] **Astra orchestrates data flow**
  - Astra calls `sensorManager.update()`
  - Astra fetches vectors from SensorManager
  - Astra passes vectors to State methods
  - State no longer pulls data, receives data
  - Clean separation: Astra = orchestrator, State = math, SensorManager = hardware

- [X] **Clarify Memory Ownership**
  - Document ownership model in headers:
    ```cpp
    // Astra OWNS: SensorManager (internal)
    // Astra NON-OWNING: State*, Sensors* (passed via config)
    // User OWNS: Sensors, State, Filters
    ```

- [ ] **Remove Mahony Rocket-Specific Logic**
  - Remove `lockFrame()` - user/RocketState calls it
  - Remove `MahonyMode` enum (CALIBRATING/CORRECTING/GYRO_ONLY)
  - Make Mahony pure AHRS with tunable params
  - Push high-G switching logic to State or RocketState
  - User controls mode transitions explicitly

### 2. New Sensor API - Event-Driven Updates
**Problem:** Fixed update rates don't match sensor capabilities
**Goal:** Sensors update when data is ready, not on a schedule

- [ ] **Remove fixed update intervals** from Astra
  - Sensor updates: Run every loop iteration, let `shouldUpdate()` decide
  - Prediction: Run every loop iteration at max speed
  - Measurement: Run when new sensor data available (flag-based)
  - Logging: Keep at configurable rate (storage constraint)

- [ ] **Fix update ordering** in Astra::update()
  - Enforce: Sensors → Prediction → Measurement → Logging
  - No race conditions between updates in same iteration

### 3. Error Handling Overhaul
- [ ] **Change `bool` returns to status enums**
  ```cpp
  enum class SensorStatus {
      OK,
      I2C_TIMEOUT,
      DEVICE_NOT_FOUND,
      SELF_TEST_FAILED,
      INVALID_CONFIG
  };
  ```
- [ ] **Better error messages** with actionable info
  - "BMI088 init failed: I2C device not found at 0x18. Check wiring and I2C pullups."
  - Not just "Sensor init failed"

---

## 🎯 HIGH PRIORITY - Core Features

### 4. Config System Polish
- [ ] **Config persistence** - Save/load from file
  ```cpp
  config.saveToFile("/config.json");
  AstraConfig config = AstraConfig::loadFromFile("/config.json");
  ```
- [ ] **Config validation** - Check before flight
  ```cpp
  std::vector<String> warnings = config.validate();
  ```

### 5. SITL/HITL Polish
- [ ] **Abstract HITL logic** out of Astra::update()
  - Create HITLController class
  - Handles time injection, sensor mocking
  - Cleaner separation

- [ ] **Better SITL connection** handling
  - Auto-reconnect on socket drop
  - Cleaner handshake protocol

### 6. Calibration System
- [ ] **Guided calibration workflow**
  ```cpp
  Calibration cal;
  cal.calibrateGyro(2.0);    // 2 second still period
  cal.calibrateAccel();      // 6-position calibration
  cal.calibrateMag();        // Mag dance
  cal.saveToFile();
  cal.loadFromFile();
  ```
- [ ] Medium priority - defer to v0.3 if time-constrained

---

## 🔧 MEDIUM PRIORITY - Quality of Life

### 7. LED/Buzzer Framework Decision
- [ ] **Decide:** Base Astra or AstraRocket?
  - Recommendation: Keep simple LED/buzzer in base (status feedback)
  - Move complex patterns to AstraRocket (flight-specific)

### 8. AstraRocket Improvements
- [X] **Sensor manager convenience methods**
  ```cpp
  config.withAccel(&accel)
        .withGyro(&gyro)
        .withBaro(&baro)
        .withGPS(&gps);
  ```
  - AstraRocket creates SensorManager internally
  - User doesn't touch SensorManager directly

- [X] **Make thresholds configurable**
  ```cpp
  config.withLiftoffAccelThreshold(3.0)
        .withDrogueDescentMin(5.0)
        .withMainDeployAltitude(400.0);
  ```

### 9. DataLogger Singleton - Keep But Document
- [ ] **Add comment explaining singleton choice**
  - One logger per system is appropriate
  - LOGI/LOGE/LOGW convenience macros justify it
  - Document that it's intentional, not oversight

- [X] **Optional:** Allow injecting logger for testing
  ```cpp
  EventLogger::setInstance(&mockLogger); // For tests only
  ```

---

## 📊 LOW PRIORITY - Future Features

### 10. Health Monitoring
- [ ] System health checks
  - `isIMUHealthy()` - detect stuck readings
  - `isGPSHealthy()` - check fix quality
  - `isStorageHealthy()` - detect write failures

### 11. Pyro Control (AstraRocket)
- [ ] PyroController class
- [ ] Arming/disarming logic
- [ ] Continuity checks
- [ ] Defer until needed for flight

### 12. Event System - MAYBE
**Concern:** C++ callback limitations make this bloated
**Decision:** Defer unless compelling use case emerges
- Flight statistics → Ground station responsibility
- Event callbacks → May not be worth complexity

---

## ✅ COMPLETED (from previous TODO)
- [X] Voltage Sensor code
- [X] GPS velocity (NED frame)
- [X] Radio integration fixes
- [X] HITL improvements
- [X] Mahony magnetometer integration
- [X] Mahony initial tilt estimation
- [X] KF testing and rewrite
- [X] Drogue/main deploy detection
- [X] Fix withOtherDataReporters
- [X] Debug log
- [X] Auto-increment version
- [X] Clean up comments

---

## 🗑️ REJECTED / DEFERRED

### Not Doing (Rationale)
- ❌ **Preset configs** - That's what derived libraries (AstraRocket) are for
- ❌ **Smart pointers** - Hard on embedded, document ownership instead
- ❌ **Validation helper** - Low value, config validation is enough
- ❌ **Flight statistics in base** - Ground station responsibility

---

## 🎯 RECOMMENDED ORDER OF ATTACK

**Week 1: Architecture Cleanup (Critical)**
1. Mahony filter - strip rocket logic
2. State/SensorManager decoupling via DataSource
3. Memory ownership documentation

**Week 2: Update System Refactor**
4. Remove fixed update intervals
5. Event-driven sensor updates
6. Fix update ordering

**Week 3: Error Handling & Config**
7. Status enums instead of bools
8. Better error messages
9. Config persistence and validation

**Week 4: Polish**
10. HITL/SITL cleanup
11. AstraRocket convenience methods
12. Documentation updates

---

## 💭 NOTES

**On Feeling Bloated:**
The bones ARE good. The coupling is making it feel unwieldy. Once State doesn't know about SensorManager and Mahony isn't rocket-specific, it'll feel much cleaner.

**On Singleton:**
DataLogger/EventLogger as singletons are fine. The convenience macros justify it. One logger per system is appropriate.

**On Event System:**
C++ callback limitations (can't easily bind member functions) make this more trouble than it's worth. Current logging is sufficient.

**On Burnout:**
Start with #1-3 (architecture cleanup). Once the coupling is fixed, the rest will flow easier. The paralysis comes from knowing something's wrong but not seeing the path forward. This list IS the path.

---

## 📊 LOW PRIORITY - Deferred Until API Stable

### 13. STM32 eMMC Support & File System CLI
**Goal:** Use STM32 SDMMC peripheral with eMMC chip for onboard storage
- [ ] **Adapt STM32Duino SD library for eMMC**
  - Copy and modify SD HAL wrapper for SDMMC+eMMC
  - Test on STM32 hardware with eMMC chip
- [ ] **Command-line file system interface**
  - `CMD/LS [path]` - list files/directories
  - `CMD/CAT <file>` - stream file contents to current ILogSink
  - `CMD/RM <file>` - delete file
  - `CMD/CP <src> <dest>` - copy file (optional)
  - `CMD/DF` - disk free space
- [ ] **Integration with config persistence**
  - Enable `config.saveToFile("/emmc/config.json")`
  - Enable `config.loadFromFile("/emmc/config.json")`
- **Priority:** Medium-Low - Wait until Section 4 (Config System) is complete

### 14. Teensy CrashReport Integration
- [ ] **Auto-capture crashes to log**
  - Hook Teensy CrashReport on boot
  - Log crash info via EventLogger on next startup
  - Optionally write to SD/eMMC for persistence
- [ ] **CLI command to view crash**
  - `CMD/CRASH` - display last crash report
  - `CMD/CRASH CLEAR` - clear crash report
- **Priority:** Low - Nice-to-have for debugging

### 15. Unit Testing - DEFER UNTIL v0.3
**Reason:** Wait for API to stabilize before writing tests
- [ ] **Native platform tests for math components**
  - Kalman filter prediction/update
  - Mahony orientation estimation
  - Coordinate transformations
- [ ] **Mock sensor tests**
  - Test State with simulated sensor data
  - Verify sensor fusion behavior
- [ ] **Integration tests**
  - HITL test scenarios
  - Verify update ordering
- **Priority:** Low - Defer to v0.3 after API stabilizes

### 16. Better Examples - DEFER UNTIL v0.3
**Reason:** Wait for API to stabilize, then showcase features
- [ ] **Data logger only** (no state estimation)
- [ ] **Drone/quadcopter** (different state model)
- [ ] **Ground vehicle** (2D state estimation)
- [ ] **Weather balloon** (altitude focus)
- [ ] **Minimal sensor setup** (accel+gyro only)
- **Priority:** Low - Defer to v0.3 after API stabilizes

### 17. Documentation Updates - DEFER UNTIL v0.3
**Note:** No migration guide needed - breaking changes are intentional
- [ ] **Updated README**
  - New architecture overview (Astra orchestrates, State is pure math)
  - Quick start with new API
  - Removed SensorManager from user-facing docs
- [ ] **API Reference**
  - Document new State vector-based methods
  - Document AstraConfig sensor methods
  - Document ownership model
- [ ] **Tutorial/Getting Started**
  - Step-by-step setup with new API
  - Common patterns (6DoF IMU, GPS+Baro, etc.)
- **Priority:** Medium - Wait until API stabilizes before documenting
