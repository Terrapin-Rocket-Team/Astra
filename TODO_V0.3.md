# TODO - Version 0.3 Roadmap

## 🎯 VERSION 0.3 GOALS

**Theme:** Configuration, Commands, and File Management

Version 0.3 focuses on making the system more usable in the field:
- Persistent configuration (no recompiling to change parameters)
- Robust command system (control the vehicle via serial)
- File management (retrieve data, manage storage)

---

## ✅ PREREQUISITES (v0.2)

- See TODO_V0.2.md for documentation, examples, and testing blockers that must be complete before v0.3 can ship.

---

## 🔥 CRITICAL FEATURES (Must Have for v0.3)

### 1. Config System - Persistence & Validation

**Goal:** Save/load configuration from files, enable field tuning without recompiling

**Phase 1: Basic Persistence** (Blocking for other features)
- [ ] Add `loadFromFile()` to AstraConfig
  - Read JSON config from storage
  - Apply to Astra/State/Filters at init
  - Use ArduinoJson library
- [ ] Add `saveToFile()` to AstraConfig
  - Write current config to JSON
  - Preserve comments/formatting
- [ ] Define config file format (JSON)
  - Sensor calibration (gyro bias, accel matrix, mag cal)
  - Algorithm tuning (Mahony Kp/Ki, Kalman noise)
  - Logging parameters (rate, enabled sinks)
  - Hardware config (I2C addresses, pins)
- [ ] Boot sequence integration
  - Check for config file at startup
  - Load if present, use defaults if missing
  - Log config load status
- [ ] Example config files
  - Default config (conservative settings)
  - High-rate config (fast logging)
  - Minimal config (accel+gyro only)

**Phase 2: Runtime Modification**
- [ ] Add `CONFIG/` command handler to SerialMessageRouter
  - `CONFIG/LOAD` - Reload config from file
  - `CONFIG/SAVE` - Save current config to file
  - `CONFIG/GET <key>` - Query config value
  - `CONFIG/SET <key>=<value>` - Modify config value
  - `CONFIG/LIST` - List all config keys
- [ ] Hot-reloadable parameters (no reboot required)
  - Mahony Kp/Ki gains
  - Kalman noise parameters
  - Logging rate
  - Calibration offsets
- [ ] Cold parameters (require restart)
  - Sensor selection
  - I2C addresses
  - Pin assignments
  - Mark with warning on SET

**Phase 3: Validation**
- [ ] Add `validate()` method to AstraConfig
  - Check for invalid values (negative rates, etc.)
  - Check for missing required config
  - Check for incompatible settings
  - Return list of warnings/errors
- [ ] `CONFIG/VALIDATE` command
  - Run validation on current config
  - Report issues to serial
- [ ] Pre-flight validation
  - Call validate() before arming/launch
  - Refuse to arm if critical errors

**Priority:** CRITICAL - Blocks calibration system and file CLI
**Estimated Effort:** 3-5 days

---

### 2. Command System - Core Commands

**Goal:** Implement essential commands for vehicle control and diagnostics

**Basic Commands:**
- [ ] `CMD/STATUS` - System status dump
  - Sensor health
  - GPS fix status
  - Storage space
  - Uptime
  - Battery voltage (if available)
- [ ] `CMD/PING` - Connectivity test
  - Respond with `PONG` + timestamp
- [ ] `CMD/REBOOT` - Software reset
  - Save any pending data
  - Trigger watchdog or soft reset
- [ ] `CMD/LOGLEVEL <level>` - Change logging verbosity
  - DEBUG, INFO, WARN, ERROR levels
- [ ] `CMD/HEADER` - Already works! ✅

**Response Protocol:**
- [ ] Standardize response format
  - `ACK/<command>/[data]` - Success
  - `NAK/<command>/<error_code>` - Failure
  - `STATUS/<field>/<value>` - Async status
- [ ] Error codes enum
  - INVALID_COMMAND
  - INVALID_ARGS
  - HARDWARE_ERROR
  - PERMISSION_DENIED
  - etc.

**Priority:** HIGH - Needed for field operations
**Estimated Effort:** 2-3 days

---

### 3. File System Commands

**Goal:** Manage files on SD/eMMC via serial commands

**File Management Commands:**
- [ ] `FILE/LIST [path]` - List files/directories
  - Default path: root
  - Show filename, size, modified date
  - CSV format for parsing
- [ ] `FILE/GET <filename>` - Stream file contents
  - Use existing BOF/EOF markers
  - Optional: Add chunk numbers for reliability
- [ ] `FILE/DELETE <filename>` - Remove file
  - Confirmation via ACK/NAK
- [ ] `FILE/SIZE <filename>` - Get file size
  - Returns size in bytes
- [ ] `FILE/STATS` - Storage statistics
  - Total capacity
  - Used space
  - Free space
  - Number of files

**File Transfer Improvements:**
- [ ] Unify RetrieveSDCardData and FileReader
  - Choose one implementation
  - Delete or fully integrate the other
- [ ] Decide fate of legacy SerialHandler stubs
  - Remove or replace with SerialMessageRouter-based handlers
- [ ] Resolve RetrieveSDCardData TODOs
  - Define readFile API (streaming vs buffer return)
  - Implement or remove `formatCard()` stub
- [ ] Clarify or remove FileReader (purpose and ownership)
- [ ] Register file handler with SerialMessageRouter
  - `router->withListener("FILE/", handleFileCommand);`
- [ ] Progress reporting for large files
  - Percentage complete
  - Bytes transferred
- [ ] Optional: Chunked protocol with CRC
  - Defer if too complex
  - Current BOF/EOF is acceptable for v0.3

**Priority:** HIGH - Needed to retrieve flight data
**Estimated Effort:** 2-3 days

---

## 🔧 MEDIUM PRIORITY (Nice to Have for v0.3)

### 4. Calibration System

**Goal:** Interactive calibration workflows via serial commands

**Calibration Commands:**
- [ ] `CAL/GYRO <duration_sec>` - Gyro bias calibration
  - Keep still for N seconds
  - Calculate average readings
  - Store bias in config
- [ ] `CAL/ACCEL` - Accelerometer 6-position calibration
  - Interactive prompts for each orientation
  - Calculate offset and scale factors
  - Store in config
- [ ] `CAL/MAG` - Magnetometer calibration
  - "Mag dance" - rotate in all axes
  - Collect 100+ samples
  - Fit ellipsoid (hard/soft iron)
  - Store in config
- [ ] `CAL/BARO` - Set barometer ground level
  - Use current reading as reference altitude
  - Store offset in config
- [ ] `CAL/SAVE` - Save calibration to config file
- [ ] `CAL/LOAD` - Load calibration from config file
- [ ] `CAL/RESET` - Clear all calibration, use defaults

**CalibrationManager Class:**
- [ ] Create CalibrationManager
  - Handles calibration state machine
  - Collects samples
  - Computes calibration parameters
  - Integrates with SerialMessageRouter
- [ ] Guided workflow
  - Step-by-step instructions via serial
  - Progress feedback
  - Success/failure indication

**Priority:** MEDIUM - Very useful but not blocking
**Estimated Effort:** 3-4 days

---

### 5. BlinkBuzz PWM Support

**Goal:** Add PWM dimming and volume control for LEDs and buzzers

**Tasks:**
- [ ] Add PWM-capable pin handling (platform-specific)
- [ ] Extend BlinkBuzz patterns to accept duty cycle or intensity
- [ ] Document supported platforms and limitations

**Priority:** MEDIUM - Useful for status clarity, not blocking
**Estimated Effort:** 1-2 days

---

### 6. Matrix/Vector Swap (Gemini)

**Goal:** Evaluate and migrate to the Gemini Matrix/Vector implementations

**Tasks:**
- [ ] Compare API parity and performance
- [ ] Plan migration steps and breakages
- [ ] Implement swap if benefits outweigh the change cost

**Priority:** MEDIUM - Nice-to-have, not blocking
**Estimated Effort:** 2-4 days

---

### 7. Native/Unix Build Parity

**Goal:** Keep `unix` and `native` builds in lockstep with the same compiled sources.

**Tasks:**
- [ ] Remove special `build_src_filter` include/exclude differences between `native` and `unix`
- [ ] Ensure both environments build the same examples/src set by default
- [ ] Allow per-env mocks only (no platform-only source exclusions)

**Priority:** MEDIUM - Reduces maintenance and CI surprises
**Estimated Effort:** 1-2 days

---

## 📊 LOW PRIORITY (Defer if Time-Constrained)

### 8. STM32 eMMC Support

**Goal:** Full eMMC support on STM32 platform

**Tasks:**
- [ ] Adapt STM32Duino SD library for eMMC
  - Copy and modify HAL wrapper
  - Configure SDMMC peripheral for eMMC mode
- [ ] Test on STM32 hardware with eMMC chip
  - Verify read/write performance
  - Test with FileLogSink
- [ ] Document eMMC setup
  - Hardware requirements
  - Platform configuration
  - Pin assignments

**Priority:** LOW - Platform-specific, not blocking
**Estimated Effort:** 2-3 days

---

### 9. Teensy CrashReport Integration

**Goal:** Automatic crash capture and reporting

**Implementation:**
- [ ] Hook Teensy CrashReport library
  - Check for crash on boot
  - Log crash info via EventLogger
- [ ] Persistent crash storage
  - Write to SD/eMMC if available
  - Circular buffer if no storage
- [ ] `CMD/CRASH` command
  - Display last crash report
  - Show registers, stack trace, etc.
- [ ] `CMD/CRASH CLEAR` command
  - Clear crash report from memory

**Priority:** LOW - Nice-to-have for debugging
**Estimated Effort:** 1-2 days

---

## 🗑️ REJECTED / DEFERRED

### Out of Scope for v0.3

**Better Error Messages** (Rejected)
- Would require mapping error codes from every vendor library
- Raw error codes work fine and are orders of magnitude less work
- Not worth the effort

**Status Enums** (Rejected)
- Current `int` return codes are sufficient
- Refactor cost outweighs benefit for v0.3

**AstraRocket Improvements** (Deferred - separate repo)
- Will be developed in its own repository
- Not part of core Astra library

**Pyro Control** (Deferred to post-v1.0)
- Too much safety overhead
- Core architecture must be rock-solid first
- Requires extensive testing and validation

**Event System** (Rejected)
- C++ callback limitations make this unwieldy
- Current logging is sufficient
- Not worth the complexity

**Advanced Flight Modes** (Deferred to v0.4+)
- Launch detection
- Recovery logic
- Flight phase state machine
- Apogee prediction

**Radio Telemetry Integration** (Deferred to v0.4+)
- RadioMessage library exists but commented out
- Needs design work
- Not critical for v0.3

**Display Integration** (Deferred to v0.4+)
- SSD1306 OLED library included
- Not integrated with Astra
- Low priority

---

## 📥 NEEDS TRIAGE

**Before finalizing v0.3 scope, need decisions on:**

- [ ] **dt vs currentTime in State/Sensors?**
  - Current: Methods take `currentTime` (absolute time)
  - Alternative: Methods take `dt` (time delta)
  - **Question:** Which is cleaner for event-driven architecture?
  - **Decision deferred:** Evaluate impact on APIs and sensor update paths.

- [ ] **Chunked file transfer protocol?**
  - Current: Simple BOF/EOF streaming
  - Alternative: Packet-based with sequence numbers and CRC
  - **Question:** Is reliability needed, or is simple streaming enough?

---

## 🎯 RECOMMENDED SCOPE FOR V0.3

**Must-Have (Critical Path):**
1. Config System (Phase 1: Persistence) - 3-5 days
2. Command System (Core commands) - 2-3 days
3. File System Commands - 2-3 days

**Should-Have (High Value):**
4. Config System (Phase 2: Runtime modification) - 2 days
5. Calibration System - 3-4 days

**Nice-to-Have (If Time Permits):**
6. Config System (Phase 3: Validation) - 1-2 days

**Defer to v0.4:**
- STM32 eMMC
- Teensy CrashReport

**Prerequisite from v0.2:**
- See TODO_V0.2.md for documentation, examples, and testing blockers.

**Total Estimated Effort:** 13-18 days (depending on scope)

---

## 🚀 PHASED ROLLOUT STRATEGY

**Phase 1: Foundation (Week 1-2)**
- Config persistence (Phase 1)
- Core commands (STATUS, PING, REBOOT)
- File commands (LIST, GET, DELETE)
- **Deliverable:** Usable system for field work

**Phase 2: Polish (Week 3)**
- Config runtime modification
- Calibration system
- Config validation (optional)
- **Deliverable:** Field-tunable system

**Phase 3 prerequisites (from v0.2):**
- See TODO_V0.2.md for documentation, examples, and testing blockers.
- **Deliverable:** Documented, stable v0.3 release

---

## 🎯 VERSION 0.3 SUCCESS CRITERIA

**Release Blockers (Must be complete):**
- [ ] Config persistence working (load/save)
- [ ] Core commands implemented (STATUS, PING, REBOOT)
- [ ] File commands working (LIST, GET, DELETE)
- [ ] No critical bugs
- [ ] Documentation, examples, and tests complete (see TODO_V0.2.md)

**Nice-to-Have (Not blocking):**
- Config runtime modification
- Calibration system
- Config validation

**When these are met, v0.3 can be released.**

---

## 📝 NOTES FOR V0.3 DEVELOPMENT

**Key Architectural Decisions:**
- Config system extends AstraConfig (not a separate manager)
- Use ArduinoJson for parsing (don't write custom parser)
- Keep command parsing simple (no state machines)
- File access uses existing IStorage abstraction
- Commands integrate with SerialMessageRouter (no separate system)

**Risks:**
- Config system touches everything (test thoroughly)
- File commands could block main loop (keep transfers small or chunk)
- ArduinoJson adds memory overhead (monitor heap usage)

**Testing Strategy:**
- Unit test config load/save on native platform
- HITL test commands on real hardware
- Test config persistence across reboots
- Verify file commands don't corrupt storage

**Documentation Priority:**
- README first (most visible)
- API reference second (developers need it)
- Tutorials last (can defer to v0.4)
