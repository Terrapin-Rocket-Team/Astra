# Astra: guide for AI agents

Read this file first when working in this repository. It gives a new agent or
contributor the project boundaries, architecture, runtime flow, and checks needed
to navigate the code. Its scope is the entire Astra repository. Read the relevant
implementation and tests before changing behavior; this guide is a map, not a
replacement for the source.

## Document ownership and required change history

- Created: **2026-10-06** (America/New_York).
- Created by: Divyansh Srivastava
- Last updated: **2026-10-06**.
- Last updated by: Divyansh Srivastava, assisted by OpenAI Codex (AI agent).
- Initial review: library version **0.2.1**, repository commit **9664ffe**.

**Every change to this file must update `Last updated` and `Last updated by`, and
append a row to the change history below with the actual date, editor, and a
specific description.** Use `YYYY-MM-DD` dates and identify the human or AI agent
that made the edit. Preserve earlier entries and the creation metadata. Record
separate edits even if they happen on the same date. When changes to architecture,
public APIs, integration, or tooling make this guide inaccurate, update the
affected sections and record that update in the same change.

| Date | Changed by | Change |
| --- | --- | --- |
| 2026-10-06 | Divyansh Srivastava | Created the repository guide after reviewing code, examples, build configuration, tests, and documentation. |
| 2026-10-06 | Divyansh Srivastava, assisted by OpenAI Codex (AI agent) | Reviewed the guide against implementation; clarified sensor health, telemetry timing/flags, and hardware versus simulation data flow. |

## What Astra is and how it reaches the flight computer

**Astra is Terrapin Rocket Team's reusable flight software library.** It is a
C++17 library built with PlatformIO and Arduino interfaces, packaged as
`TRT-Astra` in [library.json](library.json). It provides sensor drivers and
abstractions, attitude and position estimation, telemetry and event logging,
serial routing, and simulation interfaces.

**The documented deployment model is that SRAD-Avionics consumes and instantiates
Astra as part of its flight-computer application, directly or through the
Astra-Rocket mission layer.** The application supplies sensors, board settings,
mission behavior, and an `AstraConfig`; it constructs an `astra::Astra` instance
and drives its lifecycle. PlatformIO compiles and links the application and
libraries into firmware, and that firmware is uploaded/flashed to the flight
computer. Astra runs on the computer as part of the resulting firmware.

The stack's responsibilities are documented in [README.md](README.md) and the
[documentation home](docs/docs/index.md):

| Project | Responsibility |
| --- | --- |
| **Astra** (this repository) | Reusable sensor, estimator, logging, communications, and HITL/SITL foundation. |
| **Astra-Rocket** | Rocket flight stages, recovery/deployment logic, rocket defaults, and ARC command handling. |
| **SRAD-Avionics** | Board-specific flight-computer firmware and the complete installation, build, upload, and integration workflow. |
| **Astra-Support** | Shared setup diagnostics, maintained test CLI, simulator/process launcher, and native Arduino/sensor mocks. |
| **Airbrake** | Astra-Rocket-based airbrake application and closed-loop simulation. |

For the downstream entry point, actual board wiring, upload configuration, and
full-stack procedure, consult
[SRAD-Avionics's software-stack guide](https://github.com/Terrapin-Rocket-Team/SRAD-Avionics/blob/main/docs/software-stack.md).
The current SRAD application is outside this checkout; its precise instantiation
path is not established by Astra's source alone. `examples/Avionics` here is an
example consumer, not the current SRAD deployment project. Keep general library
APIs here and mission-specific changes in the appropriate downstream project.

## Repository map

| Location | Purpose and useful entry points |
| --- | --- |
| [src/Utils](src/Utils) | `Astra.h/.cpp` orchestrate the system; `AstraConfig.h/.cpp` wire dependencies and runtime modes. `CircBuffer.h` and `Hash.h` provide small utilities. |
| [src/Sensors](src/Sensors) | `Sensor.h`, typed sensor bases, `RotatableSensor.h`, `MountingTransform.h`, composite IMUs, `DualRangeAccel`, voltage sensing, and the header-only `SensorManager/SensorManager.h`. |
| [src/Sensors/HW](src/Sensors/HW) | Hardware wrappers for BMI088/BNO055 IMUs; ADXL375/H3LIS331DL accelerometers; BMP390/DPS368/LPS22DF/MS5611 barometers; LIS3MDL/MLX90393/MMC5603NJ magnetometers; MAX_M10S/SAM_M10Q GPS. |
| [src/Sensors/HITL](src/Sensors/HITL) | Simulated typed sensors and the shared `HITLSensorBuffer`; the same interfaces feed the estimator in simulation. |
| [src/State](src/State) | Math-based `State` estimator and ready-to-use `DefaultState`. |
| [src/Filters](src/Filters) | `Mahony.h` attitude estimator; `LinearKalmanFilter` abstraction; `DefaultKalmanFilter` position/velocity model. |
| [src/Math](src/Math) | Fixed-size `Vector<N>`, `Quaternion`, and dynamically allocated `Matrix` with copy/move support and matrix operations. Preserve embedded third-party license notices. |
| [src/RecordData](src/RecordData) | `DataReporter`/`SimpleDataReporter`, `DataLogger`, `EventLogger`, `ILogSink` implementations, storage interfaces/backends, and active `Retrieval/FileReader.h`. |
| [src/Communication](src/Communication) | `SerialMessageRouter`: buffered text routing across Arduino `Stream` interfaces. |
| [src/Testing](src/Testing) | Active `HITLParser.h`. `SerialHandler.*` contains commented-out legacy code. |
| [src/BlinkBuzz](src/BlinkBuzz) | LED/buzzer patterns, pin registration, and queued asynchronous status feedback. |
| [src/RetrieveData](src/RetrieveData) | Commented-out legacy SD retrieval; use the active storage/retrieval interfaces for new work. |
| [examples](examples) | Consumer programs: Avionics, MinimalSensorSetup, UnixMinimal, DataLoggerOnly, SimpleDataReporter, SerialRouterExample, HITLWithRouter, SITL_Example, and MahonyVisualizer. |
| [test](test) | Eight native Unity suites: communication, data, filters, integration, math, sensors, state, and utils. |
| [docs/docs](docs/docs) | User and maintainer documentation; this explains the intended connections between components. |
| [docs/mkdocs.yml](docs/mkdocs.yml) | MkDocs configuration; `docs/overrides` and stylesheets customize the published site. `docs/SITL_Guide.md` adds simulation background. |
| [platformio.ini](platformio.ini), [library.json](library.json) | Repository build/test environments and exported library package/dependencies, respectively. Check both when changing dependencies or platform support. |
| [boards](boards), [custom_variants](custom_variants), [ldscripts](ldscripts) | Maintained STM32H723 board metadata, pin/clock definitions, and linker configuration. These are source files. |
| [.github/workflows](.github/workflows) | PR test runner and main-branch documentation deployment. |
| Root Python scripts | `run_tests.py` runs the local build/test matrix; `get_version.py` injects `ASTRA_VERSION`; `inject_ws2_32.py` supplies native Windows socket linkage. |

`.pio/` is PlatformIO output/dependency/cache state. `lib/` is the conventional
local-library slot; declared dependencies live in the manifests rather than a
vendored library tree. Treat build caches, coverage output, simulation CSVs,
Python bytecode, and generated compile commands as artifacts.

## Construction, lifetime, and startup

Public classes mostly live in namespace `astra`. Typical consumer code includes
`Utils/Astra.h`, constructs hardware sensor objects and an optional
`DefaultState`, and configures one long-lived system:

```cpp
#include <Arduino.h>
#include <Utils/Astra.h>
#include <State/DefaultState.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/HW/GPS/MAX_M10S.h>

using namespace astra;
BMI088 imu;
DPS368 baro;
MAX_M10S gps;
DefaultState state;
PrintLog serialLog(Serial, true);
ILogSink* sinks[] = { &serialLog };
AstraConfig config;
Astra sys(&config);

void setup() {
    Serial.begin(115200);
    config.with6DoFIMU(&imu).withBaro(&baro).withGPS(&gps)
          .withState(&state).withDataLogs(sinks, 1).withEventLogs(sinks, 1);
    const int errors = sys.init();
    if (errors != 0) LOGW("Astra initialized with %d sensor error(s)", errors);
}
void loop() { sys.update(); }
```

Configure all objects before `init()`. Caller-supplied config, sensors, reporters,
streams, log sinks, and state must remain alive while the system uses them.
`Astra` borrows its `AstraConfig`. The config owns its inline `SensorManager`
and any generated HITL sensors. `Astra` owns its router, default Time reporter,
and a fallback `DefaultState` if no state was supplied. Plain `State` borrows
its filters; `DefaultState` creates and owns its filters. Avoid copying a config
after it owns HITL sensors: its raw owning pointers have no safe deep-copy policy.

`Astra::init()` resolves the runtime mode and, for a native simulation, connects
the transport. It configures event sinks, starts Wire and BlinkBuzz, supplies a
default state if needed, registers telemetry reporters, and configures telemetry
sinks/header. It then populates and initializes the sensor manager, installs
serial listeners, begins the state, establishes or defers the barometric baseline,
and emits initialization feedback. Under `ENV_STM`, it currently initializes I2C
on SDA PB9 / SCL PB8 at 400 kHz.

`init()` returns the counted sensor initialization errors (zero on success);
missing config or a failed native SITL connection can return `-1`. It can mark
the system ready with sensor failures. Missing sensors are warnings, and an
optional magnetometer initialization failure is excluded from the count. Its
return value therefore does not prove the whole mission sensor suite is usable.

## Runtime data flow and timing

```text
hardware drivers ------------------------------------------> SensorManager
simulator -> HITLParser -> HITLSensorBuffer -> HITL sensors -> SensorManager

SensorManager -> State -> Mahony -> ENU acceleration -> Kalman prediction
SensorManager -> State -> GPS/baro measurements -------> Kalman correction

Time + sensors + State + extra DataReporters
  -> DataLogger CSV -> ILogSink -> serial/storage
event messages -> EventLogger -> ILogSink
```

One hardware `Astra::update()` cycle pumps the router and BlinkBuzz, assigns the
state time, calls the state's `update()` hook, and polls due sensors. New accel
**or** gyro data triggers an orientation update followed immediately by Kalman
prediction using the newly transformed acceleration. New healthy GPS data with
a fix corrects horizontal position; new healthy barometer data corrects vertical
position unless optional Mach lockout suppresses it. When logging is due and a
telemetry sink is available, the system refreshes auto-update reporters and emits
CSV. It then refreshes status LEDs.

Time passed to `update(timeSeconds)` and `Sensor::shouldUpdate()` is in **seconds**.
Hardware `update()` uses `millis()/1000.0`. Each sensor schedules its own reads;
there is no global sensor polling rate. `setUpdateRate()` accepts Hz;
`withLoggingRate()` accepts Hz; `withLoggingInterval()` accepts milliseconds and
converts to seconds internally. Default telemetry cadence is 10 Hz. Use positive
rates and increasing simulation timestamps so prediction receives positive `dt`.

`didPredictState()` and `didUpdateState()` identify estimator branches taken.
`didLog()` means logging was due and can be true without an available telemetry
sink. These flags are not overall readiness/health checks. SensorManager's update
flags represent scheduled read attempts rather than guaranteed successful reads.
Current Astra intentionally does not gate accel/gyro propagation on their health;
GPS, baro, and optional mag fusion have additional checks.

## Sensors and estimation contracts

`Sensor` derives from `DataReporter`. `begin()` calls the driver's `init()`;
`update()` calls `read()` and updates health. Driver methods return **0 for
success**. Implement drivers against the appropriate `Accel`, `Gyro`, `Mag`,
`Barometer`, `GPS`, or IMU interface and register stable telemetry fields in the
constructor. Health and initialization are distinct: a later read can fail after
successful initialization. The generic Sensor policy tracks read errors. Accel,
Gyro, Mag, and Barometer provide stuck-reading checks; drivers can override
`updateHealth()` when they need a different policy. HITL sensors accept constant
simulated readings.

`with6DoFIMU()` extracts accel/gyro components; `with9DoFIMU()` also extracts mag.
Both add the parent IMU as a miscellaneous sensor. The manager reads miscellaneous
sensors **before** primaries so the component interfaces see current parent data.
Component telemetry columns are suppressed to avoid duplicating the parent's
columns. `withMiscSensor()` otherwise supports sensors that are polled/logged
without becoming primary state inputs. `DualRangeAccel` combines low/high range
accelerometers; configure each underlying sensor's mounting separately.

Preserve the established units and frames:

| Value | Convention |
| --- | --- |
| Acceleration | m/s^2; sensor/body readings include gravity; state acceleration has gravity removed. |
| Angular velocity | rad/s. |
| Magnetic field | microtesla. |
| Barometric pressure / temperature | hPa / degrees Celsius; computed altitude is metres ASL. |
| GPS position input | `(latitude degrees, longitude degrees, altitude metres)`; GPS velocity getters use NED. |
| State position / velocity | Local ENU: X East, Y North, Z Up; metres / m/s. |
| State orientation | Body-to-inertial quaternion; quaternion constructor order is `(w, x, y, z)`. |

`RotatableSensor` and `MountingTransform` map sensor axes into the configured
frame. Mahony also supports board-to-body mounting. Trace both transforms before
changing orientation to avoid applying the same rotation twice. Logged raw fields
can differ from transformed getter values.

`State` consumes values, not sensor ownership. `DefaultState` combines
`MahonyAHRS` and `DefaultKalmanFilter`. The default Kalman state is
`[px, py, pz, vx, vy, vz]`, with ENU acceleration as control, GPS horizontal
position and barometric altitude as measurements. The current GPS correction
ignores GPS velocity and GPS altitude as measurements and uses a small-distance
latitude/longitude approximation relative to the first fix. Barometric correction
uses displacement from the barometric origin.

State uses accel tilt correction only when acceleration magnitude is within
1 m/s^2 of 9.81; otherwise it propagates attitude using gyro only. Magnetometer
fusion requires Mahony calibration collection/finalization; supplying a mag alone
does not guarantee nine-DoF fusion. Change filter behavior in `src/Filters` and
estimation plumbing in `src/State`; rocket phase detection and deployment belong
in the downstream mission layer. For a custom Kalman model, implement the
`LinearKalmanFilter` matrix methods and keep State's six-value layout assumptions
in mind.

## Telemetry, event logs, storage, and commands

`DataReporter` columns hold pointers to live values plus printf formats/labels;
keep those values valid for the reporter's lifetime. Constructors do not globally
register reporters. During initialization, `AstraConfig::registerResolvedReporters`
resets the registry and registers Time, primaries, State, miscellaneous sensors,
then explicit `withReporter()` entries. The logger has a **32-reporter total
capacity**, including built-ins. Begin standalone custom reporters yourself;
Astra begins its Time reporter, sensors, and state. Sensors/State disable reporter
`autoUpdate` to avoid extra reads during logging.

`DataLogger` emits CSV header/data, optionally prefixed `TELEM/`. `EventLogger`
uses separate sink configuration and `LOGI`, `LOGW`, `LOGE`, `LOGD`, optionally
prefixed `LOG/`; debug output requires `DEBUG`. No successful telemetry sink means
no telemetry output. `ILogSink` extends Arduino `Print`: UART, USB, generic Print,
circular buffer, file, and radio adapters are available.

`FileLogSink` uses `IStorage`/`IFile` through `StorageFactory`, or borrows a
caller-provided backend. Factory-created backends are owned by the file sink.
File naming avoids overwriting existing logs. Teensy and ESP32 have SD backends;
STM32 has an active eMMC backend. **The current STM32 SD backend is deliberately
disabled/stubbed** because STM32SD and STM32EMMC export conflicting globals.
Internal flash is future functionality, and native storage tests use mocks.
`RecordData/Retrieval/FileReader.h` is the active retrieval utility; its interactive
command loop blocks and should not be inserted into the regular flight update.

After `init()`, extend `getMessageRouter()` to add interfaces/listeners. The
router buffers newline-delimited text, strips a matching prefix, and dispatches
its payload with the source stream (and optional callback context). Default Astra
capacity is four interfaces, eight prefixes, and 256-byte line buffers. Prefix
registration order matters. Astra already pumps its router in `update()`.

Built-in command messages are `CMD/HEADER`, `CMD/PING [name]`, and
`CMD/PONG [name]`. PING responds on the requesting stream with
`CMD/PONG <configured system name>`; `withName()` sets that name.

`DataLogger`, `EventLogger`, `HITLSensorBuffer`, and the global BlinkBuzz instance
are shared process state. Multiple Astra objects do not have isolated logging,
simulation, or indicator state. Reset shared state in tests as existing suites do.

## HITL and SITL

HITL runs firmware on hardware with simulated sensors over a `Stream`; SITL runs
the native executable on a PC with a TCP simulator. Both use this text protocol:

```text
HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
```

`HITLParser` injects the shared buffer; typed HITL sensors expose the values to
the normal manager/state path. At least seven fields supply time, accel, and gyro;
ten enable mag, twelve enable baro, and eighteen enable GPS. Units are those in
the sensor table; timestamp is seconds.

`withHITL()` immediately installs config-owned primary HITL sensors. Later sensor
setters override the corresponding defaults, so call order matters when wrapping
sensors or mixing real and simulated inputs. `withHITLInterface()` selects the
embedded input stream. Native **non-test** builds automatically resolve SITL and
install default HITL primaries unless explicit HITL configuration controls them.
Native tests bypass that automatic mode/connection. The default TCP endpoint is
`localhost:5555`, configurable through `withSITLEndpoint()` or native build flags.

The application loop calls `sys.update()`. In simulation, an update without a
timestamp pumps transport/indicators; Astra's `HITL/` listener parses each packet
and calls `update(simTime)` internally. Do not add a second simulation dispatcher.
Explicit simulation updates log every cycle rather than observing the hardware
logging interval. Current simulation baro-baseline setup checks initialization,
not baro packet validity, so a partial first packet can establish a zero baseline.

From the repository root, the maintained simulator is provided by Astra-Support:

```sh
astra-support sim list --project .
astra-support sim run --project . --mode sitl --source physics --build
```

The runner can reuse an existing executable; `--build` rebuilds after changes.
Simulator launchers/native mocks are in Astra-Support, not a standalone simulator
script in this repository.

## Building and checking changes

Start with [installation](docs/docs/user-guide/installation.md) for toolchain
setup. The supported handoff check from the repository root is:

```sh
astra-support doctor --project .
astra-support test --project . --clean --no-progress
```

It builds all four maintained environments and runs native Unity tests. The first
run downloads platforms/libraries and may take several minutes. Hardware upload
and device HITL are separate checks requiring the corresponding flight computer.

| PlatformIO environment | Target / build define | Default example application |
| --- | --- | --- |
| `teensy41` | Teensy 4.1 / `ENV_TEENSY` | `examples/MahonyVisualizer` |
| `stm32h723vehx` | Custom STM32H723 / `ENV_STM` | `examples/Avionics` |
| `esp32` | ESP32-S3 DevKitM-1 / `ENV_ESP` | `examples/Avionics` |
| `native` | Host compiler / `NATIVE=1` | `examples/SITL_Example` |

These build repository examples. Select exactly one application entry point when
changing `build_src_filter`. Native uses Astra-Support mocks, Unity, GCC coverage
flags, and Windows socket linkage; it cannot validate real sensors, storage,
radio behavior, board clocks, or deployment wiring.

For focused implementation checks, use the relevant environment/suite:

```sh
pio run -e native
pio run -e stm32h723vehx
pio test -e native -f test_state_suite
pio test -e native
```

Substitute another existing `test_<area>_suite` for focused checks. The suite
`.cpp` files register functions from their test headers with Unity. Add behavioral
coverage to the matching suite when a change warrants it. The CI entry point is
`python run_tests.py --no-progress`; its `--no-install --no-builds --no-progress`
form runs native suites with existing dependencies. `--clean` belongs to
`astra-support`, not `run_tests.py`. The Python runner uses one worker on Windows
to avoid PlatformIO archive/cache contention.

Hardware drivers depend on external sensor libraries listed in both manifests.
Many versions are unpinned; explicit references include MS5611 `#0.4.1`,
ArduinoEigen `^0.3.2`, and Astra-Support `#main`. Avoid platform-only includes
leaking into native paths; preserve the platform guards around storage and
hardware code. `get_version.py` derives `ASTRA_VERSION` from the manifest's major
and minor version plus a seven-character Git hash.

## Documentation and maintenance workflow

Read [basic use](docs/docs/user-guide/basic-use.md) and the
[Astra system guide](docs/docs/user-guide/utils/mmfssys.md) for configuration;
[sensor](docs/docs/user-guide/ifaces/sensor.md),
[DataReporter](docs/docs/user-guide/ifaces/data-reporter.md),
[State](docs/docs/user-guide/ifaces/state.md), and
[filters](docs/docs/user-guide/ifaces/filters.md) for contracts;
[logging](docs/docs/user-guide/utils/logger.md) and
[serial routing](docs/docs/user-guide/utils/serial-router.md) for outputs;
[HITL](docs/docs/user-guide/hitl.md) and [SITL](docs/docs/user-guide/sitl.md) for
simulation. The [maintainer guide](docs/docs/maintainer-guide/index.md) and
[SensorManager page](docs/docs/maintainer-guide/sensor-manager.md) cover internals.

From the repository root:

```sh
mkdocs serve --config-file docs/mkdocs.yml
mkdocs build --config-file docs/mkdocs.yml
```

The docs workflow installs MkDocs Material, the autolinks plugin, and the
awesome-pages plugin; these are not supplied by root `requirements.txt`. Edit
pages in `docs/docs/`, including `.pages` navigation when appropriate. Main-branch
pushes deploy documentation through CI. Release instructions update `library.json`
and `docs/mkdocs.yml`'s `extra.repo_version`, then tag the release.

When adding a sensor, implement the typed driver, register its telemetry, wire it
through configuration, and document it under `user-guide/ifaces/sensors/`. Keep
sensor reads non-blocking where possible and avoid unnecessary new allocations
in repeated update paths. Preserve the established math types and public include
paths. For an API change, check consumers in examples/tests and update the relevant
user guide and this file's dated history.

Resolve disagreements between older prose and current behavior by reading
headers, implementations, and tests. Known mismatches at initial review:

- `withState()` is optional despite an older docs "Required" heading.
- Reporter registration is explicit during Astra initialization, despite older
  claims of constructor auto-registration.
- Some component READMEs describe old HITL dispatch or individual test folders;
  use the current `docs/docs` workflow and consolidated suites.
- `withSensorBiasCorrectionDataLength`, `withSensorBiasCorrectionDataIgnore`, and
  `withUsingSensorBiasCorrection` are declared in `AstraConfig.h` but have no
  implementation; do not recommend them as working APIs.
- Package version is `0.2.1`; the documentation version label is `0.2`.

Before handing off a change, review the diff, run checks appropriate to what
changed, and report which checks actually ran and any remaining limitations.
