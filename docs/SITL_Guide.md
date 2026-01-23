# SITL (Software-In-The-Loop) Guide

## Overview

SITL mode allows you to run the Astra flight software natively on your development machine while connecting to an external physics simulator. This enables:

- **Rapid testing** without hardware
- **Physics simulation** for trajectory validation
- **Integration with simulators** (Python, MATLAB, JSBSim, etc.)
- **GUI development** for monitoring and control
- **Automated testing** of flight scenarios

## Architecture

```
┌─────────────────────────────────────────┐
│  Astra Flight Software (Native Build)  │
│  ┌─────────────────────────────────┐   │
│  │ HITL Sensors → HITLSensorBuffer │   │
│  └──────────▲──────────────────────┘   │
│             │ HITLParser              │
│  ┌──────────┴──────────────────────┐   │
│  │   Serial (SITL-enabled mock)   │   │
│  │   - Reads from TCP socket       │   │
│  │   - Writes to TCP socket        │   │
│  └──────────┬──────────────────────┘   │
└─────────────┼──────────────────────────┘
              │ TCP (localhost:5555)
┌─────────────┼──────────────────────────┐
│  External Simulator (Python/MATLAB)    │
│  - Receives TELEM/ messages            │
│  - Runs physics simulation             │
│  - Sends HITL/ messages                │
└────────────────────────────────────────┘
```

## Protocol

### HITL Messages (Simulator → Flight Software)

The simulator sends sensor data to the flight software using the HITL protocol:

```
HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
```

**Fields:**
- `timestamp`: Simulation time (seconds)
- `ax,ay,az`: Acceleration (m/s²)
- `gx,gy,gz`: Angular velocity (rad/s)
- `mx,my,mz`: Magnetic field (µT)
- `pressure`: Pressure (hPa)
- `temp`: Temperature (°C)
- `lat,lon`: GPS coordinates (decimal degrees)
- `alt`: GPS altitude MSL (m)
- `fix`: GPS fix status (0 or 1)
- `fixqual`: GPS fix quality (number of satellites)
- `heading`: GPS heading (degrees)

### TELEM Messages (Flight Software → Simulator)

The flight software can send telemetry back to the simulator for logging/visualization. Format is flexible, typically:

```
TELEM/<subsystem>,<key1>=<value1>,<key2>=<value2>,...
```

Example:
```
TELEM/baro,pressure=1013.25,alt=100.0,temp=25.0
TELEM/state,phase=BOOST,velocity=50.0
```

## Quick Start

### 1. Build Flight Software for Native

```bash
pio run -e native
```

### 2. Start the Simulator

```bash
# Static test (constant values)
python sitl_simulator.py --sim static

# Parabolic trajectory
python sitl_simulator.py --sim parabolic

# Custom port
python sitl_simulator.py --port 6666
```

### 3. Run Flight Software

The flight software will automatically connect to the simulator when built for native.

```bash
.pio/build/native/program
```

## Using SITL in Your Code

### Basic Setup

```cpp
#include <Arduino.h>
#include <Sensors/HITL/HITL.h>
#include <Communication/SerialMessageRouter.h>

using namespace astra;

// HITL Sensors
HITLBarometer* baro;
HITLAccel* accel;
HITLGPS* gps;

SerialMessageRouter router;

void setup() {
    // Connect to SITL simulator
    if (Serial.connectSITL("localhost", 5555)) {
        LOGI("Connected to SITL!");
    }

    // Create HITL sensors
    baro = new HITLBarometer();
    accel = new HITLAccel();
    gps = new HITLGPS();

    // Setup message router to parse HITL data
    router.withInterface(&Serial)
          .withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
              double simTime;
              HITLParser::parse(msg, simTime);
          });
}

void loop() {
    router.update();  // Process incoming HITL messages

    // Read sensors (populated by HITL parser)
    if (baro->dataReady()) {
        float alt = baro->getAltitude();
        Serial.printf("TELEM/alt=%.2f\n", alt);
    }
}
```

### Conditional SITL Connection

You can make SITL connection optional with environment variables:

```cpp
void setup() {
    const char* sitlHost = getenv("SITL_HOST");
    const char* sitlPort = getenv("SITL_PORT");

    if (sitlHost && sitlPort) {
        Serial.connectSITL(sitlHost, atoi(sitlPort));
    }
}
```

Then run:
```bash
SITL_HOST=localhost SITL_PORT=5555 .pio/build/native/program
```

## Creating Custom Simulators

### Python Example

```python
from sitl_simulator import SITLSimulator
import math

class CustomSimulator(SITLSimulator):
    def generate_sensor_data(self, t):
        # Your custom physics here
        ax = math.sin(t)
        ay = 0.0
        az = -9.81

        # ... compute other sensor values ...

        return f"{t},{ax},{ay},{az},..."

# Run it
sim = CustomSimulator(port=5555)
sim.start()
```

### MATLAB/Simulink Integration

Create a MATLAB script that:
1. Opens TCP server on port 5555
2. Runs your Simulink model
3. Sends HITL messages with simulated sensor data
4. Receives TELEM messages for logging

## NativeTestMocks SITL API

The enhanced Serial mock provides:

```cpp
// Connect to external simulator
bool connectSITL(const char* host, int port);

// Disconnect from simulator
void disconnectSITL();

// Check connection status
bool isSITLConnected() const;
```

### Implementation Details

- **Non-blocking I/O**: Reads from TCP socket are non-blocking
- **Automatic buffering**: Incoming data is buffered in `inputBuffer`
- **Cross-platform**: Works on Windows, Linux, macOS
- **Transparent**: Works with existing `Serial.read()`, `Serial.write()` code

## Testing vs SITL Mode

The NativeTestMocks library supports both modes:

**Test Mode** (default):
- `Serial.simulateInput()` for injecting test data
- `Serial.write()` goes to `fakeBuffer` only
- No external connections

**SITL Mode** (when connected):
- `Serial.connectSITL()` establishes TCP connection
- `Serial.write()` sends to simulator AND `fakeBuffer`
- `Serial.read()` receives from simulator
- Automatically polls for incoming data

## Troubleshooting

### "Failed to connect to SITL simulator"

- Ensure the simulator is running first
- Check the port number matches (default: 5555)
- Verify firewall isn't blocking localhost connections

### Flight software hangs

- Make sure you're calling `router.update()` in your loop
- Check that the simulator is sending HITL messages

### Data not updating

- Verify HITL sensors are created (e.g., `new HITLBarometer()`)
- Check that `HITLParser::parse()` is being called
- Ensure simulator is sending correct HITL format

## Next Steps

- Integrate with JSBSim for 6DOF simulation
- Add GUI for real-time visualization
- Implement Monte Carlo analysis
- Create automated test scenarios
