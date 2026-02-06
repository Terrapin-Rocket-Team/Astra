# SITL Example

This example demonstrates Software-In-The-Loop (SITL) simulation with Astra.

## What This Demonstrates

- Connecting to an external simulator via TCP
- Using HITL sensors to read simulated data
- Sending telemetry back to the simulator
- Running flight software natively without hardware

## Quick Start

### 1. Start the Python Simulator

In one terminal:
```bash
python sitl_simulator.py --sim parabolic
```

You should see:
```
SITL Simulator listening on 0.0.0.0:5555
Waiting for Astra flight software to connect...
```

### 2. Build and Run the Flight Software

In another terminal:
```bash
# Build for native
pio run -e native

# Run the example
.pio/build/native/program.exe
```

### 3. Watch the Data Flow

The flight software will:
1. Connect to the simulator
2. Receive HITL/ messages with sensor data
3. Process the data through HITL sensors + Astra
4. Send TELEM/ CSV messages back (DataLogger)

You'll see output like:
```
TELEM/State - Time (s),State - PX (m),...,HITL_Accelerometer - Acc X (m/s^2),...
TELEM/0.000,0.000,...,0.000,...
...
```

## Available Simulators

The Python harness includes several built-in simulators:

### Static Simulator
```bash
python sitl_simulator.py --sim static
```
Returns constant sensor values. Good for testing connections.

### Parabolic Simulator
```bash
python sitl_simulator.py --sim parabolic
```
Simulates a simple ballistic trajectory:
- Sits on pad until t=2s
- Launches vertically at 100 m/s
- Falls back down
- Lands at t≈22s

## Customizing

### Create Your Own Simulator

Extend the `SITLSimulator` class in [sitl_simulator.py](../../sitl_simulator.py):

```python
class CustomSimulator(SITLSimulator):
    def generate_sensor_data(self, t):
        # Your physics here
        ax, ay, az = compute_acceleration(t)
        # ... other sensors ...

        return f"{t},{ax},{ay},{az},..."
```

### Modify the Flight Software

Edit [SITL_Example.cpp](SITL_Example.cpp) to:
- Add more sensors
- Implement your flight computer logic
- Change telemetry format
- Add state machines

## Integration with GUI

The TCP interface makes it easy to build a monitoring GUI:

```python
import socket

# Connect to flight software
sock = socket.socket()
sock.connect(('localhost', 5555))

# Send sensor data
sock.send(b"HITL/1.0,0,0,-9.81,...\n")

# Receive telemetry
data = sock.recv(1024)
print(f"Received: {data}")
```

## Troubleshooting

**"Failed to connect to SITL simulator"**
- Make sure the Python simulator is running first
- Check that port 5555 isn't blocked by firewall
- Try connecting to "127.0.0.1" instead of "localhost"

**No data appearing**
- Verify the simulator is sending HITL/ messages
- Check that `router.update()` is being called in loop()
- Add debug prints in the HITL/ listener callback

**Flight software crashes**
- Check that all HITL sensors are properly initialized
- Verify the HITL message format matches the parser

## Next Steps

- Integrate with JSBSim for 6DOF simulation
- Build a PyQt GUI for real-time visualization
- Run Monte Carlo simulations
- Export data for analysis in MATLAB/Python
