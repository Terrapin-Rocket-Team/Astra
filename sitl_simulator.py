#!/usr/bin/env python3
"""
SITL Simulator Harness for Astra Flight Software

This is a basic example of an external simulator that communicates with
the Astra flight software running in SITL mode via TCP sockets.

Protocol:
- Receives TELEM/ messages from flight software (for logging/visualization)
- Sends HITL/ messages to flight software (simulated sensor data)

HITL Protocol Format:
HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading

Usage:
    python sitl_simulator.py [--port PORT] [--sim SIMULATOR]

Where SIMULATOR can be:
    - static: Static test data (default)
    - parabolic: Simple parabolic trajectory simulation
    - file: Read from trajectory file
"""

import socket
import threading
import time
import argparse
import sys
import math


class SITLSimulator:
    """Base class for SITL simulators"""

    def __init__(self, host='0.0.0.0', port=5555):
        self.host = host
        self.port = port
        self.running = False
        self.client_socket = None
        self.server_socket = None
        self.sim_time = 0.0
        self.dt = 0.01  # 100 Hz simulation rate

    def start(self):
        """Start the simulator server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)

        print(f"SITL Simulator listening on {self.host}:{self.port}")
        print("Waiting for Astra flight software to connect...")

        self.client_socket, addr = self.server_socket.accept()
        print(f"Connected to Astra at {addr}")

        self.running = True

        # Start receiver thread for TELEM messages
        recv_thread = threading.Thread(target=self._receive_telemetry, daemon=True)
        recv_thread.start()

        # Run simulation loop
        self._simulation_loop()

    def stop(self):
        """Stop the simulator"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

    def _receive_telemetry(self):
        """Receive and process TELEM/ messages from flight software"""
        buffer = ""
        while self.running:
            try:
                data = self.client_socket.recv(1024).decode('utf-8', errors='ignore')
                if not data:
                    print("Connection closed by flight software")
                    self.running = False
                    break

                buffer += data
                lines = buffer.split('\n')
                buffer = lines[-1]  # Keep incomplete line

                for line in lines[:-1]:
                    line = line.strip()
                    if line.startswith("TELEM/"):
                        self._process_telemetry(line)

            except Exception as e:
                if self.running:
                    print(f"Error receiving telemetry: {e}")
                break

    def _process_telemetry(self, line):
        """Process incoming TELEM/ message - override in subclasses"""
        print(f"RX: {line}")

    def _simulation_loop(self):
        """Main simulation loop - sends HITL/ messages"""
        try:
            while self.running:
                start_time = time.time()

                # Generate simulated sensor data
                hitl_data = self.generate_sensor_data(self.sim_time)

                # Send HITL message
                message = f"HITL/{hitl_data}\n"
                try:
                    self.client_socket.sendall(message.encode('utf-8'))
                    # print(f"TX: {message.strip()}")  # Uncomment for debugging
                except Exception as e:
                    print(f"Error sending HITL data: {e}")
                    self.running = False
                    break

                # Advance simulation time
                self.sim_time += self.dt

                # Sleep to maintain simulation rate
                elapsed = time.time() - start_time
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)

        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
        finally:
            self.stop()

    def generate_sensor_data(self, t):
        """
        Generate simulated sensor data at time t
        Override this in subclasses to implement different simulations

        Returns: CSV string with format:
        timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
        """
        raise NotImplementedError("Subclasses must implement generate_sensor_data()")


class StaticSimulator(SITLSimulator):
    """Static test simulator - returns constant sensor values"""

    def generate_sensor_data(self, t):
        # Static values for testing
        ax, ay, az = 0.0, 0.0, -9.81  # 1G down
        gx, gy, gz = 0.0, 0.0, 0.0    # No rotation
        mx, my, mz = 20.0, 0.0, -45.0  # Earth's magnetic field
        pressure = 1013.25  # Sea level
        temp = 25.0
        lat, lon = 38.123, -122.456  # Sample GPS coordinates
        alt = 100.0
        fix = 1
        fix_quality = 8
        heading = 0.0

        return f"{t:.3f},{ax},{ay},{az},{gx},{gy},{gz},{mx},{my},{mz},{pressure},{temp},{lat},{lon},{alt},{fix},{fix_quality},{heading}"


class ParabolicSimulator(SITLSimulator):
    """Simple parabolic trajectory simulator"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.launch_time = 2.0  # Launch at t=2s
        self.initial_velocity = 100.0  # m/s
        self.g = 9.81

    def generate_sensor_data(self, t):
        if t < self.launch_time:
            # On pad
            ax, ay, az = 0.0, 0.0, -9.81
            gx, gy, gz = 0.0, 0.0, 0.0
            alt = 0.0
            velocity = 0.0
        else:
            # In flight
            flight_time = t - self.launch_time
            velocity = self.initial_velocity - self.g * flight_time
            alt = self.initial_velocity * flight_time - 0.5 * self.g * flight_time**2

            if alt < 0:
                alt = 0
                velocity = 0
                ax, ay, az = 0.0, 0.0, -9.81
            else:
                # Acceleration in body frame (assuming vertical flight)
                total_accel = -self.g - (velocity - (self.initial_velocity - self.g * flight_time)) / self.dt
                ax, ay, az = 0.0, 0.0, total_accel

            gx, gy, gz = 0.0, 0.0, 0.0  # No rotation for simple trajectory

        # Other sensors
        mx, my, mz = 20.0, 0.0, -45.0
        pressure = 1013.25 * math.exp(-alt / 8500.0)  # Barometric formula
        temp = 25.0 - alt * 0.0065  # Standard atmosphere
        lat, lon = 38.123, -122.456
        fix = 1
        fix_quality = 8
        heading = 0.0

        return f"{t:.3f},{ax},{ay},{az},{gx},{gy},{gz},{mx},{my},{mz},{pressure:.2f},{temp:.2f},{lat},{lon},{alt:.2f},{fix},{fix_quality},{heading}"


def main():
    parser = argparse.ArgumentParser(description='SITL Simulator for Astra Flight Software')
    parser.add_argument('--port', type=int, default=5555, help='TCP port to listen on (default: 5555)')
    parser.add_argument('--sim', choices=['static', 'parabolic'], default='static',
                        help='Simulator type (default: static)')

    args = parser.parse_args()

    # Select simulator
    if args.sim == 'static':
        simulator = StaticSimulator(port=args.port)
    elif args.sim == 'parabolic':
        simulator = ParabolicSimulator(port=args.port)
    else:
        print(f"Unknown simulator type: {args.sim}")
        sys.exit(1)

    print(f"Starting {args.sim} simulator...")
    simulator.start()


if __name__ == '__main__':
    main()
