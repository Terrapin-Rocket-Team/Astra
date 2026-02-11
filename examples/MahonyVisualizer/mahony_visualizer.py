import argparse
import time
from collections import deque

import numpy as np
import serial
from serial.tools import list_ports

import matplotlib.pyplot as plt


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found.")

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} ({port.description})")

    while True:
        selection = input("Select port: ").strip()
        if selection.isdigit() and 0 <= int(selection) < len(ports):
            return ports[int(selection)].device
        print("Invalid selection.")


def quat_to_rot(w, x, y, z):
    n = w * w + x * x + y * y + z * z
    if n == 0.0:
        return np.eye(3)
    s = 2.0 / n

    wx = s * w * x
    wy = s * w * y
    wz = s * w * z
    xx = s * x * x
    xy = s * x * y
    xz = s * x * z
    yy = s * y * y
    yz = s * y * z
    zz = s * z * z

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def main():
    parser = argparse.ArgumentParser(description="Mahony AHRS 3D visualizer.")
    parser.add_argument("--port", help="Serial port (ex: COM5).")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate.")
    parser.add_argument("--plot-hz", type=float, default=10.0, help="Plot update rate (Hz).")
    parser.add_argument("--max-points", type=int, default=300, help="Max points in each plot.")
    args = parser.parse_args()

    port = args.port or pick_port()
    ser = serial.Serial(port, args.baud, timeout=0)
    time.sleep(2.0)
    ser.reset_input_buffer()

    plt.ion()
    fig = plt.figure(figsize=(12, 8))

    ax_orient = fig.add_subplot(2, 2, 1, projection="3d")
    ax_orient.set_xlim(-1, 1)
    ax_orient.set_ylim(-1, 1)
    ax_orient.set_zlim(-1, 1)
    ax_orient.set_xlabel("X")
    ax_orient.set_ylabel("Y")
    ax_orient.set_zlabel("Z")
    ax_orient.set_title("Mahony Orientation (Body Axes in Inertial Frame)")

    x_line, = ax_orient.plot([0, 1], [0, 0], [0, 0], color="r", linewidth=2, label="X")
    y_line, = ax_orient.plot([0, 0], [0, 1], [0, 0], color="g", linewidth=2, label="Y")
    z_line, = ax_orient.plot([0, 0], [0, 0], [0, 1], color="b", linewidth=2, label="Z")
    ax_orient.legend(loc="upper left")

    ax_acc = fig.add_subplot(2, 2, 2)
    ax_acc.set_title("Accel (m/s^2)")
    ax_acc.set_xlabel("Time (s)")
    ax_acc.set_ylabel("m/s^2")
    ax_acc.grid(True, alpha=0.3)

    ax_gyro = fig.add_subplot(2, 2, 3)
    ax_gyro.set_title("Gyro (rad/s)")
    ax_gyro.set_xlabel("Time (s)")
    ax_gyro.set_ylabel("rad/s")
    ax_gyro.grid(True, alpha=0.3)

    ax_mag = fig.add_subplot(2, 2, 4)
    ax_mag.set_title("Mag (uT)")
    ax_mag.set_xlabel("Time (s)")
    ax_mag.set_ylabel("uT")
    ax_mag.grid(True, alpha=0.3)

    status_text = fig.text(0.02, 0.01, "", fontsize=9, family="monospace")

    window = max(50, args.max_points)
    t0 = None
    times = deque(maxlen=window)
    acc_x = deque(maxlen=window)
    acc_y = deque(maxlen=window)
    acc_z = deque(maxlen=window)
    gyro_x = deque(maxlen=window)
    gyro_y = deque(maxlen=window)
    gyro_z = deque(maxlen=window)
    mag_x = deque(maxlen=window)
    mag_y = deque(maxlen=window)
    mag_z = deque(maxlen=window)

    acc_line_x, = ax_acc.plot([], [], color="r", label="Ax")
    acc_line_y, = ax_acc.plot([], [], color="g", label="Ay")
    acc_line_z, = ax_acc.plot([], [], color="b", label="Az")
    ax_acc.legend(loc="upper right")

    gyro_line_x, = ax_gyro.plot([], [], color="r", label="Gx")
    gyro_line_y, = ax_gyro.plot([], [], color="g", label="Gy")
    gyro_line_z, = ax_gyro.plot([], [], color="b", label="Gz")
    ax_gyro.legend(loc="upper right")

    mag_line_x, = ax_mag.plot([], [], color="r", label="Mx")
    mag_line_y, = ax_mag.plot([], [], color="g", label="My")
    mag_line_z, = ax_mag.plot([], [], color="b", label="Mz")
    ax_mag.legend(loc="upper right")

    plot_interval = 1.0 / max(0.5, args.plot_hz)
    last_plot = time.time()
    latest_quat = None
    serial_buffer = bytearray()

    while True:
        if not plt.fignum_exists(fig.number):
            break
        had_data = False
        waiting = ser.in_waiting
        if waiting:
            serial_buffer.extend(ser.read(waiting))
            had_data = True

        while b"\n" in serial_buffer:
            raw_line, _, serial_buffer = serial_buffer.partition(b"\n")
            line = raw_line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("CAL,"):
                print(line)
                status_text.set_text(line)
                continue

            if line.startswith("RAW,"):
                parts = line.split(",")
                if len(parts) != 10:
                    status_text.set_text(line)
                    continue
                try:
                    values = list(map(float, parts[1:]))
                except ValueError:
                    status_text.set_text(line)
                    continue

                if t0 is None:
                    t0 = time.time()
                t = time.time() - t0
                times.append(t)
                acc_x.append(values[0])
                acc_y.append(values[1])
                acc_z.append(values[2])
                gyro_x.append(values[3])
                gyro_y.append(values[4])
                gyro_z.append(values[5])
                mag_x.append(values[6])
                mag_y.append(values[7])
                mag_z.append(values[8])

                continue

            if not line.startswith("Q,"):
                status_text.set_text(line)
                print(line)
                continue

            parts = line.split(",")
            if len(parts) != 5:
                continue

            try:
                w, x, y, z = map(float, parts[1:])
            except ValueError:
                continue

            latest_quat = (w, x, y, z)

        now = time.time()
        if now - last_plot >= plot_interval:
            last_plot = now

            if latest_quat is not None:
                r = quat_to_rot(*latest_quat)
                x_axis = r @ np.array([1.0, 0.0, 0.0])
                y_axis = r @ np.array([0.0, 1.0, 0.0])
                z_axis = r @ np.array([0.0, 0.0, 1.0])

                x_line.set_data([0, x_axis[0]], [0, x_axis[1]])
                x_line.set_3d_properties([0, x_axis[2]])

                y_line.set_data([0, y_axis[0]], [0, y_axis[1]])
                y_line.set_3d_properties([0, y_axis[2]])

                z_line.set_data([0, z_axis[0]], [0, z_axis[1]])
                z_line.set_3d_properties([0, z_axis[2]])

            acc_line_x.set_data(times, acc_x)
            acc_line_y.set_data(times, acc_y)
            acc_line_z.set_data(times, acc_z)
            gyro_line_x.set_data(times, gyro_x)
            gyro_line_y.set_data(times, gyro_y)
            gyro_line_z.set_data(times, gyro_z)
            mag_line_x.set_data(times, mag_x)
            mag_line_y.set_data(times, mag_y)
            mag_line_z.set_data(times, mag_z)

            for axis in (ax_acc, ax_gyro, ax_mag):
                axis.relim()
                axis.autoscale_view()

            plt.pause(0.001)

        if not had_data:
            time.sleep(0.001)

    ser.close()


if __name__ == "__main__":
    main()
