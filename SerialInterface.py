import serial
from serial.tools import list_ports
import time
import csv
import sys
import select

# returns all serial ports open
def select_serial_port():
    ports = list(list_ports.comports())
    print(">>List of Serial Ports ---------")
    i = 0
    for port in ports:
        print(str(i) + ": " + str(port.device))
        i+=1

    selection = int(input(">>Enter choice: "))
    serial_port = ports[selection]
    
    return serial_port


# gets the raw serial data from the microcontroller
def get_data_from_serial(ser:serial.Serial):
    data = []
    currentLine = ""
    # Check we are not past EOF
    while currentLine != b'|----------EOF----------|\r\n':
        bytes = ser.readline()
        data.append(bytes)
        currentLine = bytes
    # encode data as UTF-8 to make it easier to work with
    data = [element.decode('utf-8').strip() for element in data]
    return data
# removes data between BOF and EOF identifiers
def clean_data(raw_data):
    csv_data = []
    in_block = False
    for line in raw_data:
        # check for file transfer beginning and ending
        if line == '|----------BOF----------|':
            in_block = True
        elif line == '|----------EOF----------|':
            in_block = False
        # check for BOF identifier since we will be in the same loop as when we check for it
        if in_block and line != '|----------BOF----------|':
            csv_data.append(line)
    return csv_data
# Writes data to CSV
def write_data(cleaned_data):
    with open("output.csv", "w", newline="") as f:
        writer = csv.writer(f)
        # Write the header first
        header = cleaned_data[0].split(",")
        writer.writerow(header)
        # Write the rest of the data
        for line in cleaned_data[1:]:
            writer.writerow(line.split(","))


# Receive a file transfer from the microcontroller and write it to output.csv.
def _handle_file_transfer(ser: serial.Serial):
    print(">>Starting file transfer... reading from serial")
    raw = get_data_from_serial(ser)
    cleaned = clean_data(raw)
    if not cleaned:
        print(">>No file data received.")
        return
    write_data(cleaned)
    print(">>File written to output.csv")


def interactive_loop(ser: serial.Serial):
    file_transfer_in_progress = False
    try:
        while True:
            # select on stdin and the serial port file descriptor
            rlist = [sys.stdin, ser]
            readable, _, _ = select.select(rlist, [], [])

            for src in readable:
                if src is sys.stdin:
                    # Read one line from stdin (non-blocking because select said it's ready)
                    line = sys.stdin.readline()
                    if not line:
                        print('\n>>EOF on stdin, exiting')
                        return
                    line = line.strip()
                    if not line:
                        continue
                    if line.lower() in ("exit", "quit"):
                        print('>>Exiting')
                        return

                    # Send the command to the MCU
                    ser.write(line.encode() + b"\n")

                    parts = line.split()
                    if parts and parts[0].lower() == 'cmd/sf':
                        # Pause normal serial processing and handle the transfer
                        file_transfer_in_progress = True
                        # Small delay to give MCU time to start sending (if needed)
                        time.sleep(0.05)
                        _handle_file_transfer(ser)
                        file_transfer_in_progress = False

                else:
                    # src is the serial port and it's ready to be read
                    # If a file transfer is being handled we skip printing here
                    if file_transfer_in_progress:
                        # Let get_data_from_serial handle reading during transfer
                        continue
                    # Non-file serial data: read and print one line
                    try:
                        resp = ser.readline()
                    except Exception as e:
                        print(f">>Error reading serial: {e}")
                        continue
                    if resp:
                        try:
                            print(resp.decode('utf-8', errors='ignore').rstrip())
                        except Exception:
                            # Fallback if bytes can't be decoded
                            print(resp)
    except KeyboardInterrupt:
        print('\nInterrupted, exiting')


if __name__ == '__main__':
    serial_port = select_serial_port()
    ser = serial.Serial(serial_port.device, baudrate=115200)
    interactive_loop(ser)