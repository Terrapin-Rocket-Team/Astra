# Teensy Serial Bridge

Simple serial passthrough that routes Serial1 (hardware UART) to USB Serial for monitoring.

## Use Case

Monitor external devices connected to Teensy's Serial1 pins without needing a separate USB-to-Serial adapter.

## Hardware Connections

**Teensy 4.1:**
- **Pin 0 (RX1)** - Connect to external device's TX
- **Pin 1 (TX1)** - Connect to external device's RX
- **GND** - Connect to external device's GND
- **USB** - Connect to computer for monitoring

## Building and Uploading

```bash
# Build
pio run -e teensy_serial_bridge

# Upload
pio run -e teensy_serial_bridge -t upload

# Monitor USB Serial
pio device monitor -b 115200
```

## Configuration

Edit [main.cpp](main.cpp) to change baud rates:

```cpp
// Serial1 baud rate (adjust to match your external device)
#define SERIAL1_BAUD 115200

// USB Serial baud rate
#define USB_BAUD 115200
```

## Features

- **Bidirectional:** Data flows both ways (USB ↔ Serial1)
- **Type commands** in the USB serial monitor and they'll be sent to the device on Serial1
- **Receive data** from Serial1 and display it on USB serial monitor
- **Fast:** Minimal latency passthrough

## Example Use Cases

1. **Debug STM32:** Connect STM32's UART to Teensy Serial1, monitor via USB
2. **GPS Module:** Connect GPS TX to Pin 0, monitor NMEA sentences via USB
3. **Radio Module:** Connect radio module to Serial1, send/receive commands via USB
4. **Sensor with UART:** Monitor sensor output that uses UART communication

## Troubleshooting

**No data appearing:**
- Verify baud rates match on both sides
- Check TX/RX are not swapped (TX→RX, RX→TX)
- Verify ground connection between devices
- Check that external device is powered and transmitting

**Garbled data:**
- Baud rate mismatch - adjust `SERIAL1_BAUD` to match external device
