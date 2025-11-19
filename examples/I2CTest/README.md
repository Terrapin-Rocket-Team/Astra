# STM32 I2C Sensor Test Suite

This example tests I2C communication with three sensors on your STM32:
- **DPS368**: Precision barometric pressure sensor
- **SAM-M10Q**: u-blox GPS module
- **BMI088**: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)

## Features

✓ I2C bus scanning to detect all connected devices
✓ Individual sensor initialization tests
✓ Data reading verification
✓ Continuous monitoring mode
✓ Detailed diagnostic output
✓ Troubleshooting guidance

## Hardware Setup

### I2C Connections
All sensors share the same I2C bus (Wire):
- **SCL**: I2C clock line
- **SDA**: I2C data line
- **VCC**: 3.3V power (check your sensor voltage requirements!)
- **GND**: Ground

### Pull-up Resistors
I2C requires pull-up resistors (typically 4.7kΩ) on both SCL and SDA lines. Your board may have them built-in, or you may need to add external resistors.

### Default I2C Addresses
- **DPS368**: 0x77 (or 0x76 if SDO pulled low)
- **SAM-M10Q**: 0x42
- **BMI088 Accel**: 0x18 (or 0x19 if SDO pulled high)
- **BMI088 Gyro**: 0x68 (or 0x69 if SDO pulled high)

If your sensors use different addresses, edit the `#define` statements at the top of [main.cpp](main.cpp).

## Building and Uploading

### Using PlatformIO CLI

```bash
# Build the test
pio run -e stm32_i2c_test

# Upload to STM32
pio run -e stm32_i2c_test -t upload

# Open serial monitor
pio device monitor -b 115200
```

### Using PlatformIO IDE (VSCode)

1. Open the PlatformIO sidebar
2. Select the `stm32_i2c_test` environment
3. Click "Build" (checkmark icon)
4. Click "Upload" (arrow icon)
5. Open Serial Monitor (plug icon) at 115200 baud

## Expected Output

### Successful Test
```
========================================
  STM32 I2C Sensor Test Suite
  DPS368 | SAM-M10Q | BMI088
========================================
Scanning I2C bus...
========================================
I2C device found at address 0x18 (Likely BMI088 Accelerometer)
I2C device found at address 0x42 (Likely SAM-M10Q GPS)
I2C device found at address 0x68 (Likely BMI088 Gyroscope)
I2C device found at address 0x77 (Likely DPS368 Barometer)
Found 4 device(s).
========================================

[DPS368 Test]
Initializing DPS368... SUCCESS
Configuration complete.
Temperature: 24.35 °C
Pressure: 1013.25 hPa
DPS368 is working correctly!

[SAM-M10Q GPS Test]
Initializing GPS... SUCCESS
Configuration complete.
Satellites in view: 8
Fix type: 3D fix
GPS is working correctly!

[BMI088 Accelerometer Test]
Initializing accelerometer... SUCCESS
Accel X: 0.124 m/s²
Accel Y: -0.089 m/s²
Accel Z: 9.814 m/s²
BMI088 Accelerometer is working correctly!

[BMI088 Gyroscope Test]
Initializing gyroscope... SUCCESS
Gyro X: 0.001 rad/s
Gyro Y: -0.002 rad/s
Gyro Z: 0.000 rad/s
BMI088 Gyroscope is working correctly!

========================================

=== Test Summary ===
DPS368 Barometer:      ✓ PASS
SAM-M10Q GPS:          ✓ PASS
BMI088 Accelerometer:  ✓ PASS
BMI088 Gyroscope:      ✓ PASS

Total: 4 / 4 sensors working
========================================

Entering continuous monitoring mode...
Sensor data will update every second.

--- Sensor Readings ---
DPS368 | T: 24.38°C  P: 1013.27 hPa
GPS    | Sats: 9  Fix: 3  Lat: 38.987654  Lon: -76.943210  Alt: 45.2m
Accel  | X: 0.11  Y: -0.08  Z: 9.81 m/s²
Gyro   | X: 0.00  Y: 0.00  Z: 0.00 rad/s
```

## Troubleshooting

### No I2C Devices Found
1. **Check wiring**: Verify SCL and SDA connections
2. **Pull-up resistors**: Ensure 4.7kΩ resistors are present on SCL and SDA
3. **Power supply**: Verify sensors are receiving proper voltage (usually 3.3V)
4. **Continuity**: Test for broken wires or poor connections

### Wrong I2C Addresses Detected
Some sensors have configurable addresses based on SDO/SA0 pins:
- Check your sensor breakout board schematic
- Modify the `#define` addresses in [main.cpp](main.cpp) lines 14-17

### Sensor Found but Initialization Fails
1. **Library compatibility**: Ensure sensor libraries support your specific sensor version
2. **I2C speed**: Try reducing I2C clock to 100kHz (change `Wire.setClock(400000)` to `Wire.setClock(100000)`)
3. **Power cycling**: Some sensors need proper power-up sequencing
4. **Reset**: Check if sensors have a reset pin that needs to be pulled high

### GPS Shows "No Fix"
- GPS needs a clear view of the sky
- Initial fix can take several minutes
- Indoor testing may not work
- Check antenna connection (if using external antenna)
- Satellites in view > 0 means GPS I2C is working

### BMI088 Errors
The BMI088 has two separate I2C devices (accel and gyro):
- Both must be powered and connected
- Check both I2C addresses are correct
- Some breakout boards have separate CS/SDO pins for each

## Modifying the Test

### Changing I2C Bus
If you're using Wire1 or Wire2 instead of Wire:
```cpp
// In setup(), change:
Wire.begin();  // to:
Wire1.begin();

// Update sensor constructors to use Wire1
```

### Adjusting Update Rate
Change the delay in `printAllSensorData()`:
```cpp
if (millis() - lastPrint < 1000) return; // Change 1000 to desired ms
```

### Testing Specific Sensors Only
Comment out sensor tests in `setup()`:
```cpp
// dpsFound = testDPS368();  // Skip DPS368
gpsFound = testGPS();
accelFound = testAccel();
gyroFound = testGyro();
```

## Additional Notes

- **GPS Fix Time**: First fix can take 30+ seconds outdoors, longer indoors
- **I2C Speed**: Default is 400kHz (Fast Mode). Some sensors may require 100kHz (Standard Mode)
- **BMI088 Orientation**: Z-axis should read ~9.8 m/s² when flat
- **DPS368 Altitude**: Can be calculated from pressure using barometric formula

## Reference

- [DPS368 Datasheet](https://www.infineon.com/dgdl/Infineon-DPS368-DS-v01_00-EN.pdf)
- [SAM-M10Q Datasheet](https://www.u-blox.com/en/docs/UBX-21044601)
- [BMI088 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
