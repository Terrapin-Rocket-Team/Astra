/*
 * I2C Sensor Test Example for STM32
 * Tests: DPS368 (Barometer), SAM-M10Q (GPS), BMI088 (Accel/Gyro)
 *
 * This example performs comprehensive I2C diagnostics:
 * 1. I2C bus scan to detect all connected devices
 * 2. Individual sensor initialization tests
 * 3. Data reading from each sensor
 * 4. Continuous monitoring mode
 *
 * UART Output: PB12 (TX), PB13 (RX) at 115200 baud
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <bmi088.h>

// Hardware Serial on PB12 (TX) and PB13 (RX)
HardwareSerial Serial5(PB13, PB12);

// I2C Addresses
#define DPS368_ADDR 0x77      // DPS368 default I2C address (or 0x76 depending on SDO pin)
#define SAM_M10Q_ADDR 0x42    // u-blox GPS default I2C address
#define BMI088_ACCEL_ADDR 0x18 // BMI088 Accelerometer (or 0x19 depending on SDO pin)
#define BMI088_GYRO_ADDR 0x68  // BMI088 Gyroscope (or 0x69 depending on SDO pin)

// Sensor objects
Adafruit_DPS310 dps;
SFE_UBLOX_GNSS gps;
Bmi088Accel accel(Wire, BMI088_ACCEL_ADDR);
Bmi088Gyro gyro(Wire, BMI088_GYRO_ADDR);

// Test results
bool dpsFound = false;
bool gpsFound = false;
bool accelFound = false;
bool gyroFound = false;

void printDivider() {
    Serial5.println(F("========================================"));
}

// Scan I2C bus for all devices
void scanI2CBus() {
    byte error, address;
    int deviceCount = 0;
    int errorCount = 0;

    printDivider();
    Serial5.println(F("Scanning I2C bus..."));
    printDivider();

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial5.print(F("I2C device found at address 0x"));
            if (address < 16) Serial5.print("0");
            Serial5.print(address, HEX);

            // Identify known devices
            if (address == 0x76 || address == 0x77) {
                Serial5.print(F(" (Likely DPS368 Barometer)"));
            } else if (address == 0x42) {
                Serial5.print(F(" (Likely SAM-M10Q GPS)"));
            } else if (address == 0x18 || address == 0x19) {
                Serial5.print(F(" (Likely BMI088 Accelerometer)"));
            } else if (address == 0x68 || address == 0x69) {
                Serial5.print(F(" (Likely BMI088 Gyroscope)"));
            }
            Serial5.println();
            deviceCount++;
        } else if (error == 2) {
            // NACK on transmit of address - normal, no device present
        } else if (error == 3) {
            Serial5.print(F("NACK on transmit of data at 0x"));
            if (address < 16) Serial5.print("0");
            Serial5.println(address, HEX);
            errorCount++;
        } else if (error == 4) {
            Serial5.print(F("Unknown error at address 0x"));
            if (address < 16) Serial5.print("0");
            Serial5.println(address, HEX);
            errorCount++;
        }
    }

    if (deviceCount == 0) {
        Serial5.println(F("No I2C devices found!"));
        Serial5.println();
        Serial5.println(F("TROUBLESHOOTING:"));
        Serial5.println(F("1. Check wiring:"));
        Serial5.println(F("   - PB8 → SCL on all sensors"));
        Serial5.println(F("   - PB9 → SDA on all sensors"));
        Serial5.println(F("   - Connect all sensor GND to STM32 GND"));
        Serial5.println(F("2. Add pull-up resistors:"));
        Serial5.println(F("   - 4.7kΩ from SCL to 3.3V"));
        Serial5.println(F("   - 4.7kΩ from SDA to 3.3V"));
        Serial5.println(F("3. Check sensor power (3.3V)"));
        Serial5.println(F("4. Verify sensor I2C addresses (check SDO pins)"));
    } else {
        Serial5.print(F("Found "));
        Serial5.print(deviceCount);
        Serial5.println(F(" device(s)."));
    }

    if (errorCount > 0) {
        Serial5.print(F("Warning: "));
        Serial5.print(errorCount);
        Serial5.println(F(" I2C errors detected"));
    }
    printDivider();
}

// Test DPS368 Barometer
bool testDPS368() {
    Serial5.println(F("\n[DPS368 Test]"));
    Serial5.print(F("Initializing DPS368... "));

    if (!dps.begin_I2C(DPS368_ADDR)) {
        Serial5.println(F("FAILED"));
        Serial5.println(F("Could not find DPS368 sensor!"));
        return false;
    }

    Serial5.println(F("SUCCESS"));

    // Configure sensor
    dps.configurePressure(DPS310_64HZ, DPS310_32SAMPLES);
    dps.configureTemperature(DPS310_32HZ, DPS310_8SAMPLES);
    dps.setMode(DPS310_CONT_PRESTEMP);

    Serial5.println(F("Configuration complete."));

    // Test reading
    delay(100); // Wait for first reading
    sensors_event_t temp_event, pressure_event;

    if (dps.getEvents(&temp_event, &pressure_event)) {
        Serial5.print(F("Temperature: "));
        Serial5.print(temp_event.temperature);
        Serial5.println(F(" °C"));

        Serial5.print(F("Pressure: "));
        Serial5.print(pressure_event.pressure);
        Serial5.println(F(" hPa"));

        Serial5.println(F("DPS368 is working correctly!"));
        return true;
    } else {
        Serial5.println(F("Failed to read from DPS368"));
        return false;
    }
}

// Test SAM-M10Q GPS
bool testGPS() {
    Serial5.println(F("\n[SAM-M10Q GPS Test]"));
    Serial5.print(F("Initializing GPS... "));

    if (!gps.begin(Wire, SAM_M10Q_ADDR)) {
        Serial5.println(F("FAILED"));
        Serial5.println(F("Could not find SAM-M10Q GPS!"));
        return false;
    }

    Serial5.println(F("SUCCESS"));

    // Configure GPS
    gps.setI2COutput(COM_TYPE_UBX);
    gps.setNavigationFrequency(5);

    Serial5.println(F("Configuration complete."));
    Serial5.println(F("Waiting for GPS fix (this may take a while)..."));

    // Try to get initial data (don't wait for fix)
    delay(1000);
    if (gps.getPVT()) {
        Serial5.print(F("Satellites in view: "));
        Serial5.println(gps.getSIV());

        Serial5.print(F("Fix type: "));
        byte fixType = gps.getFixType();
        if (fixType == 0) Serial5.println(F("No fix"));
        else if (fixType == 1) Serial5.println(F("Dead reckoning"));
        else if (fixType == 2) Serial5.println(F("2D fix"));
        else if (fixType == 3) Serial5.println(F("3D fix"));
        else if (fixType == 4) Serial5.println(F("GNSS + Dead reckoning"));
        else Serial5.println(F("Time only"));

        Serial5.println(F("GPS is working correctly!"));
        return true;
    } else {
        Serial5.println(F("No data from GPS yet (may need time to acquire signal)"));
        Serial5.println(F("GPS I2C communication is OK."));
        return true;
    }
}

// Test BMI088 Accelerometer
bool testAccel() {
    Serial5.println(F("\n[BMI088 Accelerometer Test]"));
    Serial5.print(F("Initializing accelerometer... "));

    int status = accel.begin();
    if (status < 0) {
        Serial5.println(F("FAILED"));
        Serial5.print(F("Accel initialization error: "));
        Serial5.println(status);
        return false;
    }

    Serial5.println(F("SUCCESS"));

    // Test reading
    accel.readSensor();
    Serial5.print(F("Accel X: "));
    Serial5.print(accel.getAccelX_mss(), 3);
    Serial5.println(F(" m/s²"));

    Serial5.print(F("Accel Y: "));
    Serial5.print(accel.getAccelY_mss(), 3);
    Serial5.println(F(" m/s²"));

    Serial5.print(F("Accel Z: "));
    Serial5.print(accel.getAccelZ_mss(), 3);
    Serial5.println(F(" m/s²"));

    Serial5.println(F("BMI088 Accelerometer is working correctly!"));
    return true;
}

// Test BMI088 Gyroscope
bool testGyro() {
    Serial5.println(F("\n[BMI088 Gyroscope Test]"));
    Serial5.print(F("Initializing gyroscope... "));

    int status = gyro.begin();
    if (status < 0) {
        Serial5.println(F("FAILED"));
        Serial5.print(F("Gyro initialization error: "));
        Serial5.println(status);
        return false;
    }

    Serial5.println(F("SUCCESS"));

    // Test reading
    gyro.readSensor();
    Serial5.print(F("Gyro X: "));
    Serial5.print(gyro.getGyroX_rads(), 3);
    Serial5.println(F(" rad/s"));

    Serial5.print(F("Gyro Y: "));
    Serial5.print(gyro.getGyroY_rads(), 3);
    Serial5.println(F(" rad/s"));

    Serial5.print(F("Gyro Z: "));
    Serial5.print(gyro.getGyroZ_rads(), 3);
    Serial5.println(F(" rad/s"));

    Serial5.println(F("BMI088 Gyroscope is working correctly!"));
    return true;
}

// Print all sensor data
void printAllSensorData() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 1000) return; // Print once per second
    lastPrint = millis();

    Serial5.println(F("\n--- Sensor Readings ---"));

    // DPS368
    if (dpsFound) {
        sensors_event_t temp_event, pressure_event;
        delay(10); // Give sensor time to update
        bool success = dps.getEvents(&temp_event, &pressure_event);
        if (success) {
            Serial5.print(F("DPS368 | T: "));
            Serial5.print(temp_event.temperature, 2);
            Serial5.print(F("°C  P: "));
            Serial5.print(pressure_event.pressure, 2);
            Serial5.println(F(" hPa"));
        } else {
            Serial5.println(F("DPS368 | Read FAILED"));
        }
    }

    // GPS
    if (gpsFound) {
        if (gps.getPVT()) {
            Serial5.print(F("GPS    | Sats: "));
            Serial5.print(gps.getSIV());
            Serial5.print(F("  Fix: "));
            Serial5.print(gps.getFixType());
            if (gps.getFixType() >= 2) {
                Serial5.print(F("  Lat: "));
                Serial5.print(gps.getLatitude() / 10000000.0, 6);
                Serial5.print(F("  Lon: "));
                Serial5.print(gps.getLongitude() / 10000000.0, 6);
                Serial5.print(F("  Alt: "));
                Serial5.print(gps.getAltitude() / 1000.0, 1);
                Serial5.print(F("m"));
            }
            Serial5.println();
        }
    }

    // BMI088 Accelerometer
    if (accelFound) {
        accel.readSensor();
        Serial5.print(F("Accel  | X: "));
        Serial5.print(accel.getAccelX_mss(), 2);
        Serial5.print(F("  Y: "));
        Serial5.print(accel.getAccelY_mss(), 2);
        Serial5.print(F("  Z: "));
        Serial5.print(accel.getAccelZ_mss(), 2);
        Serial5.println(F(" m/s²"));
    }

    // BMI088 Gyroscope
    if (gyroFound) {
        gyro.readSensor();
        Serial5.print(F("Gyro   | X: "));
        Serial5.print(gyro.getGyroX_rads(), 2);
        Serial5.print(F("  Y: "));
        Serial5.print(gyro.getGyroY_rads(), 2);
        Serial5.print(F("  Z: "));
        Serial5.print(gyro.getGyroZ_rads(), 2);
        Serial5.println(F(" rad/s"));
    }
}

void setup() {
    // Initialize serial on PB12/PB13
    Serial5.begin(115200);
    delay(1000);

    Serial5.println(F("\n\n"));
    printDivider();
    Serial5.println(F("  STM32 I2C Sensor Test Suite"));
    Serial5.println(F("  DPS368 | SAM-M10Q | BMI088"));
    printDivider();

    // Initialize I2C on PB8 (SCL) and PB9 (SDA)
    Wire.setSCL(PB8);
    Wire.setSDA(PB9);
    Wire.begin();
    Wire.setClock(100000); // 100kHz Standard Mode (more reliable)

    Serial5.println(F("\nI2C initialized:"));
    Serial5.println(F("  SCL: PB8"));
    Serial5.println(F("  SDA: PB9"));
    Serial5.println(F("  Clock: 100kHz"));
    Serial5.println(F("  WARNING: External pull-ups (4.7kΩ) required!"));

    delay(100);

    // Scan I2C bus
    scanI2CBus();

    // Test each sensor
    dpsFound = testDPS368();
    gpsFound = testGPS();
    accelFound = testAccel();
    gyroFound = testGyro();

    // Summary
    printDivider();
    Serial5.println(F("\n=== Test Summary ==="));
    Serial5.print(F("DPS368 Barometer:      "));
    Serial5.println(dpsFound ? F("✓ PASS") : F("✗ FAIL"));

    Serial5.print(F("SAM-M10Q GPS:          "));
    Serial5.println(gpsFound ? F("✓ PASS") : F("✗ FAIL"));

    Serial5.print(F("BMI088 Accelerometer:  "));
    Serial5.println(accelFound ? F("✓ PASS") : F("✗ FAIL"));

    Serial5.print(F("BMI088 Gyroscope:      "));
    Serial5.println(gyroFound ? F("✓ PASS") : F("✗ FAIL"));

    int passCount = dpsFound + gpsFound + accelFound + gyroFound;
    Serial5.print(F("\nTotal: "));
    Serial5.print(passCount);
    Serial5.println(F(" / 4 sensors working"));
    printDivider();

    if (passCount > 0) {
        Serial5.println(F("\nEntering continuous monitoring mode..."));
        Serial5.println(F("Sensor data will update every second."));
    } else {
        Serial5.println(F("\nNo sensors detected! Check:"));
        Serial5.println(F("1. Wiring and connections"));
        Serial5.println(F("2. I2C pull-up resistors (4.7kΩ typical)"));
        Serial5.println(F("3. Power supply to sensors"));
        Serial5.println(F("4. I2C addresses (check SDO pins)"));
    }
}

void loop() {
    printAllSensorData();
    // No extra delay needed - printAllSensorData has its own timing
}
