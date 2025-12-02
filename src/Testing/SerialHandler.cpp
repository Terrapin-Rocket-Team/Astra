//Divyansh Srivastava 11/19/2025
//Copied over from old MMFS



#include "SerialHandler.h"
//#include "Events/Event.h"  need something to replace event's and stuff 

#include "SerialHandler.h"

void SerialHandler::handle() {
    parseSerialStream();
    
    if (messageReady) {
        processCompleteMessage(); //might need to change, this is the step for when it runs the avionics algorithms 
        messageReady = false;
    }
}

void SerialHandler::parseSerialStream() {
    while (Serial.available() > 0) {
        uint8_t inByte = Serial.read();
        
        // Start of message
        if (inByte == START_MARKER) {
            bufferPos = 0;
        }
        // End of message - ready to decode
        else if (inByte == END_MARKER) {
            uint16_t decoded = decode(buffer, bufferPos);
            
            if (decoded > 0) {
                messageReady = true;
                // TODO: Add logging here - log successful decode
                // Logger.log("Decoded %d bytes of sensor data", decoded);
            } else {
                // TODO: Add logging here - log decode error
                // Logger.error("Failed to decode message");
            }
            
            bufferPos = 0;
        }
        // Collect message bytes
        else {
            if (bufferPos < sizeof(buffer) - 1) {
                buffer[bufferPos++] = inByte;
            } else {
                // TODO: Add logging here - buffer overflow
                // Logger.error("Serial buffer overflow");
                bufferPos = 0;
            }
        }
    }
}

uint16_t SerialHandler::decode(uint8_t *data, uint16_t sz) {
    uint16_t pos = 0;
    
    // Check minimum size (13 floats = 52 bytes minimum)
    if (sz < 52) {
        return 0;
    }
    
    // Decode all sensor values matching your BlueRaven columns
    memcpy(&sensorData.batteryVoltage, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.altitude, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.pressure, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.temperature, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.velocity, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.angle, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.accelX, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.accelY, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.accelZ, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.gyroX, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.gyroY, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.gyroZ, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.rollAngle, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    memcpy(&sensorData.tiltAngle, &data[pos], sizeof(float));
    pos += sizeof(float);
    
    // TODO: Add logging here - log decoded values
    // Logger.log("Alt: %.2f, Temp: %.2f, Vel: %.2f", 
    //            sensorData.altitude, sensorData.temperature, sensorData.velocity);
    
    return pos;  // Return number of bytes decoded
}

void SerialHandler::processCompleteMessage() {
    // Process the decoded sensor data
    // TODO: Add logging here - log to SD card or whatever storage you use
    // Logger.logSensorData(sensorData);
    
    // You can access individual values like:
    // float currentAlt = sensorData.altitude;
    // float currentTemp = sensorData.temperature;
}

const char* SerialHandler::getLastLine() const {
    return line;
}

// Command handlers (your existing functions)
void SerialHandler::fetchList() {
    // TODO: Implement - this would interact with your new logging system
}

void SerialHandler::clearFiles() {
    // TODO: Implement - this would interact with your new logging system
}

void SerialHandler::removeFile(char *args) {
    // TODO: Implement
}

void SerialHandler::latestFiles() {
    // TODO: Implement
}

void SerialHandler::copyFile(char *args) {
    // TODO: Implement
}

// Singleton accessor
SerialHandler& getSerialHandler() {
    static SerialHandler instance;
    return instance;
}