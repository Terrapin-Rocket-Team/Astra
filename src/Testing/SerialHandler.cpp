//Divyansh Srivastava 11/19/2025
//Copied over from old MMFS
//Created this serial parser class to extract all data coming in from a csv file. I choose to use
//a buffer because HITL will read and process one line at a time. Incoming data includes: battery voltage, 
//altitude, pressure, temperature, etc. 

#if !defined(UNIT_TEST) && (defined(ENV_STM) || defined(ENV_ESP) || defined(ENV_TEENSY))

#include "SerialHandler.h"
//#include "Events/Event.h"  need something to replace event's and stuff

void SerialHandler::handle() {
    parseSerialStream();
    
    if (messageReady) {
        processCompleteMessage(); //might need to change, this is the step for when it runs the avionics algorithms 
        messageReady = false;
    }
}

//this buffer is used to collect all the data between a start marker
//and an ending marker 
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
                //this is where i set pointers to what data is there (ex. &sensorData.altitude, etc. )
            } else {
                // TODO: Add logging here - log decode error
                // Logger.error("Failed to decode message");
            }
            
            bufferPos = 0;
        }
        // Collect message bytes
        else {
            if (bufferPos < sizeof(buffer) - 1) {
                buffer[bufferPos] = inByte; //assign data within buffer
                bufferPos++; //increment buffer position each BYTE 
            } else {
                // TODO: Add logging here - buffer overflow
                // Logger.error("Serial buffer overflow");
                bufferPos = 0;
            }
        }
    }
}


//Here, I have a data array (sort of like a buffer) and I bring all that data from data into 
//the saved struct I have and assign each field to what it respectiverly should be.
uint16_t SerialHandler::decode(uint8_t *data /*source pointer*/, uint16_t sz) {
    uint16_t pos = 0;
    
    // Check minimum size (13 floats = 52 bytes minimum)
    if (sz < 52) {
        return 0;
    }
    
    // Decode all sensor values matching your BlueRaven columns
   memcpy(&sensorData.batteryVoltage, &data[pos], sizeof(sensorData.batteryVoltage));
    pos += sizeof(sensorData.batteryVoltage);
    
    memcpy(&sensorData.altitude, &data[pos], sizeof(sensorData.altitude));
    pos += sizeof(sensorData.altitude);
    
    memcpy(&sensorData.pressure, &data[pos], sizeof(sensorData.pressure));
    pos += sizeof(sensorData.pressure);
    
    memcpy(&sensorData.temperature, &data[pos], sizeof(sensorData.temperature));
    pos += sizeof(sensorData.temperature);
    
    memcpy(&sensorData.velocity, &data[pos], sizeof(sensorData.velocity));
    pos += sizeof(sensorData.velocity);
    
    memcpy(&sensorData.angle, &data[pos], sizeof(sensorData.angle));
    pos += sizeof(sensorData.angle);
    
    memcpy(&sensorData.accelX, &data[pos], sizeof(sensorData.accelX));
    pos += sizeof(sensorData.accelX);
    
    memcpy(&sensorData.accelY, &data[pos], sizeof(sensorData.accelY));
    pos += sizeof(sensorData.accelY);
    
    memcpy(&sensorData.accelZ, &data[pos], sizeof(sensorData.accelZ));
    pos += sizeof(sensorData.accelZ);
    
    memcpy(&sensorData.gyroX, &data[pos], sizeof(sensorData.gyroX));
    pos += sizeof(sensorData.gyroX);
    
    memcpy(&sensorData.gyroY, &data[pos], sizeof(sensorData.gyroY));
    pos += sizeof(sensorData.gyroY);
    
    memcpy(&sensorData.gyroZ, &data[pos], sizeof(sensorData.gyroZ));
    pos += sizeof(sensorData.gyroZ);
    
    memcpy(&sensorData.rollAngle, &data[pos], sizeof(sensorData.rollAngle));
    pos += sizeof(sensorData.rollAngle);
    
    memcpy(&sensorData.tiltAngle, &data[pos], sizeof(sensorData.tiltAngle));
    pos += sizeof(sensorData.tiltAngle);
    
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

#endif // !UNIT_TEST && (ENV_STM || ENV_ESP || ENV_TEENSY)