//Divyansh Srivastava 11/19/2025
//Copied over from old mmfs


#include "Data.h"
#include <Arduino.h>

#define LS_DATE 2
#define LS_SIZE 4

// Sensor data structure matching your BlueRaven columns
struct SensorData {
    float batteryVoltage;
    float altitude;
    float pressure;
    float temperature;
    float velocity;
    float angle;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float rollAngle, tiltAngle;
};

class SerialHandler
{
public:
    void handle();
    const char *getLastLine() const;
    
    // Decode incoming serial data into sensor structure
    uint16_t decode(uint8_t *data, uint16_t sz);
    
    // Get the latest decoded sensor data
    const SensorData& getSensorData() const { return sensorData; }
    
private:
    void parseSerialStream(); //parse incoming serial stream 
    void processCompleteMessage();
    
    // Command handlers, will probably not need because it will happen one at a time 
    void fetchList();
    void clearFiles();
    void removeFile(char *args);
    void latestFiles();
    void copyFile(char *args);
    
private:
    char line[2500];
    uint8_t buffer[512];      // Buffer for incoming serial data
    uint16_t bufferPos;       // Current position in buffer
    bool messageReady;        // Flag for complete message
    
    SensorData sensorData;    // Decoded sensor data
    
    // Message framing
    const char START_MARKER = '<';
    const char END_MARKER = '>';
};

SerialHandler &getSerialHandler();