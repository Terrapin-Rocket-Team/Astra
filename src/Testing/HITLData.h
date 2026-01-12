//Created by Divyansh Srivastava on 1/11/2026
//This class will be for creating a HITLData type by inheritign from the Data class defined within RadioMessage
//through the repository: https://github.com/erodarob/RadioMessage.git
#ifndef HITL_DATA_H
#define HITL_DATA_H

#include <RadioMessage.h>

class HITLData : public Data {
    public: 
        uint16_t encode(uint8_t *data, uint16_t sz);
        uint16_t decode(uint8_t *data, uint16_t sz);
        uint16_t toJSON(char *json, uint16_t sz, int deviceId);
        uint16_t fromJSON(char *json, uint16_t sz, int &deviceId);
  

    private:
        //these are the data memebers that we need to run the Avionics algorithms on 
        
        float batteryVoltage;
        float temperature;
        float pressure;
        float altitude;
        float velocity;
        float accelx;
        float accely;
        float accelz;
        float gyrox;
        float gyroy;
        float gyroz;


};




#endif // HITL_DATA_H