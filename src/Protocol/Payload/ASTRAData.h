//Created by Divyansh Srivastava on 1/13/2026
//This is the header file for the ASTRAData class defined within ASTRAData.cpp
//This class is tied wiht the entire HITL system and will be used to hold filtered sensor outputs after 
//the state algorithm has run
#ifndef ASTRA_DATA_H
#define ASTRA_DATA_H

#include <Data.h>


namespace astra {
    
    class ASTRAData : public Data {
        public: 
            uint16_t encode(uint8_t *data, uint16_t sz);
            uint16_t decode(uint8_t *data, uint16_t sz);
            uint16_t toJSON(char *json, uint16_t sz, int deviceId);
            uint16_t fromJSON(char *json, uint16_t sz, int &deviceId);
    

        private:
            //these are the data memebers that we need to run the Avionics algorithms on 
    };

}
#endif // ASTRA_DATA_H