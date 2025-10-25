#ifdef ENV_STM
#include "SDMMCBackend.h"

namespace astra
{
    bool SDMMCBackend::begin()
    {
        // SD_DETECT_NONE should only be used if there is a detect pin on the sd slot. With EMMC, there should not be a detect pin being used anyways
        return SD.begin(SD_DETECT_NONE);
    }
    // only call when a card is removed or to re-init a card
    bool SDMMCBackend::end()
    {
        return SD.end();
    }
    bool SDMMCBackend::write(char *data, char *filename)
    {
        file = SD.open(filename, FILE_WRITE);
        
        if(file){
            // send data to eMMC
            file.println(data);
            // make sure data gets written
            file.flush();
            // close out file once operations are finished
            file.close();
            
        }
        return true;
    }
    //TODO: figure out what to shove the data from the file into (buffer or serial)
    char *SDMMCBackend::read(char *filename)
    {

        char *fileData;
        file = SD.open(filename, FILE_READ);
        if(file){
            while(file.available()){
                file.read();

            }
        }
        return "ff";
    }
}
#endif