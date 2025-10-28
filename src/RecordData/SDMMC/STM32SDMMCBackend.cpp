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
        if(!SD.exists(filename)){
            return false; // should only write to files which exist already
        }
        file = SD.open(filename, FILE_WRITE);
        
        if(file){
            // send data to eMMC
            file.println(data);
            // make sure data gets written
            file.flush();
            // close out file once operations are finished
            file.close();
            return true;
        } else {
            return false;
        }
    }
    // will return true if a file is created or already exists
    bool SDMMCBackend::createFile(char *filename){
        file = SD.open(filename);
        if(file){
            file.close();
            return true;
        } else {
            file.close();
            return false;
        }
    }
}
#endif