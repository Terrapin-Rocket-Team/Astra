//
// Created by aditya on 10/27/25.
//

#ifndef FILEHANDLER_H
#define FILEHANDLER_H

#include "LoggingBackend/ILogSink.h"
#include "LoggingBackend/SDFatBoilerplate.h"
#include <Arduino.h>


enum class StorageMedium{
        SD_Card,
        EMMC,
		Flash
};

namespace astra
{
    class FileHandler{
        private:
            StorageMedium _medium;                                                                                                            //Storage Medium (SD_Card or EMMC)
            SdFat _sd;                                                                                                                        //SdFat filesystem object
            File _file;                                                                                                                       //Currently open file handle
            bool _ready = false;                                                                                                              //Flag if handler is initialized
            bool _prefix;                                                                                                                     // Whether the sink wants file prefix
            const char *_path;                                                                                                                //File path for logging
            uint8_t _csPin;                                                                                                                   //Pin for the storage location

        public:
            FileHandler(const char *path, StorageMedium medium = StorageMedium::SD_Card, uint8_t csPin = SD_CS_PIN, bool prefix = false);
            ~FileHandler();                                                                                                                   //Desturctor - closes the file and cleans up the resources

            bool begin() override;                                                                                                            //Initialize the storage medium
            bool end() override;                                                                                                              //close the log file
            bool ok() const override;                                                                                                         //Check if handler is ready
            bool wantsPrefix() const override;                                                                                                //whether the sink wants prefix
            size_t write(uint8_t b) override;                                                                                                 //Write a single byte to the log file
            void flush() override;                                                                                                            //Flush any buffered data to the storage

        private:
            bool initSD();                                                                                                                    //Initialize the SD card
            bool initEMMC();
			bool initFlash()                                                                                                                  //Initialize EMMC storage
            bool openFile();                                                                                                                  //Open the log file for writing
    };
}


#endif //FILEHANDLER_H
