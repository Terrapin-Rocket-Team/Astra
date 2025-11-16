#ifdef ENV_STM
#include "SDMMCBackend.h"

namespace astra
{
    bool SDMMCBackend::begin()
    {
        // TODO: the following pins should be set by the USER, not the library
        //  set data pins
        SD.setDx(PC8, PC9, PC10, PC11);
        // clock pin
        SD.setCK(PC12);
        // CMD pin
        SD.setCMD(PD2);
        /*
            SD_DETECT_NONE should only be used if there is a detect pin on the sd slot.
            With EMMC, there should not be a detect pin being used anyways.
            Most SD card slots do not have a detect pin either
        */
        return SD.begin(SD_DETECT_NONE);
    }
    // only call when a card is removed or to re-init a card
    bool SDMMCBackend::end()
    {
        return SD.end();
    }
    bool SDMMCBackend::write(char *data, char *filename)
    {
        if (!SD.exists(filename))
        {
            return false; // should only write to files which exist already
        }
        file = SD.open(filename, FILE_WRITE);

        if (file)
        {
            // go to the end of the file
            file.seek(file.size());
            file.println(data);
            // make sure data gets written
            file.flush();
            file.close();
            return true;
        }
        else
        {
            return false;
        }
    }
    // will return true if a file is created. Will return false if a file exists already
    bool SDMMCBackend::createFile(char *filename)
    {
        if (SD.exists(filename))
        {
            return false;
        }
        file = SD.open(filename);
        if (file)
        {
            file.close();
            return true;
        }
        else
        {
            file.close();
            return false;
        }
    }
    void SDMMCBackend::listFiles()
    {
        file = SD.open("/");
        if (file)
        {
            // will print to serial
            file.ls(LS_DATE | LS_SIZE, TAB_SIZE);
        }
    }
    // Copied over from RetrieveSDCardData
    bool SDMMCBackend::readFile(char *filename)
    {
        file = SD.open(filename, FILE_READ);
        if (!file)
        {
            Serial.println("ERROR: File name not found");
            return false;
        }
        char currentLine[LINE_LENGTH];
        // for python program parsing
        Serial.println("|----------BOF----------|");
        while (file.available())
        {
            // get the current line (buffer size), stores the array in currentLine, stopping the line at any newline char
            int n = file.readBytesUntil('\n', currentLine, sizeof(currentLine));

            // fgets returns -1 for no data, 0 for EOF, length of string otherwise
            if (n <= 0)
            {
                // formatting is for python program parsing
                Serial.println("|---No data read from file---|");
                return false;
            }
            // hopefully this never happens. Essentially the length of one line exceeds the buffer size
            if (n == (sizeof(currentLine) - 1))
            {
                Serial.println("|---line too long---|");
                return false;
            }
            if (currentLine[n - 1] == '\n')
            {
                // Remove new line.
                currentLine[n - 1] = 0;
            }
            // parseLine prints out the contents of the line
            Serial.println(currentLine);
        }
        // for python program parsing
        Serial.println("|----------EOF----------|");
        file.close();
        return true;
    }
    bool SDMMCBackend::deleteFile(char *filename)
    {
        return SD.remove(filename);
    }
    // Copied over from RetrieveSDCardData    
    bool SDMMCBackend::handleChoices()
    {
        if (Serial)
        {
            char input[40];
            // read bytes until a newline
            // leave room for a terminating NUL
            int i = file.readBytesUntil('\n', input, sizeof(input) - 1);
            // Only process when we read a new input
            if (i > 0)
            {
                // terminate input with null char
                input[i] = '\0';
                // strip CRLF
                if (i > 0 && input[i - 1] == '\r')
                {
                    input[i - 1] = '\0';
                }
                if (strncmp("cmd/", input, 4) == 0)
                {
                    char *cmd = strtok(input + 4, " ");
                    char *args = strtok(nullptr, "");

                    if (strcmp(cmd, "ls") == 0)
                    {
                        listFiles();
                    }
                    else if (strcmp(cmd, "rm") == 0)
                    {
                        Serial.print("Deleting file: " + String(args));
                        bool success = deleteFile(args);
                        if (success)
                        {
                            Serial.println("Successfully deleted file");
                        }
                        else
                        {
                            Serial.println("ERROR: File not deleted: " + String(args));
                        }
                    }
                    else if (strcmp(cmd, "sf") == 0)
                    {
                        Serial.print("Copying File: ");
                        Serial.println(args);
                        bool success = readFile(args);
                        if (success)
                        {
                            Serial.println("Done sending file");
                        }
                        else
                        {
                            Serial.println("Could not send file");
                        }
                    }
                    else if (strcmp(cmd, "help") == 0)
                    {
                        Serial.println("Choices are:\nls - list all files on sd card\nrm - remove file on disk\nsf - send file to computer\nquit\nhelp - print this statement");
                    }
                    else if (strcmp(cmd, "quit") == 0)
                    {
                        // return true when quit is entered
                        return true;
                    }
                    return false;
                }
            }
            return false;
        }
        else
        {
            return true;
        }
    }
    bool SDMMCBackend::exists(char *filename)
    {
        return SD.exists(filename);
    }
}

#endif