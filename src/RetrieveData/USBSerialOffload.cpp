#include "USBSerialOffload.h"
using namespace astra;

USBSerialOffload &astra::getDataRetrieverInstance()
{
    static USBSerialOffload instance;
    return instance;
}

bool USBSerialOffload::init()
{
    if (!sd.begin(SD_CONFIG))
    {
        sd.initErrorHalt(&Serial);
        return false;
    }

    Serial.println(F("Initialized SD Card Reader"));
    return true;
}
// TODO: maybe change this to a char* and return a big array??? or maybe handle read funcionality in the
bool USBSerialOffload::readFile(char *path)
{
    bool success = file.open(path, FILE_READ);
    if (!success)
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
        int n = file.fgets(currentLine, sizeof(currentLine), "\n");
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

bool USBSerialOffload::deleteFile(char *path)
{
    bool success = file.remove(path);
    return success;
}
// returns true if there is no serial connection or "cmd/quit" is typed
bool USBSerialOffload::handleChoices()
{
    if (Serial)
    {
        char input[40];
        // read bytes until a newline
        // leave room for a terminating NUL
        int i = Serial.readBytesUntil('\n', input, sizeof(input) - 1);
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
                else if(strcmp(cmd, "quit") == 0){
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

void USBSerialOffload::listFiles()
{
    // ls the size, date modified, and all files, including hidden
    bool success = sd.ls("/", LS_R);
    if (!success)
    {
        Serial.println("ERROR: Could not list files");
    }
}
