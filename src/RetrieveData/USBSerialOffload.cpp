
#include "USBSerialOffload.h"
using namespace astra;

// Store error strings in flash to save RAM.
#define sd_print_error(s) sd.errorHalt(&Serial, F(s))

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
    Serial.println(F("Initialized SD Card"));
    return true;
}

// Check for extra characters in field or find minus sign.
// Given from ReadCSV
char *USBSerialOffload::skipSpace(char *str)
{
    while (isspace(*str))
        str++;
    return str;
}
// given from readCSV Example
bool USBSerialOffload::parseLine(char *str)
{
    char *ptr;

    // Set strtok start of line.
    str = strtok(str, ",");
    if (!str)
        return false;

    // Print text field.
    Serial.println(str);

    // Subsequent calls to strtok expects a null pointer.
    str = strtok(nullptr, ",");
    if (!str)
        return false;

    // Convert string to long integer.
    int32_t i32 = strtol(str, &ptr, 0);
    if (str == ptr || *skipSpace(ptr))
        return false;
    Serial.println(i32);

    str = strtok(nullptr, ",");
    if (!str)
        return false;

    // strtoul accepts a leading minus with unexpected results.
    if (*skipSpace(str) == '-')
        return false;

    // Convert string to unsigned long integer.
    uint32_t u32 = strtoul(str, &ptr, 0);
    if (str == ptr || *skipSpace(ptr))
        return false;
    Serial.println(u32);

    str = strtok(nullptr, ",");
    if (!str)
        return false;

    // Convert string to double.
    double d = strtod(str, &ptr);
    if (str == ptr || *skipSpace(ptr))
        return false;
    Serial.println(d);

    // Check for extra fields.
    return strtok(nullptr, ",") == nullptr;
}
//------------------------------------------------------------------------------
// TODO: maybe change this to a char* and return a big array??? or maybe handle read funcionality in the
bool USBSerialOffload::readFile(char *path)
{
    if (!validateFileName(path))
    {
        sd_print_error("|---File not found---|");
        return false;
    }
    file.open(path, FILE_READ);
    char currentLine[LINE_LENGTH];
    // for python program parsing
    Serial.println("----------BOF----------");
    while (file.available())
    {
        // get the current line (buffer size), stores the array in currentLine, stopping the line at any newline char
        int n = file.fgets(currentLine, sizeof(currentLine), "\n");
        // fgets returns -1 for no data, 0 for EOF, length of string otherwise
        if (n <= 0)
        {
            // formatting is for python program parsing
            sd_print_error("|---No data read from file---|");
        }
        // hopefully this never happens
        if (currentLine[n - 1] != '\n' && n == (sizeof(currentLine) - 1))
        {
            sd_print_error("|---line too long---|");
        }
        if (currentLine[n - 1] == '\n')
        {
            // Remove new line.
            currentLine[n - 1] = 0;
        }
        // parseLine prints out the contents of the line
        if (!parseLine(currentLine))
        {
            sd_print_error("|---parseLine failed---|");
        }
        Serial.println();
    }
    file.close();
    // for python program parsing
    Serial.println("----------EOF----------");
    return true;
}

bool USBSerialOffload::validateFileName(char *name)
{
    return file.exists(name);
}

bool USBSerialOffload::deleteFile(char *path)
{
    if(validateFileName(path)){
        file.remove(path);
        return true;
    } else {
        return false;
    }
}

void USBSerialOffload::handleChoices()
{
    if (Serial.available())
    {
        char input[40];
        int i = Serial.readBytesUntil('\n', input, sizeof(input));
        if (strncmp("cmd/", input, 4) == 0)
        {
            input[i] = '\0';
            char *cmd = strtok(input+4, " ");
            char *args = strtok(nullptr, "");
            if (strcmp(cmd, "ls") == 0)
            {
                Serial.println("ok ls");
                listFiles();
            }
            else if (strcmp(cmd, "rm") == 0)
            {
                Serial.println("ok rm");
                deleteFile(args);
            }
            else if (strcmp(cmd, "cp") == 0)
            {
                Serial.println("ok cp");
                readFile(args);
            }
            else if (strcmp(cmd, "latest") == 0)
            {
                Serial.println("ok latest");
                listFiles();
            }
            else
            {
                Serial.printf("no cmd: %s\n", cmd);
            }
        } else {
            // TODO: figure out this :/
        }
    }
}

void USBSerialOffload::listFiles()
{
    // ls the size, date modified, and all files, including hidden
    file.ls(LS_SIZE | LS_A | LS_DATE);
}
