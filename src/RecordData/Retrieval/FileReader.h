#ifndef FILE_READER_H
#define FILE_READER_H

#include "../Storage/IStorage.h"
#include "../Storage/IFile.h"
#include <Arduino.h>

// TODO: What is this file?

namespace astra
{

    /**
     * @brief Utility for reading files from storage backends
     *
     * Provides high-level file reading operations including:
     * - Reading entire files to Serial
     * - Line-by-line processing with callbacks
     * - Interactive command-line interface
     *
     * Works with any IStorage implementation (eMMC, SD, Flash).
     */
    class FileReader
    {
    private:
        static constexpr size_t READ_BUFFER_SIZE = 1024;
        static constexpr size_t LINE_BUFFER_SIZE = 1024;

        IStorage *_backend;

    public:
        /**
         * @brief Construct FileReader
         * @param backend Storage backend to read from (not owned by FileReader)
         */
        FileReader(IStorage *backend) : _backend(backend) {}

        /**
         * @brief Print entire file to Serial
         * @param filename Path to file
         * @return true if file was read successfully
         */
        bool printFile(const char *filename)
        {
            if (!_backend || !_backend->ok())
            {
                Serial.println("ERROR: Storage backend not ready");
                return false;
            }

            if (!_backend->exists(filename))
            {
                Serial.print("ERROR: File not found: ");
                Serial.println(filename);
                return false;
            }

            IFile *file = _backend->openRead(filename);
            if (!file || !file->isOpen())
            {
                Serial.println("ERROR: Failed to open file");
                return false;
            }

            Serial.println("|----------BOF----------|");

            uint8_t buffer[READ_BUFFER_SIZE];
            int bytesRead;
            while ((bytesRead = file->readBytes(buffer, sizeof(buffer))) > 0)
            {
                Serial.write(buffer, bytesRead);
            }

            Serial.println("|----------EOF----------|");

            file->close();
            delete file;
            return true;
        }

        /**
         * @brief Read file line by line with callback
         * @param filename Path to file
         * @param callback Function called for each line
         * @return true if file was read successfully
         */
        bool readLines(const char *filename, void (*callback)(const char *line))
        {
            if (!_backend || !_backend->ok())
                return false;
            if (!_backend->exists(filename))
                return false;

            IFile *file = _backend->openRead(filename);
            if (!file || !file->isOpen())
                return false;

            char lineBuffer[LINE_BUFFER_SIZE];
            int linePos = 0;

            while (file->available())
            {
                int c = file->read();
                if (c == -1)
                    break;

                if (c == '\n' || linePos >= (LINE_BUFFER_SIZE - 1))
                {
                    lineBuffer[linePos] = '\0';
                    callback(lineBuffer);
                    linePos = 0;
                }
                else if (c != '\r')
                {
                    lineBuffer[linePos++] = (char)c;
                }
            }

            // Process final line if it doesn't end with newline
            if (linePos > 0)
            {
                lineBuffer[linePos] = '\0';
                callback(lineBuffer);
            }

            file->close();
            delete file;
            return true;
        }

        /**
         * @brief Check if file exists
         * @param filename Path to file
         * @return true if file exists
         */
        bool exists(const char *filename)
        {
            if (!_backend || !_backend->ok())
                return false;
            return _backend->exists(filename);
        }

        /**
         * @brief Delete a file
         * @param filename Path to file
         * @return true if file was deleted successfully
         */
        bool deleteFile(const char *filename)
        {
            if (!_backend || !_backend->ok())
                return false;
            return _backend->remove(filename);
        }

        /**
         * @brief Interactive command-line interface
         *
         * Commands:
         * - cmd/sf <filename> : Show file contents
         * - cmd/rm <filename> : Remove file
         * - cmd/help : Show help
         * - cmd/quit : Exit
         */
        void handleCommands()
        {
            if (!_backend || !_backend->ok())
            {
                Serial.println("ERROR: Storage backend not ready");
                return;
            }

            Serial.println("File Reader CLI - Enter 'cmd/help' for commands");

            while (true)
            {
                if (Serial.available())
                {
                    String command = Serial.readStringUntil('\n');
                    command.trim();

                    if (command == "cmd/quit")
                    {
                        Serial.println("Exiting...");
                        break;
                    }
                    else if (command == "cmd/help")
                    {
                        Serial.println("Commands:");
                        Serial.println("  cmd/sf <filename> - Show file");
                        Serial.println("  cmd/rm <filename> - Remove file");
                        Serial.println("  cmd/help - Show this help");
                        Serial.println("  cmd/quit - Exit");
                    }
                    else if (command.startsWith("cmd/sf "))
                    {
                        String filename = command.substring(7);
                        filename.trim();
                        printFile(filename.c_str());
                    }
                    else if (command.startsWith("cmd/rm "))
                    {
                        String filename = command.substring(7);
                        filename.trim();
                        if (deleteFile(filename.c_str()))
                        {
                            Serial.print("Deleted: ");
                            Serial.println(filename);
                        }
                        else
                        {
                            Serial.print("Failed to delete: ");
                            Serial.println(filename);
                        }
                    }
                    else
                    {
                        Serial.print("Unknown command: ");
                        Serial.println(command);
                        Serial.println("Type 'cmd/help' for available commands");
                    }
                }
            }
        }
    };

} // namespace astra

#endif // FILE_READER_H
