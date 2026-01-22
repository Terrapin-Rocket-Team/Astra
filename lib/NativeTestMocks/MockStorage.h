#ifndef MOCK_STORAGE_H
#define MOCK_STORAGE_H

#include <cstdio>
#include "RecordData/Storage/IStorage.h"
#include "RecordData/Storage/IFile.h"

class MockFile : public astra::IFile {
public:
    MockFile(const char* filename, const char* mode);
    ~MockFile();

    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    bool flush() override;

    int read() override;
    int readBytes(uint8_t *buffer, size_t length) override;
    int available() override;

    bool seek(uint32_t pos) override;
    uint32_t position() override;
    uint32_t size() override;
    bool close() override;

    bool isOpen() const override;

private:
    FILE* _file;
};

class MockStorage : public astra::IStorage {
public:
    bool begin() override;
    bool end() override;
    bool ok() const override;

    astra::IFile *openRead(const char *filename) override;
    astra::IFile *openWrite(const char *filename, bool append = true) override;

    bool exists(const char *filename) override;
    bool remove(const char *filename) override;
    bool mkdir(const char *path) override;
    bool rmdir(const char *path) override;
};

#endif // MOCK_STORAGE_H
