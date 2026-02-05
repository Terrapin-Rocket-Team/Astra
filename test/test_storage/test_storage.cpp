#include <unity.h>
#include <string>
#include <map>
#include <vector>
#include <cstring>
#include "RecordData/Storage/IStorage.h"
#include "RecordData/Storage/IFile.h"

using namespace astra;

// --- Mock File Implementation ---

class MockFile : public IFile
{
private:
    std::vector<uint8_t> data;
    size_t readPos = 0;
    bool isOpen_ = false;

public:
    MockFile() {}

    void open() { isOpen_ = true; readPos = 0; }

    // Writing
    size_t write(uint8_t b) override
    {
        if (!isOpen_) return 0;
        data.push_back(b);
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override
    {
        if (!isOpen_) return 0;
        data.insert(data.end(), buffer, buffer + size);
        return size;
    }

    bool flush() override
    {
        return isOpen_;
    }

    // Reading
    int read() override
    {
        if (!isOpen_ || readPos >= data.size()) return -1;
        return data[readPos++];
    }

    int readBytes(uint8_t *buffer, size_t length) override
    {
        if (!isOpen_) return 0;

        size_t bytesRead = 0;
        while (bytesRead < length && readPos < data.size()) {
            buffer[bytesRead++] = data[readPos++];
        }
        return bytesRead;
    }

    int available() override
    {
        if (!isOpen_) return 0;
        return data.size() - readPos;
    }

    // File operations
    bool seek(uint32_t pos) override
    {
        if (!isOpen_) return false;
        if (pos > data.size()) return false;
        readPos = pos;
        return true;
    }

    uint32_t position() override
    {
        return readPos;
    }

    uint32_t size() override
    {
        return data.size();
    }

    bool close() override
    {
        isOpen_ = false;
        return true;
    }

    // Status
    bool isOpen() const override
    {
        return isOpen_;
    }

    // Test helpers
    std::string getContent() const
    {
        return std::string(data.begin(), data.end());
    }

    void setContent(const std::string &content)
    {
        data.assign(content.begin(), content.end());
        readPos = 0;
    }
};

// --- Mock Storage Implementation ---

class MockStorage : public IStorage
{
private:
    std::map<std::string, MockFile*> files;
    bool begun = false;
    bool shouldFailBegin = false;

public:
    MockStorage() {}

    ~MockStorage()
    {
        for (auto &pair : files) {
            delete pair.second;
        }
    }

    void setShouldFailBegin(bool fail) { shouldFailBegin = fail; }

    // Lifecycle
    bool begin() override
    {
        if (shouldFailBegin) return false;
        begun = true;
        return true;
    }

    bool end() override
    {
        begun = false;
        return true;
    }

    bool ok() const override
    {
        return begun;
    }

    // File operations
    IFile *openRead(const char *filename) override
    {
        if (!begun) return nullptr;

        auto it = files.find(filename);
        if (it == files.end()) return nullptr;

        it->second->open();
        it->second->seek(0);
        return it->second;
    }

    IFile *openWrite(const char *filename, bool append = true) override
    {
        if (!begun) return nullptr;

        auto it = files.find(filename);
        if (it == files.end()) {
            // Create new file
            MockFile *newFile = new MockFile();
            files[filename] = newFile;
            newFile->open();
            return newFile;
        }

        it->second->open();
        if (!append) {
            // Clear existing content
            it->second->setContent("");
        }
        return it->second;
    }

    // Filesystem operations
    bool exists(const char *filename) override
    {
        if (!begun) return false;
        return files.find(filename) != files.end();
    }

    bool remove(const char *filename) override
    {
        if (!begun) return false;

        auto it = files.find(filename);
        if (it == files.end()) return false;

        delete it->second;
        files.erase(it);
        return true;
    }

    bool mkdir(const char *path) override
    {
        if (!begun) return false;
        // Mock implementation - just return success
        return true;
    }

    bool rmdir(const char *path) override
    {
        if (!begun) return false;
        // Mock implementation - just return success
        return true;
    }

    // Test helpers
    size_t getFileCount() const { return files.size(); }
};

// setUp and tearDown
void setUp(void) {}
void tearDown(void) {}

// --- IFile Tests ---

void test_file_default_state(void)
{
    MockFile file;
    TEST_ASSERT_FALSE(file.isOpen());
    TEST_ASSERT_EQUAL(0, file.size());
    TEST_ASSERT_EQUAL(0, file.available());
}

void test_file_open_close(void)
{
    MockFile file;
    file.open();
    TEST_ASSERT_TRUE(file.isOpen());

    file.close();
    TEST_ASSERT_FALSE(file.isOpen());
}

void test_file_write_single_byte(void)
{
    MockFile file;
    file.open();

    size_t written = file.write('A');
    TEST_ASSERT_EQUAL(1, written);
    TEST_ASSERT_EQUAL(1, file.size());
}

void test_file_write_buffer(void)
{
    MockFile file;
    file.open();

    const uint8_t data[] = {1, 2, 3, 4, 5};
    size_t written = file.write(data, 5);

    TEST_ASSERT_EQUAL(5, written);
    TEST_ASSERT_EQUAL(5, file.size());
}

void test_file_write_when_closed(void)
{
    MockFile file;

    size_t written = file.write('A');
    TEST_ASSERT_EQUAL(0, written);
}

void test_file_read_single_byte(void)
{
    MockFile file;
    file.setContent("ABC");
    file.open();

    int byte = file.read();
    TEST_ASSERT_EQUAL('A', byte);

    byte = file.read();
    TEST_ASSERT_EQUAL('B', byte);
}

void test_file_read_bytes(void)
{
    MockFile file;
    file.setContent("Hello");
    file.open();

    uint8_t buffer[10];
    int read = file.readBytes(buffer, 5);

    TEST_ASSERT_EQUAL(5, read);
    TEST_ASSERT_EQUAL_CHAR_ARRAY("Hello", buffer, 5);
}

void test_file_read_beyond_end(void)
{
    MockFile file;
    file.setContent("AB");
    file.open();

    file.read();  // A
    file.read();  // B
    int byte = file.read();  // Should return -1

    TEST_ASSERT_EQUAL(-1, byte);
}

void test_file_available(void)
{
    MockFile file;
    file.setContent("Hello");
    file.open();

    TEST_ASSERT_EQUAL(5, file.available());

    file.read();
    TEST_ASSERT_EQUAL(4, file.available());

    file.read();
    file.read();
    TEST_ASSERT_EQUAL(2, file.available());
}

void test_file_seek(void)
{
    MockFile file;
    file.setContent("ABCDE");
    file.open();

    bool success = file.seek(2);
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_EQUAL(2, file.position());

    int byte = file.read();
    TEST_ASSERT_EQUAL('C', byte);
}

void test_file_seek_beyond_end(void)
{
    MockFile file;
    file.setContent("ABC");
    file.open();

    bool success = file.seek(10);
    TEST_ASSERT_FALSE(success);
}

void test_file_position(void)
{
    MockFile file;
    file.setContent("ABC");
    file.open();

    TEST_ASSERT_EQUAL(0, file.position());

    file.read();
    TEST_ASSERT_EQUAL(1, file.position());

    file.read();
    TEST_ASSERT_EQUAL(2, file.position());
}

void test_file_flush(void)
{
    MockFile file;
    file.open();
    file.write('A');

    bool success = file.flush();
    TEST_ASSERT_TRUE(success);
}

void test_file_bool_operator(void)
{
    MockFile file;
    TEST_ASSERT_FALSE((bool)file);

    file.open();
    TEST_ASSERT_TRUE((bool)file);

    file.close();
    TEST_ASSERT_FALSE((bool)file);
}

void test_file_write_and_read_roundtrip(void)
{
    MockFile file;
    file.open();

    const char *data = "Test Data 123";
    file.write((const uint8_t *)data, strlen(data));

    file.close();
    file.open();
    file.seek(0);

    uint8_t buffer[20];
    int read = file.readBytes(buffer, strlen(data));

    TEST_ASSERT_EQUAL(strlen(data), read);
    TEST_ASSERT_EQUAL_CHAR_ARRAY(data, buffer, strlen(data));
}

// --- IStorage Tests ---

void test_storage_default_state(void)
{
    MockStorage storage;
    TEST_ASSERT_FALSE(storage.ok());
}

void test_storage_begin_end(void)
{
    MockStorage storage;

    bool success = storage.begin();
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_TRUE(storage.ok());

    storage.end();
    TEST_ASSERT_FALSE(storage.ok());
}

void test_storage_begin_failure(void)
{
    MockStorage storage;
    storage.setShouldFailBegin(true);

    bool success = storage.begin();
    TEST_ASSERT_FALSE(success);
    TEST_ASSERT_FALSE(storage.ok());
}

void test_storage_open_write_new_file(void)
{
    MockStorage storage;
    storage.begin();

    IFile *file = storage.openWrite("test.txt");
    TEST_ASSERT_NOT_NULL(file);
    TEST_ASSERT_TRUE(file->isOpen());

    TEST_ASSERT_TRUE(storage.exists("test.txt"));
}

void test_storage_open_read_existing_file(void)
{
    MockStorage storage;
    storage.begin();

    // Create file
    IFile *writeFile = storage.openWrite("data.txt");
    writeFile->write((const uint8_t *)"content", 7);
    writeFile->close();

    // Read file
    IFile *readFile = storage.openRead("data.txt");
    TEST_ASSERT_NOT_NULL(readFile);
    TEST_ASSERT_TRUE(readFile->isOpen());

    uint8_t buffer[10];
    int read = readFile->readBytes(buffer, 7);
    TEST_ASSERT_EQUAL(7, read);
    TEST_ASSERT_EQUAL_CHAR_ARRAY("content", buffer, 7);
}

void test_storage_open_read_nonexistent_file(void)
{
    MockStorage storage;
    storage.begin();

    IFile *file = storage.openRead("nonexistent.txt");
    TEST_ASSERT_NULL(file);
}

void test_storage_open_write_append(void)
{
    MockStorage storage;
    storage.begin();

    // Create file with initial content
    IFile *file1 = storage.openWrite("log.txt");
    file1->write((const uint8_t *)"Line1\n", 6);
    file1->close();

    // Append to file
    IFile *file2 = storage.openWrite("log.txt", true);
    file2->write((const uint8_t *)"Line2\n", 6);
    file2->close();

    // Read back
    IFile *readFile = storage.openRead("log.txt");
    uint8_t buffer[20];
    int read = readFile->readBytes(buffer, 12);

    TEST_ASSERT_EQUAL(12, read);
    TEST_ASSERT_EQUAL_CHAR_ARRAY("Line1\nLine2\n", buffer, 12);
}

void test_storage_open_write_overwrite(void)
{
    MockStorage storage;
    storage.begin();

    // Create file with initial content
    IFile *file1 = storage.openWrite("data.txt");
    file1->write((const uint8_t *)"OldData", 7);
    file1->close();

    // Overwrite file
    IFile *file2 = storage.openWrite("data.txt", false);
    file2->write((const uint8_t *)"New", 3);
    file2->close();

    // Read back
    IFile *readFile = storage.openRead("data.txt");
    uint8_t buffer[10];
    int read = readFile->readBytes(buffer, 10);

    TEST_ASSERT_EQUAL(3, read);
    TEST_ASSERT_EQUAL_CHAR_ARRAY("New", buffer, 3);
}

void test_storage_exists(void)
{
    MockStorage storage;
    storage.begin();

    TEST_ASSERT_FALSE(storage.exists("new.txt"));

    IFile *file = storage.openWrite("new.txt");
    file->close();

    TEST_ASSERT_TRUE(storage.exists("new.txt"));
}

void test_storage_remove(void)
{
    MockStorage storage;
    storage.begin();

    // Create file
    IFile *file = storage.openWrite("temp.txt");
    file->close();

    TEST_ASSERT_TRUE(storage.exists("temp.txt"));

    // Remove file
    bool success = storage.remove("temp.txt");
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_FALSE(storage.exists("temp.txt"));
}

void test_storage_remove_nonexistent(void)
{
    MockStorage storage;
    storage.begin();

    bool success = storage.remove("nonexistent.txt");
    TEST_ASSERT_FALSE(success);
}

void test_storage_mkdir(void)
{
    MockStorage storage;
    storage.begin();

    bool success = storage.mkdir("/data");
    TEST_ASSERT_TRUE(success);
}

void test_storage_rmdir(void)
{
    MockStorage storage;
    storage.begin();

    bool success = storage.rmdir("/data");
    TEST_ASSERT_TRUE(success);
}

void test_storage_operations_when_not_begun(void)
{
    MockStorage storage;

    TEST_ASSERT_NULL(storage.openWrite("test.txt"));
    TEST_ASSERT_NULL(storage.openRead("test.txt"));
    TEST_ASSERT_FALSE(storage.exists("test.txt"));
    TEST_ASSERT_FALSE(storage.remove("test.txt"));
    TEST_ASSERT_FALSE(storage.mkdir("/data"));
    TEST_ASSERT_FALSE(storage.rmdir("/data"));
}

void test_storage_multiple_files(void)
{
    MockStorage storage;
    storage.begin();

    // Create multiple files
    IFile *f1 = storage.openWrite("file1.txt");
    f1->write((const uint8_t *)"1", 1);
    f1->close();

    IFile *f2 = storage.openWrite("file2.txt");
    f2->write((const uint8_t *)"2", 1);
    f2->close();

    IFile *f3 = storage.openWrite("file3.txt");
    f3->write((const uint8_t *)"3", 1);
    f3->close();

    // Verify all exist
    TEST_ASSERT_TRUE(storage.exists("file1.txt"));
    TEST_ASSERT_TRUE(storage.exists("file2.txt"));
    TEST_ASSERT_TRUE(storage.exists("file3.txt"));

    // Remove one
    storage.remove("file2.txt");

    TEST_ASSERT_TRUE(storage.exists("file1.txt"));
    TEST_ASSERT_FALSE(storage.exists("file2.txt"));
    TEST_ASSERT_TRUE(storage.exists("file3.txt"));
}

void test_storage_reopen_file_multiple_times(void)
{
    MockStorage storage;
    storage.begin();

    // Write
    IFile *f1 = storage.openWrite("data.txt");
    f1->write((const uint8_t *)"ABC", 3);
    f1->close();

    // Read first time
    IFile *f2 = storage.openRead("data.txt");
    int byte = f2->read();
    TEST_ASSERT_EQUAL('A', byte);
    f2->close();

    // Read second time (should reset position)
    IFile *f3 = storage.openRead("data.txt");
    byte = f3->read();
    TEST_ASSERT_EQUAL('A', byte);
    f3->close();
}

void test_storage_write_read_binary_data(void)
{
    MockStorage storage;
    storage.begin();

    const uint8_t binaryData[] = {0x00, 0xFF, 0xAA, 0x55, 0x12, 0x34};

    IFile *wFile = storage.openWrite("binary.dat");
    wFile->write(binaryData, 6);
    wFile->close();

    IFile *rFile = storage.openRead("binary.dat");
    uint8_t buffer[10];
    int read = rFile->readBytes(buffer, 6);

    TEST_ASSERT_EQUAL(6, read);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(binaryData, buffer, 6);
}

void test_storage_large_file(void)
{
    MockStorage storage;
    storage.begin();

    IFile *file = storage.openWrite("large.dat");

    // Write 1000 bytes
    for (int i = 0; i < 1000; i++) {
        file->write((uint8_t)(i % 256));
    }
    file->close();

    // Read back
    IFile *rFile = storage.openRead("large.dat");
    TEST_ASSERT_EQUAL(1000, rFile->size());
    TEST_ASSERT_EQUAL(1000, rFile->available());

    // Verify content
    for (int i = 0; i < 1000; i++) {
        int byte = rFile->read();
        TEST_ASSERT_EQUAL(i % 256, byte);
    }
}

// Main function
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // IFile tests
    RUN_TEST(test_file_default_state);
    RUN_TEST(test_file_open_close);
    RUN_TEST(test_file_write_single_byte);
    RUN_TEST(test_file_write_buffer);
    RUN_TEST(test_file_write_when_closed);
    RUN_TEST(test_file_read_single_byte);
    RUN_TEST(test_file_read_bytes);
    RUN_TEST(test_file_read_beyond_end);
    RUN_TEST(test_file_available);
    RUN_TEST(test_file_seek);
    RUN_TEST(test_file_seek_beyond_end);
    RUN_TEST(test_file_position);
    RUN_TEST(test_file_flush);
    RUN_TEST(test_file_bool_operator);
    RUN_TEST(test_file_write_and_read_roundtrip);

    // IStorage tests
    RUN_TEST(test_storage_default_state);
    RUN_TEST(test_storage_begin_end);
    RUN_TEST(test_storage_begin_failure);
    RUN_TEST(test_storage_open_write_new_file);
    RUN_TEST(test_storage_open_read_existing_file);
    RUN_TEST(test_storage_open_read_nonexistent_file);
    RUN_TEST(test_storage_open_write_append);
    RUN_TEST(test_storage_open_write_overwrite);
    RUN_TEST(test_storage_exists);
    RUN_TEST(test_storage_remove);
    RUN_TEST(test_storage_remove_nonexistent);
    RUN_TEST(test_storage_mkdir);
    RUN_TEST(test_storage_rmdir);
    RUN_TEST(test_storage_operations_when_not_begun);
    RUN_TEST(test_storage_multiple_files);
    RUN_TEST(test_storage_reopen_file_multiple_times);
    RUN_TEST(test_storage_write_read_binary_data);
    RUN_TEST(test_storage_large_file);

    UNITY_END();

    return 0;
}
