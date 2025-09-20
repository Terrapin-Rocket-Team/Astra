#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"

// include other headers you need to test here

#include <cstdio>
#include <string>
#include <vector>
#include <fstream>

// ---

// Set up and global variables or mocks for testing here

static const char *kTestFile = "test_output.bin";

static std::vector<uint8_t> readAll(const char *path)
{
    std::ifstream ifs(path, std::ios::binary);
    std::vector<uint8_t> out;
    if (!ifs.is_open())
        return out;
    ifs.seekg(0, std::ios::end);
    auto len = static_cast<size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);
    out.resize(len);
    if (len)
        ifs.read(reinterpret_cast<char *>(out.data()), len);
    return out;
}

// ---

// These two functions are called before and after each test function, and are required in unity, even if empty.
void setUp(void)
{
    // set stuff up before each test here, if needed
    std::remove(kTestFile);
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
}
// ---

// Test functions must be void and take no arguments, put them here

void test_begin_ok_end_cycle()
{
    NativeFileLog log(kTestFile);
    TEST_ASSERT_FALSE(log.ok());
    TEST_ASSERT_TRUE(log.begin());
    TEST_ASSERT_TRUE(log.ok());
    TEST_ASSERT_TRUE(log.end());
    TEST_ASSERT_FALSE(log.ok());
}

void test_write_byte_and_block()
{
    NativeFileLog log(kTestFile);
    TEST_ASSERT_TRUE(log.begin());
    TEST_ASSERT_TRUE(log.ok());

    // byte writes
    TEST_ASSERT_EQUAL_size_t(1, log.write((uint8_t)'A'));
    TEST_ASSERT_EQUAL_size_t(1, log.write((uint8_t)'\n'));

    // block write
    const uint8_t buf[] = {0x01, 0x02, 0x03, 0x04};
    TEST_ASSERT_EQUAL_size_t(sizeof(buf), log.write(buf, sizeof(buf)));

    log.flush();
    log.end();

    auto data = readAll(kTestFile);
    TEST_ASSERT_EQUAL_size_t(2 + sizeof(buf), data.size());
    TEST_ASSERT_EQUAL_UINT8('A', data[0]);
    TEST_ASSERT_EQUAL_UINT8('\n', data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, data[2]);
    TEST_ASSERT_EQUAL_UINT8(0x02, data[3]);
    TEST_ASSERT_EQUAL_UINT8(0x03, data[4]);
    TEST_ASSERT_EQUAL_UINT8(0x04, data[5]);
}

void test_print_and_println()
{
    NativeFileLog log(kTestFile);
    TEST_ASSERT_TRUE(log.begin());

    // Use Print API via ILogSink base
    TEST_ASSERT_GREATER_OR_EQUAL_size_t(5, log.print("hello"));
    TEST_ASSERT_GREATER_OR_EQUAL_size_t(1, log.println(" world"));
    log.flush();
    log.end();

    auto s = readAll(kTestFile);
    std::string out(s.begin(), s.end());
    TEST_ASSERT_EQUAL_STRING("hello world\n", out.c_str());
}

// ---

// This is the main function that runs all the tests. It should be the last thing in the file.
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_begin_ok_end_cycle);
    RUN_TEST(test_write_byte_and_block);
    RUN_TEST(test_print_and_println);
    
    return UNITY_END();
}
// ---