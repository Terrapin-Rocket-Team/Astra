#include <unity.h>
#include <cstring>
#include "Testing/HITLParser.h"
#include "Sensors/HITL/HITLSensorBuffer.h"

using namespace astra;

void setUp(void) {
    // Reset HITL buffer before each test
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    buffer.dataReady = false;
    memset(&buffer.data, 0, sizeof(buffer.data));
}

void tearDown(void) {
}

//------------------------------------------------------------------------------
// Test: Parse valid HITL data without prefix
//------------------------------------------------------------------------------
void test_parse_valid_data_without_prefix() {
    const char* data = "1.234,0.5,1.0,9.81,0.1,0.2,0.3,20.0,10.0,-45.0,1013.25,25.5,38.123,-122.456,100.5,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.234, timestamp);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_TRUE(buffer.dataReady);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.234, buffer.data.timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.5, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.0, buffer.data.accel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 9.81, buffer.data.accel.z());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.1, buffer.data.gyro.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.2, buffer.data.gyro.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.3, buffer.data.gyro.z());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 20.0, buffer.data.mag.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 10.0, buffer.data.mag.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, -45.0, buffer.data.mag.z());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1013.25, buffer.data.pressure);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 25.5, buffer.data.temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 38.123, buffer.data.gps_lat);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -122.456, buffer.data.gps_lon);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 100.5, buffer.data.gps_alt);
    TEST_ASSERT_EQUAL(1, buffer.data.gps_fix);
    TEST_ASSERT_EQUAL(8, buffer.data.gps_fix_quality);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 90.0, buffer.data.gps_heading);
}

//------------------------------------------------------------------------------
// Test: Parse without timestamp extraction
//------------------------------------------------------------------------------
void test_parse_without_timestamp_extraction() {
    const char* data = "2.5,0.0,0.0,9.8,0.0,0.0,0.0,15.0,5.0,-40.0,1000.0,20.0,40.0,-120.0,50.0,1,6,45.0";

    bool result = HITLParser::parse(data);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_TRUE(buffer.dataReady);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.5, buffer.data.timestamp);
}

//------------------------------------------------------------------------------
// Test: Parse with null data
//------------------------------------------------------------------------------
void test_parse_null_data() {
    double timestamp;

    bool result = HITLParser::parse(nullptr, timestamp);

    TEST_ASSERT_FALSE(result);
}

//------------------------------------------------------------------------------
// Test: Parse with incomplete data (too few fields)
//------------------------------------------------------------------------------
void test_parse_incomplete_data() {
    const char* data = "1.0,2.0,3.0,4.0";  // Only 4 fields instead of 18
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

//------------------------------------------------------------------------------
// Test: Parse with malformed data
//------------------------------------------------------------------------------
void test_parse_malformed_data() {
    const char* data = "not,valid,numbers,at,all";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

//------------------------------------------------------------------------------
// Test: ParseAndInject with valid full line (with prefix)
//------------------------------------------------------------------------------
void test_parseAndInject_valid_with_prefix() {
    const char* line = "HITL/3.14,1.0,2.0,3.0,0.1,0.2,0.3,10.0,20.0,30.0,1015.0,22.0,45.0,-123.0,200.0,1,10,180.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.14, timestamp);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_TRUE(buffer.dataReady);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.14, buffer.data.timestamp);
}

//------------------------------------------------------------------------------
// Test: ParseAndInject without timestamp extraction
//------------------------------------------------------------------------------
void test_parseAndInject_without_timestamp() {
    const char* line = "HITL/5.0,0.0,0.0,9.8,0.0,0.0,0.0,15.0,5.0,-40.0,1010.0,18.0,38.0,-122.0,150.0,1,7,270.0";

    bool result = HITLParser::parseAndInject(line);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 5.0, buffer.data.timestamp);
}

//------------------------------------------------------------------------------
// Test: ParseAndInject with invalid prefix
//------------------------------------------------------------------------------
void test_parseAndInject_invalid_prefix() {
    const char* line = "INVALID/1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    TEST_ASSERT_FALSE(result);
}

//------------------------------------------------------------------------------
// Test: ParseAndInject with missing prefix
//------------------------------------------------------------------------------
void test_parseAndInject_missing_prefix() {
    const char* line = "1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    TEST_ASSERT_FALSE(result);
}

//------------------------------------------------------------------------------
// Test: Parse extremes - very large values
//------------------------------------------------------------------------------
void test_parse_extreme_values() {
    const char* data = "9999.99,100.0,200.0,300.0,50.0,60.0,70.0,1000.0,2000.0,3000.0,2000.0,100.0,90.0,180.0,10000.0,1,12,359.9";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 9999.99, timestamp);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 100.0, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 10000.0, buffer.data.gps_alt);
}

//------------------------------------------------------------------------------
// Test: Parse with negative values
//------------------------------------------------------------------------------
void test_parse_negative_values() {
    const char* data = "0.5,-9.8,-1.0,-2.0,-0.5,-0.6,-0.7,-15.0,-20.0,-50.0,950.0,-10.0,-45.0,-122.0,0.0,0,0,0.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, -9.8, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, -10.0, buffer.data.temperature);
    TEST_ASSERT_EQUAL(0, buffer.data.gps_fix);
}

//------------------------------------------------------------------------------
// Test: Parse with zero values
//------------------------------------------------------------------------------
void test_parse_zero_values() {
    const char* data = "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, buffer.data.timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, buffer.data.pressure);
}

//------------------------------------------------------------------------------
// Test: Multiple consecutive parses
//------------------------------------------------------------------------------
void test_multiple_consecutive_parses() {
    const char* data1 = "1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1000.0,20.0,40.0,-120.0,100.0,1,8,90.0";
    const char* data2 = "2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,1000.0,20.0,40.0,-120.0,200.0,1,8,90.0";
    const char* data3 = "3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,1000.0,20.0,40.0,-120.0,300.0,1,8,90.0";
    double timestamp;

    HITLParser::parse(data1, timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.0, timestamp);

    HITLParser::parse(data2, timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.0, timestamp);

    HITLParser::parse(data3, timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.0, timestamp);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 300.0, buffer.data.gps_alt);
}

//------------------------------------------------------------------------------
// Main test runner
//------------------------------------------------------------------------------
void runAllTests() {
    RUN_TEST(test_parse_valid_data_without_prefix);
    RUN_TEST(test_parse_without_timestamp_extraction);
    RUN_TEST(test_parse_null_data);
    RUN_TEST(test_parse_incomplete_data);
    RUN_TEST(test_parse_malformed_data);
    RUN_TEST(test_parseAndInject_valid_with_prefix);
    RUN_TEST(test_parseAndInject_without_timestamp);
    RUN_TEST(test_parseAndInject_invalid_prefix);
    RUN_TEST(test_parseAndInject_missing_prefix);
    RUN_TEST(test_parse_extreme_values);
    RUN_TEST(test_parse_negative_values);
    RUN_TEST(test_parse_zero_values);
    RUN_TEST(test_multiple_consecutive_parses);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    runAllTests();
    return UNITY_END();
}
