#include <unity.h>
#include <cstring>
#include "Testing/HITLParser.h"
#include "Sensors/HITL/HITLSensorBuffer.h"

using namespace astra;

void test_hitl_parser_setUp(void) {
    // Reset HITL buffer before each test
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    buffer.dataReady = false;
    memset(&buffer.data, 0, sizeof(buffer.data));
}

void test_hitl_parser_tearDown(void) {
}

// ========================= Basic Parse Tests =========================

void test_hitl_parser_test_parse_valid_data_without_prefix() {
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

void test_hitl_parser_test_parse_without_timestamp_extraction() {
    const char* data = "2.5,0.0,0.0,9.8,0.0,0.0,0.0,15.0,5.0,-40.0,1000.0,20.0,40.0,-120.0,50.0,1,6,45.0";

    bool result = HITLParser::parse(data);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_TRUE(buffer.dataReady);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.5, buffer.data.timestamp);
}

// ========================= Error Handling Tests =========================

void test_hitl_parser_test_parse_null_data() {
    double timestamp;

    bool result = HITLParser::parse(nullptr, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_null_data_no_timestamp() {
    bool result = HITLParser::parse(nullptr);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_incomplete_data() {
    const char* data = "1.0,2.0,3.0,4.0";  // Only 4 fields instead of 18
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_incomplete_data_17_fields() {
    // Missing the last field (heading)
    const char* data = "1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_malformed_data() {
    const char* data = "not,valid,numbers,at,all";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_partially_malformed_data() {
    // Valid numbers for first few, then invalid
    const char* data = "1.0,2.0,3.0,bad,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_empty_string() {
    const char* data = "";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_whitespace_only() {
    const char* data = "   ";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parse_extra_fields() {
    // 19 fields instead of 18 - should still parse first 18
    const char* data = "1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0,extra";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    // Should succeed - sscanf only reads what it needs
    TEST_ASSERT_TRUE(result);
}

// ========================= ParseAndInject Tests (Legacy API) =========================

void test_hitl_parser_test_parseAndInject_valid_with_prefix() {
    const char* line = "HITL/3.14,1.0,2.0,3.0,0.1,0.2,0.3,10.0,20.0,30.0,1015.0,22.0,45.0,-123.0,200.0,1,10,180.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.14, timestamp);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_TRUE(buffer.dataReady);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.14, buffer.data.timestamp);
}

void test_hitl_parser_test_parseAndInject_without_timestamp() {
    const char* line = "HITL/5.0,0.0,0.0,9.8,0.0,0.0,0.0,15.0,5.0,-40.0,1010.0,18.0,38.0,-122.0,150.0,1,7,270.0";

    bool result = HITLParser::parseAndInject(line);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 5.0, buffer.data.timestamp);
}

void test_hitl_parser_test_parseAndInject_invalid_prefix() {
    const char* line = "INVALID/1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parseAndInject_missing_prefix() {
    const char* line = "1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parseAndInject_case_sensitive_prefix() {
    const char* line = "hitl/1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    // Should fail - prefix is case sensitive
    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parseAndInject_partial_prefix() {
    const char* line = "HITL1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,1,8,90.0";
    double timestamp;

    bool result = HITLParser::parseAndInject(line, timestamp);

    // Missing slash - should fail
    TEST_ASSERT_FALSE(result);
}

void test_hitl_parser_test_parseAndInject_null_line() {
    double timestamp;

    bool result = HITLParser::parseAndInject(nullptr, timestamp);

    // Should fail gracefully - strncmp will handle null
    TEST_ASSERT_FALSE(result);
}

// ========================= Value Range Tests =========================

void test_hitl_parser_test_parse_extreme_values() {
    const char* data = "9999.99,100.0,200.0,300.0,50.0,60.0,70.0,1000.0,2000.0,3000.0,2000.0,100.0,90.0,180.0,10000.0,1,12,359.9";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 9999.99, timestamp);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 100.0, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 10000.0, buffer.data.gps_alt);
}

void test_hitl_parser_test_parse_negative_values() {
    const char* data = "0.5,-9.8,-1.0,-2.0,-0.5,-0.6,-0.7,-15.0,-20.0,-50.0,950.0,-10.0,-45.0,-122.0,0.0,0,0,0.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, -9.8, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, -10.0, buffer.data.temperature);
    TEST_ASSERT_EQUAL(0, buffer.data.gps_fix);
}

void test_hitl_parser_test_parse_zero_values() {
    const char* data = "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, buffer.data.timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, buffer.data.pressure);
}

void test_hitl_parser_test_parse_very_small_values() {
    const char* data = "0.001,0.0001,0.0002,0.0003,0.00001,0.00002,0.00003,0.1,0.2,0.3,1000.0,20.0,40.0,-120.0,100.0,1,8,0.001";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.001, buffer.data.timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.00001, 0.0001, buffer.data.accel.x());
}

void test_hitl_parser_test_parse_scientific_notation() {
    // Test if parser handles scientific notation (1e-3, 2e2, etc)
    const char* data = "1.0,1e-3,2e-3,3e-3,1e-4,2e-4,3e-4,1e1,2e1,3e1,1e3,2e1,4e1,-1.2e2,1e2,1,8,9e1";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.001, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, buffer.data.mag.x());
}

void test_hitl_parser_test_parse_high_precision_gps() {
    // GPS coordinates with high precision
    const char* data = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,37.7749295,-122.4194155,100.123456,1,10,90.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.0000001, 37.7749295, buffer.data.gps_lat);
    TEST_ASSERT_FLOAT_WITHIN(0.0000001, -122.4194155, buffer.data.gps_lon);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, 100.123456, buffer.data.gps_alt);
}

void test_hitl_parser_test_parse_max_gps_satellites() {
    // Maximum GPS satellite count
    const char* data = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,24,90.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_EQUAL(24, buffer.data.gps_fix_quality);
}

void test_hitl_parser_test_parse_no_gps_fix() {
    const char* data = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,0.0,0.0,0.0,0,0,0.0";
    double timestamp;

    bool result = HITLParser::parse(data, timestamp);

    TEST_ASSERT_TRUE(result);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_EQUAL(0, buffer.data.gps_fix);
    TEST_ASSERT_EQUAL(0, buffer.data.gps_fix_quality);
}

void test_hitl_parser_test_parse_extreme_temperature() {
    // Very hot and very cold temperatures
    const char* data1 = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,125.0,40.0,-120.0,100.0,1,8,90.0";
    const char* data2 = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,-60.0,40.0,-120.0,100.0,1,8,90.0";
    double timestamp;

    bool result1 = HITLParser::parse(data1, timestamp);
    TEST_ASSERT_TRUE(result1);
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.1, 125.0, buffer.data.temperature);

    bool result2 = HITLParser::parse(data2, timestamp);
    TEST_ASSERT_TRUE(result2);
    TEST_ASSERT_FLOAT_WITHIN(0.1, -60.0, buffer.data.temperature);
}

void test_hitl_parser_test_parse_extreme_pressure() {
    // Very high altitude (low pressure) and very low altitude (high pressure)
    const char* data1 = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,226.32,25.0,40.0,-120.0,10000.0,1,8,90.0"; // ~10km altitude
    const char* data2 = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1084.63,25.0,40.0,-120.0,-400.0,1,8,90.0"; // Dead Sea level
    double timestamp;

    bool result1 = HITLParser::parse(data1, timestamp);
    TEST_ASSERT_TRUE(result1);
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.1, 226.32, buffer.data.pressure);

    bool result2 = HITLParser::parse(data2, timestamp);
    TEST_ASSERT_TRUE(result2);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 1084.63, buffer.data.pressure);
}

// ========================= Multiple Parse Tests =========================

void test_hitl_parser_test_multiple_consecutive_parses() {
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

void test_hitl_parser_test_parse_overwrites_previous_data() {
    const char* data1 = "1.0,1.0,2.0,3.0,0.1,0.2,0.3,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    const char* data2 = "2.0,5.0,6.0,7.0,0.4,0.5,0.6,15.0,25.0,35.0,1015.0,26.0,41.0,-121.0,150.0,1,10,95.0";
    double timestamp;

    HITLParser::parse(data1, timestamp);
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.0, buffer.data.accel.x());

    HITLParser::parse(data2, timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 5.0, buffer.data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 6.0, buffer.data.accel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 7.0, buffer.data.accel.z());
}

void test_hitl_parser_test_parse_after_failed_parse() {
    const char* bad_data = "invalid";
    const char* good_data = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    double timestamp;

    bool result1 = HITLParser::parse(bad_data, timestamp);
    TEST_ASSERT_FALSE(result1);

    bool result2 = HITLParser::parse(good_data, timestamp);
    TEST_ASSERT_TRUE(result2);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.0, timestamp);
}

// ========================= Buffer State Tests =========================

void test_hitl_parser_test_parse_sets_data_ready_flag() {
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FALSE(buffer.dataReady);

    const char* data = "1.0,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    HITLParser::parse(data);

    TEST_ASSERT_TRUE(buffer.dataReady);
}

void test_hitl_parser_test_parse_updates_buffer_timestamp() {
    const char* data = "123.456,0.0,0.0,9.8,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    HITLParser::parse(data);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 123.456, buffer.data.timestamp);
}

void test_hitl_parser_test_failed_parse_does_not_set_data_ready() {
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    buffer.dataReady = false;

    const char* bad_data = "invalid";
    HITLParser::parse(bad_data);

    // Buffer should not be marked ready on failed parse
    TEST_ASSERT_FALSE(buffer.dataReady);
}

// ========================= Realistic Flight Scenario Tests =========================

void test_hitl_parser_test_parse_takeoff_sequence() {
    // Simulate a takeoff sequence with increasing altitude
    const char* ground = "0.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,37.7749,-122.4194,0.0,1,8,0.0";
    const char* climbing = "1.0,0.0,0.0,15.0,0.1,0.0,0.0,10.0,20.0,30.0,1000.0,24.0,37.7750,-122.4194,50.0,1,8,45.0";
    const char* altitude = "2.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,950.0,20.0,37.7751,-122.4194,500.0,1,10,45.0";

    double timestamp;
    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();

    HITLParser::parse(ground, timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, buffer.data.gps_alt);

    HITLParser::parse(climbing, timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 50.0, buffer.data.gps_alt);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 15.0, buffer.data.accel.z()); // High G during climb

    HITLParser::parse(altitude, timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 500.0, buffer.data.gps_alt);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 9.81, buffer.data.accel.z()); // Back to 1G
}

void test_hitl_parser_test_parse_rotation_data() {
    // Simulate rotation with gyro data
    const char* data = "1.0,0.0,0.0,9.81,1.57,0.78,0.52,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    HITLParser::parse(data);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.57, buffer.data.gyro.x()); // ~90 deg/s
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.78, buffer.data.gyro.y()); // ~45 deg/s
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.52, buffer.data.gyro.z()); // ~30 deg/s
}

void test_hitl_parser_test_parse_high_g_maneuver() {
    // Simulate high-G maneuver
    const char* data = "1.0,5.0,10.0,50.0,0.5,1.0,0.2,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    HITLParser::parse(data);

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
    TEST_ASSERT_FLOAT_WITHIN(0.1, 50.0, buffer.data.accel.z()); // ~5G vertical
}

void test_hitl_parser_test_parse_compass_navigation() {
    // Test all cardinal directions
    const char* north = "1.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,0.0";
    const char* east = "2.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    const char* south = "3.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,180.0";
    const char* west = "4.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,8,270.0";

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();

    HITLParser::parse(north);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, buffer.data.gps_heading);

    HITLParser::parse(east);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 90.0, buffer.data.gps_heading);

    HITLParser::parse(south);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 180.0, buffer.data.gps_heading);

    HITLParser::parse(west);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 270.0, buffer.data.gps_heading);
}

void test_hitl_parser_test_parse_gps_fix_loss() {
    // Simulate losing GPS fix
    const char* with_fix = "1.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,40.0,-120.0,100.0,1,10,90.0";
    const char* without_fix = "2.0,0.0,0.0,9.81,0.0,0.0,0.0,10.0,20.0,30.0,1013.25,25.0,0.0,0.0,0.0,0,0,0.0";

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();

    HITLParser::parse(with_fix);
    TEST_ASSERT_EQUAL(1, buffer.data.gps_fix);
    TEST_ASSERT_EQUAL(10, buffer.data.gps_fix_quality);

    HITLParser::parse(without_fix);
    TEST_ASSERT_EQUAL(0, buffer.data.gps_fix);
    TEST_ASSERT_EQUAL(0, buffer.data.gps_fix_quality);
}

void test_hitl_parser_test_parse_magnetometer_calibration_data() {
    // Simulate magnetometer readings in different orientations
    const char* data1 = "1.0,0.0,0.0,9.81,0.0,0.0,0.0,50.0,0.0,0.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    const char* data2 = "2.0,0.0,0.0,9.81,0.0,0.0,0.0,0.0,50.0,0.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";
    const char* data3 = "3.0,0.0,0.0,9.81,0.0,0.0,0.0,0.0,0.0,-50.0,1013.25,25.0,40.0,-120.0,100.0,1,8,90.0";

    HITLSensorBuffer& buffer = HITLSensorBuffer::instance();

    HITLParser::parse(data1);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 50.0, buffer.data.mag.x());

    HITLParser::parse(data2);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 50.0, buffer.data.mag.y());

    HITLParser::parse(data3);
    TEST_ASSERT_FLOAT_WITHIN(0.1, -50.0, buffer.data.mag.z());
}

// ========================= Main =========================

void runAllTests() {
    // Basic parse tests
    RUN_TEST(test_parse_valid_data_without_prefix);
    RUN_TEST(test_parse_without_timestamp_extraction);

    // Error handling tests
    RUN_TEST(test_parse_null_data);
    RUN_TEST(test_parse_null_data_no_timestamp);
    RUN_TEST(test_parse_incomplete_data);
    RUN_TEST(test_parse_incomplete_data_17_fields);
    RUN_TEST(test_parse_malformed_data);
    RUN_TEST(test_parse_partially_malformed_data);
    RUN_TEST(test_parse_empty_string);
    RUN_TEST(test_parse_whitespace_only);
    RUN_TEST(test_parse_extra_fields);

    // ParseAndInject tests (legacy API)
    RUN_TEST(test_parseAndInject_valid_with_prefix);
    RUN_TEST(test_parseAndInject_without_timestamp);
    RUN_TEST(test_parseAndInject_invalid_prefix);
    RUN_TEST(test_parseAndInject_missing_prefix);
    RUN_TEST(test_parseAndInject_case_sensitive_prefix);
    RUN_TEST(test_parseAndInject_partial_prefix);
    RUN_TEST(test_parseAndInject_null_line);

    // Value range tests
    RUN_TEST(test_parse_extreme_values);
    RUN_TEST(test_parse_negative_values);
    RUN_TEST(test_parse_zero_values);
    RUN_TEST(test_parse_very_small_values);
    RUN_TEST(test_parse_scientific_notation);
    RUN_TEST(test_parse_high_precision_gps);
    RUN_TEST(test_parse_max_gps_satellites);
    RUN_TEST(test_parse_no_gps_fix);
    RUN_TEST(test_parse_extreme_temperature);
    RUN_TEST(test_parse_extreme_pressure);

    // Multiple parse tests
    RUN_TEST(test_multiple_consecutive_parses);
    RUN_TEST(test_parse_overwrites_previous_data);
    RUN_TEST(test_parse_after_failed_parse);

    // Buffer state tests
    RUN_TEST(test_parse_sets_data_ready_flag);
    RUN_TEST(test_parse_updates_buffer_timestamp);
    RUN_TEST(test_failed_parse_does_not_set_data_ready);

    // Realistic flight scenario tests
    RUN_TEST(test_parse_takeoff_sequence);
    RUN_TEST(test_parse_rotation_data);
    RUN_TEST(test_parse_high_g_maneuver);
    RUN_TEST(test_parse_compass_navigation);
    RUN_TEST(test_parse_gps_fix_loss);
    RUN_TEST(test_parse_magnetometer_calibration_data);
}
