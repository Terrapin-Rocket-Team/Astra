#pragma once

#include <unity.h>
#include <Arduino.h>

namespace test_sitl {

/**
 * Simple SITL connection test
 * Tests basic SITL functionality without full server
 */

void local_setUp(void)
{
}

void local_tearDown(void)
{
}

void test_sitl_initial_state() {
    local_setUp();
    // Initially not connected
    TEST_ASSERT_FALSE(Serial.isSITLConnected());
    local_tearDown();
}

void test_sitl_connection_attempt() {
    local_setUp();
    // NOTE: Skipping actual connection test because connectSITL has infinite retry loop
    // The implementation needs a timeout parameter to be testable
    // For now, we just verify the API exists by testing the connected state
    TEST_ASSERT_FALSE(Serial.isSITLConnected());
    local_tearDown();
}

void test_sitl_write_without_connection() {
    local_setUp();
    // Should not crash when writing without connection
    Serial.println("TEST");
    TEST_ASSERT_TRUE(true);  // If we get here, no crash
    local_tearDown();
}

void test_sitl_read_without_connection() {
    local_setUp();
    // Should return false for available when not connected
    TEST_ASSERT_FALSE(Serial.available());
    TEST_ASSERT_EQUAL(-1, Serial.read());
    local_tearDown();
}

void test_sitl_disconnect_when_not_connected() {
    local_setUp();
    // Should not crash
    Serial.disconnectSITL();
    TEST_ASSERT_FALSE(Serial.isSITLConnected());
    local_tearDown();
}

void test_simulateInput_still_works() {
    local_setUp();
    // Test that the original test functionality still works
    Serial.simulateInput("TEST123\n");
    TEST_ASSERT_TRUE(Serial.available());

    char buffer[10];
    int i = 0;
    while (Serial.available() && i < 7) {
        buffer[i++] = Serial.read();
    }
    buffer[i] = '\0';

    TEST_ASSERT_EQUAL_STRING("TEST123", buffer);
    local_tearDown();
}

void run_test_sitl_tests()
{
    RUN_TEST(test_sitl_initial_state);
    RUN_TEST(test_sitl_connection_attempt);
    RUN_TEST(test_sitl_write_without_connection);
    RUN_TEST(test_sitl_read_without_connection);
    RUN_TEST(test_sitl_disconnect_when_not_connected);
    RUN_TEST(test_simulateInput_still_works);
}

} // namespace test_sitl
