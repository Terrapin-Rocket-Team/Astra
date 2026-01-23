/**
 * Simple SITL connection test
 * Tests basic SITL functionality without full server
 */

#include <Arduino.h>
#include <unity.h>

void setUp() {
    Serial.clearBuffer();
}

void tearDown() {
    Serial.disconnectSITL();
}

void test_sitl_initial_state() {
    // Initially not connected
    TEST_ASSERT_FALSE(Serial.isSITLConnected());
}

void test_sitl_connection_attempt() {
    // This will fail since no server is running, but tests the API
    bool connected = Serial.connectSITL("localhost", 9999);
    TEST_ASSERT_FALSE(connected);  // Should fail - no server
    TEST_ASSERT_FALSE(Serial.isSITLConnected());
}

void test_sitl_write_without_connection() {
    // Should not crash when writing without connection
    Serial.println("TEST");
    TEST_ASSERT_TRUE(true);  // If we get here, no crash
}

void test_sitl_read_without_connection() {
    // Should return false for available when not connected
    TEST_ASSERT_FALSE(Serial.available());
    TEST_ASSERT_EQUAL(-1, Serial.read());
}

void test_sitl_disconnect_when_not_connected() {
    // Should not crash
    Serial.disconnectSITL();
    TEST_ASSERT_FALSE(Serial.isSITLConnected());
}

void test_simulateInput_still_works() {
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
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_sitl_initial_state);
    RUN_TEST(test_sitl_connection_attempt);
    RUN_TEST(test_sitl_write_without_connection);
    RUN_TEST(test_sitl_read_without_connection);
    RUN_TEST(test_sitl_disconnect_when_not_connected);
    RUN_TEST(test_simulateInput_still_works);

    return UNITY_END();
}
