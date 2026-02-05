#include <unity.h>
#include <cstring>
#include "Communication/SerialMessageRouter.h"

using namespace astra;

// Test data tracking
struct CallbackData {
    int callCount;
    char lastMessage[256];
    char lastPrefix[32];
    Stream* lastSource;

    void reset() {
        callCount = 0;
        lastMessage[0] = '\0';
        lastPrefix[0] = '\0';
        lastSource = nullptr;
    }
};

// Test callback functions
CallbackData hitlData;
CallbackData radioData;
CallbackData cmdData;
CallbackData defaultData;

void handleHITL(const char* message, const char* prefix, Stream* source) {
    hitlData.callCount++;
    strncpy(hitlData.lastMessage, message, sizeof(hitlData.lastMessage) - 1);
    strncpy(hitlData.lastPrefix, prefix, sizeof(hitlData.lastPrefix) - 1);
    hitlData.lastSource = source;
}

void handleRadio(const char* message, const char* prefix, Stream* source) {
    radioData.callCount++;
    strncpy(radioData.lastMessage, message, sizeof(radioData.lastMessage) - 1);
    strncpy(radioData.lastPrefix, prefix, sizeof(radioData.lastPrefix) - 1);
    radioData.lastSource = source;
}

void handleCommand(const char* message, const char* prefix, Stream* source) {
    cmdData.callCount++;
    strncpy(cmdData.lastMessage, message, sizeof(cmdData.lastMessage) - 1);
    strncpy(cmdData.lastPrefix, prefix, sizeof(cmdData.lastPrefix) - 1);
    cmdData.lastSource = source;
}

void handleDefault(const char* message, const char* prefix, Stream* source) {
    defaultData.callCount++;
    strncpy(defaultData.lastMessage, message, sizeof(defaultData.lastMessage) - 1);
    strncpy(defaultData.lastPrefix, prefix, sizeof(defaultData.lastPrefix) - 1);
    defaultData.lastSource = source;
}

// Mock serial instances
Stream serialUSB;
Stream serial8;

void test_serial_router_setUp(void) {
    // Clear mock serial buffers
    serialUSB.clearBuffer();
    serial8.clearBuffer();

    // Reset test data
    hitlData.reset();
    radioData.reset();
    cmdData.reset();
    defaultData.reset();
}

void test_serial_router_tearDown(void) {
}

//------------------------------------------------------------------------------
// Test: Basic router construction
//------------------------------------------------------------------------------
void test_serial_router_test_router_construction() {
    SerialMessageRouter router;

    TEST_ASSERT_EQUAL(0, router.getInterfaceCount());
    TEST_ASSERT_EQUAL(0, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Register single interface
//------------------------------------------------------------------------------
void test_serial_router_test_register_interface() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB);

    TEST_ASSERT_EQUAL(1, router.getInterfaceCount());
}

//------------------------------------------------------------------------------
// Test: Register multiple interfaces
//------------------------------------------------------------------------------
void test_serial_router_test_register_multiple_interfaces() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withInterface(&serial8);

    TEST_ASSERT_EQUAL(2, router.getInterfaceCount());
}

//------------------------------------------------------------------------------
// Test: Register duplicate interface (should be ignored)
//------------------------------------------------------------------------------
void test_serial_router_test_register_duplicate_interface() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withInterface(&serialUSB);

    TEST_ASSERT_EQUAL(1, router.getInterfaceCount());
}

//------------------------------------------------------------------------------
// Test: Register single listener
//------------------------------------------------------------------------------
void test_serial_router_test_register_listener() {
    SerialMessageRouter router;

    router.withListener("HITL/", handleHITL);

    TEST_ASSERT_EQUAL(1, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Register multiple listeners
//------------------------------------------------------------------------------
void test_serial_router_test_register_multiple_listeners() {
    SerialMessageRouter router;

    router.withListener("HITL/", handleHITL)
          .withListener("RAD/", handleRadio)
          .withListener("CMD/", handleCommand);

    TEST_ASSERT_EQUAL(3, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Simple message routing - single prefix
//------------------------------------------------------------------------------
void test_serial_router_test_simple_message_routing() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL);

    serialUSB.simulateInput("HITL/1.234,0.0,9.81\n");
    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount);
    TEST_ASSERT_EQUAL_STRING("1.234,0.0,9.81", hitlData.lastMessage);
    TEST_ASSERT_EQUAL_STRING("HITL/", hitlData.lastPrefix);
    TEST_ASSERT_EQUAL(&serialUSB, hitlData.lastSource);
}

//------------------------------------------------------------------------------
// Test: Multiple messages on same interface
//------------------------------------------------------------------------------
void test_serial_router_test_multiple_messages_same_interface() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL)
          .withListener("CMD/", handleCommand);

    serialUSB.simulateInput("HITL/1.234\nCMD/REBOOT\n");
    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount);
    TEST_ASSERT_EQUAL_STRING("1.234", hitlData.lastMessage);

    TEST_ASSERT_EQUAL(1, cmdData.callCount);
    TEST_ASSERT_EQUAL_STRING("REBOOT", cmdData.lastMessage);
}

//------------------------------------------------------------------------------
// Test: Same prefix on different interfaces
//------------------------------------------------------------------------------
void test_serial_router_test_same_prefix_different_interfaces() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withInterface(&serial8)
          .withListener("CMD/", handleCommand);

    serialUSB.simulateInput("CMD/PING\n");
    router.update();

    TEST_ASSERT_EQUAL(1, cmdData.callCount);
    TEST_ASSERT_EQUAL_STRING("PING", cmdData.lastMessage);
    TEST_ASSERT_EQUAL(&serialUSB, cmdData.lastSource);

    cmdData.reset();

    serial8.simulateInput("CMD/STATUS\n");
    router.update();

    TEST_ASSERT_EQUAL(1, cmdData.callCount);
    TEST_ASSERT_EQUAL_STRING("STATUS", cmdData.lastMessage);
    TEST_ASSERT_EQUAL(&serial8, cmdData.lastSource);
}

//------------------------------------------------------------------------------
// Test: Different prefixes on different interfaces
//------------------------------------------------------------------------------
void test_serial_router_test_different_prefixes_different_interfaces() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withInterface(&serial8)
          .withListener("HITL/", handleHITL)
          .withListener("RAD/", handleRadio);

    serialUSB.simulateInput("HITL/1.234,5.678\n");
    serial8.simulateInput("RAD/RSSI:-45dBm\n");

    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount);
    TEST_ASSERT_EQUAL_STRING("1.234,5.678", hitlData.lastMessage);
    TEST_ASSERT_EQUAL(&serialUSB, hitlData.lastSource);

    TEST_ASSERT_EQUAL(1, radioData.callCount);
    TEST_ASSERT_EQUAL_STRING("RSSI:-45dBm", radioData.lastMessage);
    TEST_ASSERT_EQUAL(&serial8, radioData.lastSource);
}

//------------------------------------------------------------------------------
// Test: Unmatched message without default handler (should be ignored)
//------------------------------------------------------------------------------
void test_serial_router_test_unmatched_message_no_default() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL);

    serialUSB.simulateInput("UNKNOWN/data\n");
    router.update();

    TEST_ASSERT_EQUAL(0, hitlData.callCount);
}

//------------------------------------------------------------------------------
// Test: Unmatched message with default handler
//------------------------------------------------------------------------------
void test_serial_router_test_unmatched_message_with_default() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL)
          .withDefaultHandler(handleDefault);

    serialUSB.simulateInput("UNKNOWN/data\n");
    router.update();

    TEST_ASSERT_EQUAL(0, hitlData.callCount);
    TEST_ASSERT_EQUAL(1, defaultData.callCount);
    TEST_ASSERT_EQUAL_STRING("UNKNOWN/data", defaultData.lastMessage);
    TEST_ASSERT_EQUAL_STRING("", defaultData.lastPrefix);
}

//------------------------------------------------------------------------------
// Test: Custom delimiter
//------------------------------------------------------------------------------
void test_serial_router_test_custom_delimiter() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL)
          .withDelimiter(';');

    serialUSB.simulateInput("HITL/test;");
    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount);
    TEST_ASSERT_EQUAL_STRING("test", hitlData.lastMessage);
}

//------------------------------------------------------------------------------
// Test: Empty message (just delimiter)
//------------------------------------------------------------------------------
void test_serial_router_test_empty_message() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL);

    serialUSB.simulateInput("\n");
    router.update();

    TEST_ASSERT_EQUAL(0, hitlData.callCount);
}

//------------------------------------------------------------------------------
// Test: Partial message (no delimiter yet)
//------------------------------------------------------------------------------
void test_serial_router_test_partial_message() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL);

    // Send partial message without delimiter
    serialUSB.simulateInput("HITL/partial");
    router.update();

    TEST_ASSERT_EQUAL(0, hitlData.callCount);

    // Now send the rest with delimiter
    serialUSB.simulateInput("_data\n");
    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount);
    TEST_ASSERT_EQUAL_STRING("partial_data", hitlData.lastMessage);
}

//------------------------------------------------------------------------------
// Test: Long message handling
//------------------------------------------------------------------------------
void test_serial_router_test_long_message() {
    SerialMessageRouter router(4, 8, 64); // Small buffer size for testing

    router.withInterface(&serialUSB)
          .withListener("TEST/", handleCommand);

    // Create a message longer than buffer
    char longMsg[200];
    strcpy(longMsg, "TEST/");
    for (int i = 5; i < 150; i++) {
        longMsg[i] = 'A';
    }
    longMsg[150] = '\n';
    longMsg[151] = '\0';

    serialUSB.simulateInput(longMsg);
    router.update();

    // Message should be dropped due to buffer overflow
    TEST_ASSERT_EQUAL(0, cmdData.callCount);
}

//------------------------------------------------------------------------------
// Test: Message with no newline at end doesn't trigger callback
//------------------------------------------------------------------------------
void test_serial_router_test_no_delimiter_no_callback() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL);

    serialUSB.simulateInput("HITL/test");
    router.update();

    TEST_ASSERT_EQUAL(0, hitlData.callCount);
}

//------------------------------------------------------------------------------
// Test: Multiple updates with no new data
//------------------------------------------------------------------------------
void test_serial_router_test_multiple_updates_no_data() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL);

    serialUSB.simulateInput("HITL/test\n");
    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount);

    // Multiple updates with no new data
    router.update();
    router.update();
    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount); // Should still be 1
}

//------------------------------------------------------------------------------
// Test: Prefix priority (first registered prefix wins)
//------------------------------------------------------------------------------
void test_serial_router_test_prefix_priority() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("H", handleHITL)
          .withListener("HITL/", handleRadio); // This won't match if "H" matches first

    serialUSB.simulateInput("HITL/test\n");
    router.update();

    // Should match the first registered prefix "H"
    TEST_ASSERT_EQUAL(1, hitlData.callCount);
    TEST_ASSERT_EQUAL(0, radioData.callCount);
}

//------------------------------------------------------------------------------
// Test: Null pointer safety
//------------------------------------------------------------------------------
void test_serial_router_test_null_pointer_safety() {
    SerialMessageRouter router;

    // These should not crash
    router.withInterface(nullptr);
    router.withListener(nullptr, handleHITL);
    router.withListener("TEST/", nullptr);

    TEST_ASSERT_EQUAL(0, router.getInterfaceCount());
    TEST_ASSERT_EQUAL(0, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Real-world HITL scenario
//------------------------------------------------------------------------------
void test_serial_router_test_hitl_scenario() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("HITL/", handleHITL)
          .withListener("TELEM/", handleRadio);

    // Simulate HITL packet
    const char* hitlPacket = "HITL/1.234,0.0,0.0,9.81,0.0,0.0,0.0,20.0,10.0,-45.0,1013.25,25.0\n";
    serialUSB.simulateInput(hitlPacket);
    router.update();

    TEST_ASSERT_EQUAL(1, hitlData.callCount);
    TEST_ASSERT_EQUAL_STRING("1.234,0.0,0.0,9.81,0.0,0.0,0.0,20.0,10.0,-45.0,1013.25,25.0", hitlData.lastMessage);
}

//------------------------------------------------------------------------------
// Test: Max interfaces limit
//------------------------------------------------------------------------------
void test_serial_router_test_max_interfaces_limit() {
    SerialMessageRouter router(2, 8, 256); // Only 2 interfaces allowed

    Stream s1, s2, s3;

    router.withInterface(&s1)
          .withInterface(&s2)
          .withInterface(&s3); // This should be ignored

    TEST_ASSERT_EQUAL(2, router.getInterfaceCount());
}

//------------------------------------------------------------------------------
// Test: Max prefixes limit
//------------------------------------------------------------------------------
void test_serial_router_test_max_prefixes_limit() {
    SerialMessageRouter router(4, 2, 256); // Only 2 prefixes allowed

    router.withListener("A/", handleHITL)
          .withListener("B/", handleRadio)
          .withListener("C/", handleCommand); // This should be ignored

    TEST_ASSERT_EQUAL(2, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Message with carriage return and newline
//------------------------------------------------------------------------------
void test_serial_router_test_message_with_crlf() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("CMD/", handleCommand);

    // Message with \r before \n (common in serial comms)
    serialUSB.simulateInput("CMD/TEST\r\n");
    router.update();

    // Should match on \n, message will contain \r
    TEST_ASSERT_EQUAL(1, cmdData.callCount);
    TEST_ASSERT_EQUAL_STRING("TEST\r", cmdData.lastMessage);
}

//------------------------------------------------------------------------------
// Test: Consecutive delimiters
//------------------------------------------------------------------------------
void test_serial_router_test_consecutive_delimiters() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("CMD/", handleCommand);

    serialUSB.simulateInput("CMD/TEST\n\n\n");
    router.update();

    // Should process first message, then see empty messages
    TEST_ASSERT_EQUAL(1, cmdData.callCount);
    TEST_ASSERT_EQUAL_STRING("TEST", cmdData.lastMessage);
}

//------------------------------------------------------------------------------
// Test: Message exactly at buffer size
//------------------------------------------------------------------------------
void test_serial_router_test_message_at_buffer_boundary() {
    SerialMessageRouter router(4, 8, 32); // 32 byte buffer

    router.withInterface(&serialUSB)
          .withListener("T/", handleCommand);

    // Create message exactly 30 chars (T/ + 28 chars + \n)
    char msg[35];
    strcpy(msg, "T/");
    for (int i = 2; i < 30; i++) {
        msg[i] = 'A';
    }
    msg[30] = '\n';
    msg[31] = '\0';

    serialUSB.simulateInput(msg);
    router.update();

    // Should work since we have space for 31 chars + null terminator
    TEST_ASSERT_EQUAL(1, cmdData.callCount);
}

//------------------------------------------------------------------------------
// Test: Empty prefix handling
//------------------------------------------------------------------------------
void test_serial_router_test_empty_prefix() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("", handleCommand);

    // Note: The implementation adds empty prefix listeners (no validation)
    // This might be a bug - empty prefixes will never match in matchesPrefix()
    // because it checks prefixLen == 0 and returns false
    TEST_ASSERT_EQUAL(1, router.getListenerCount()); // Currently allows empty prefix

    // Empty prefix won't actually match anything
    serialUSB.simulateInput("test\n");
    router.update();
    TEST_ASSERT_EQUAL(0, cmdData.callCount); // Won't match
}

//------------------------------------------------------------------------------
// Test: Destructor with multiple interfaces
//------------------------------------------------------------------------------
void test_serial_router_test_destructor_cleanup() {
    {
        SerialMessageRouter* router = new SerialMessageRouter();
        router->withInterface(&serialUSB)
               .withInterface(&serial8)
               .withListener("TEST/", handleCommand);

        delete router; // Should clean up all buffers
    }
    // If we get here without crashing, destructor worked
    TEST_ASSERT_TRUE(true);
}

//------------------------------------------------------------------------------
// Test: Interface that becomes unavailable
//------------------------------------------------------------------------------
void test_serial_router_test_interface_no_data_available() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("CMD/", handleCommand);

    // No data available
    router.update();

    TEST_ASSERT_EQUAL(0, cmdData.callCount);
}

//------------------------------------------------------------------------------
// Test: Prefix with special characters
//------------------------------------------------------------------------------
void test_serial_router_test_prefix_with_special_chars() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withListener("@#$/", handleCommand);

    serialUSB.simulateInput("@#$/data\n");
    router.update();

    TEST_ASSERT_EQUAL(1, cmdData.callCount);
    TEST_ASSERT_EQUAL_STRING("data", cmdData.lastMessage);
}

//------------------------------------------------------------------------------
// Main test runner
//------------------------------------------------------------------------------
void runAllTests() {
    // Basic construction and registration tests
    RUN_TEST(test_router_construction);
    RUN_TEST(test_register_interface);
    RUN_TEST(test_register_multiple_interfaces);
    RUN_TEST(test_register_duplicate_interface);
    RUN_TEST(test_register_listener);
    RUN_TEST(test_register_multiple_listeners);

    // Message routing tests
    RUN_TEST(test_simple_message_routing);
    RUN_TEST(test_multiple_messages_same_interface);
    RUN_TEST(test_same_prefix_different_interfaces);
    RUN_TEST(test_different_prefixes_different_interfaces);
    RUN_TEST(test_unmatched_message_no_default);
    RUN_TEST(test_unmatched_message_with_default);

    // Configuration tests
    RUN_TEST(test_custom_delimiter);
    RUN_TEST(test_empty_message);
    RUN_TEST(test_partial_message);
    RUN_TEST(test_long_message);
    RUN_TEST(test_no_delimiter_no_callback);
    RUN_TEST(test_multiple_updates_no_data);
    RUN_TEST(test_prefix_priority);

    // Edge case and safety tests
    RUN_TEST(test_null_pointer_safety);
    RUN_TEST(test_max_interfaces_limit);
    RUN_TEST(test_max_prefixes_limit);
    RUN_TEST(test_message_with_crlf);
    RUN_TEST(test_consecutive_delimiters);
    RUN_TEST(test_message_at_buffer_boundary);
    RUN_TEST(test_empty_prefix);
    RUN_TEST(test_destructor_cleanup);
    RUN_TEST(test_interface_no_data_available);
    RUN_TEST(test_prefix_with_special_chars);

    // Real-world scenario tests
    RUN_TEST(test_hitl_scenario);
}
