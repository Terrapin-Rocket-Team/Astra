#include <unity.h>
#include <cstring>
#include "../../src/Communication/SerialMessageRouter.h"

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

void setUp(void) {
    // Clear mock serial buffers
    serialUSB.clearBuffer();
    serial8.clearBuffer();

    // Reset test data
    hitlData.reset();
    radioData.reset();
    cmdData.reset();
    defaultData.reset();
}

void tearDown(void) {
}

//------------------------------------------------------------------------------
// Test: Basic router construction
//------------------------------------------------------------------------------
void test_router_construction() {
    SerialMessageRouter router;

    TEST_ASSERT_EQUAL(0, router.getInterfaceCount());
    TEST_ASSERT_EQUAL(0, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Register single interface
//------------------------------------------------------------------------------
void test_register_interface() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB);

    TEST_ASSERT_EQUAL(1, router.getInterfaceCount());
}

//------------------------------------------------------------------------------
// Test: Register multiple interfaces
//------------------------------------------------------------------------------
void test_register_multiple_interfaces() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withInterface(&serial8);

    TEST_ASSERT_EQUAL(2, router.getInterfaceCount());
}

//------------------------------------------------------------------------------
// Test: Register duplicate interface (should be ignored)
//------------------------------------------------------------------------------
void test_register_duplicate_interface() {
    SerialMessageRouter router;

    router.withInterface(&serialUSB)
          .withInterface(&serialUSB);

    TEST_ASSERT_EQUAL(1, router.getInterfaceCount());
}

//------------------------------------------------------------------------------
// Test: Register single listener
//------------------------------------------------------------------------------
void test_register_listener() {
    SerialMessageRouter router;

    router.withListener("HITL/", handleHITL);

    TEST_ASSERT_EQUAL(1, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Register multiple listeners
//------------------------------------------------------------------------------
void test_register_multiple_listeners() {
    SerialMessageRouter router;

    router.withListener("HITL/", handleHITL)
          .withListener("RAD/", handleRadio)
          .withListener("CMD/", handleCommand);

    TEST_ASSERT_EQUAL(3, router.getListenerCount());
}

//------------------------------------------------------------------------------
// Test: Simple message routing - single prefix
//------------------------------------------------------------------------------
void test_simple_message_routing() {
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
void test_multiple_messages_same_interface() {
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
void test_same_prefix_different_interfaces() {
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
void test_different_prefixes_different_interfaces() {
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
void test_unmatched_message_no_default() {
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
void test_unmatched_message_with_default() {
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
void test_custom_delimiter() {
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
void test_empty_message() {
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
void test_partial_message() {
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
void test_long_message() {
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
void test_no_delimiter_no_callback() {
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
void test_multiple_updates_no_data() {
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
void test_prefix_priority() {
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
void test_null_pointer_safety() {
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
void test_hitl_scenario() {
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
// Main test runner
//------------------------------------------------------------------------------
void runAllTests() {
    RUN_TEST(test_router_construction);
    RUN_TEST(test_register_interface);
    RUN_TEST(test_register_multiple_interfaces);
    RUN_TEST(test_register_duplicate_interface);
    RUN_TEST(test_register_listener);
    RUN_TEST(test_register_multiple_listeners);
    RUN_TEST(test_simple_message_routing);
    RUN_TEST(test_multiple_messages_same_interface);
    RUN_TEST(test_same_prefix_different_interfaces);
    RUN_TEST(test_different_prefixes_different_interfaces);
    RUN_TEST(test_unmatched_message_no_default);
    RUN_TEST(test_unmatched_message_with_default);
    RUN_TEST(test_custom_delimiter);
    RUN_TEST(test_empty_message);
    RUN_TEST(test_partial_message);
    RUN_TEST(test_long_message);
    RUN_TEST(test_no_delimiter_no_callback);
    RUN_TEST(test_multiple_updates_no_data);
    RUN_TEST(test_prefix_priority);
    RUN_TEST(test_null_pointer_safety);
    RUN_TEST(test_hitl_scenario);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    runAllTests();
    return UNITY_END();
}
