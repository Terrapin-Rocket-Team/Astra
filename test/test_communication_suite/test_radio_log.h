#pragma once

#include <unity.h>
#include "RecordData/Logging/LoggingBackend/RadioLog.h"

using namespace astra;

namespace test_radio_log {

void local_setUp(void) {
    Serial1.clearBuffer();
    resetMillis();
}

void local_tearDown(void) {
}

void test_radio_log_handshake_success() {
    local_setUp();
    Serial1.simulateInput("RAD/PONG\n");

    RadioLog log(Serial1);
    bool ok = log.begin();

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(log.ok());
    local_tearDown();
}

void test_radio_log_begin_idempotent_after_success() {
    local_setUp();
    Serial1.simulateInput("RAD/PONG\n");

    RadioLog log(Serial1);
    TEST_ASSERT_TRUE(log.begin());
    TEST_ASSERT_TRUE(log.begin());
    TEST_ASSERT_TRUE(log.ok());
    local_tearDown();
}

void run_test_radio_log_tests()
{
    RUN_TEST(test_radio_log_handshake_success);
    RUN_TEST(test_radio_log_begin_idempotent_after_success);
}

} // namespace test_radio_log
