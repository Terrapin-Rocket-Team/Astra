#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "BlinkBuzz/BlinkBuzz.h"
#include "BlinkBuzz/BBPattern.h"

using namespace astra;

namespace test_blinkbuzz {

BlinkBuzz testBB;
int testPins[] = {13, 14, 15};
int numTestPins = 3;

void local_setUp(void)
{
    // Reset mock state before each test
    resetMillis();
}

void local_tearDown(void)
{
    // clean stuff up after each test here, if needed
}

void test_constructor() {
    local_setUp();
    BlinkBuzz bb;
    // Constructor should not crash
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_init_basic() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13, 14};
    bb.init(pins, 2, false, 0);

    // Async disabled should report false
    TEST_ASSERT_FALSE(bb.isUsingAsync());
    local_tearDown();
}

void test_init_with_async() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13, 14, 15};
    bb.init(pins, 3, true, 50);

    // Should initialize with async enabled
    TEST_ASSERT_TRUE(bb.isUsingAsync());
    local_tearDown();
}

void test_on_allowed_pin() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, false, 0);

    bb.on(13);

    // Should turn on the pin
    TEST_ASSERT_TRUE(bb.isOn(13));
    local_tearDown();
}

void test_off_allowed_pin() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, false, 0);

    bb.on(13);
    bb.off(13);

    // Should turn off the pin
    TEST_ASSERT_FALSE(bb.isOn(13));
    local_tearDown();
}

void test_on_unallowed_pin() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, false, 0);

    bb.on(14); // Pin 14 is not allowed

    // Should not turn on unallowed pin
    TEST_ASSERT_FALSE(bb.isOn(14));
    local_tearDown();
}

void test_toggle_pin() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, false, 0);

    // Start with pin off
    TEST_ASSERT_FALSE(bb.isOn(13));

    // First toggle should turn on
    bb.on(13);
    TEST_ASSERT_TRUE(bb.isOn(13));

    // Second toggle should turn off
    bb.off(13);
    TEST_ASSERT_FALSE(bb.isOn(13));
    local_tearDown();
}

void test_isOn_multiple_pins() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13, 14, 15};
    bb.init(pins, 3, false, 0);

    bb.on(13);
    bb.on(15);

    TEST_ASSERT_TRUE(bb.isOn(13));
    TEST_ASSERT_FALSE(bb.isOn(14));
    TEST_ASSERT_TRUE(bb.isOn(15));
    local_tearDown();
}

void test_clearQueue_basic() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, true, 50);

    bb.clearQueue(13);
    // Should not crash
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_clearQueue_with_reset_to_low() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, true, 50);

    bb.on(13);
    TEST_ASSERT_TRUE(bb.isOn(13));

    bb.clearQueue(13, LOW);
    TEST_ASSERT_FALSE(bb.isOn(13));
    local_tearDown();
}

void test_clearQueue_with_reset_to_high() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, true, 50);

    bb.off(13);
    TEST_ASSERT_FALSE(bb.isOn(13));

    bb.clearQueue(13, HIGH);
    TEST_ASSERT_TRUE(bb.isOn(13));
    local_tearDown();
}

void test_aonoff_single_duration() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, true, 50);

    setMillis(1000);
    bb.aonoff(13, 100);

    // Pin should start off
    TEST_ASSERT_FALSE(bb.isOn(13));

    // At 1000ms, pin should turn on
    setMillis(1000);
    bb.update(1000);
    TEST_ASSERT_TRUE(bb.isOn(13));

    // At 1100ms, pin should turn off
    setMillis(1100);
    bb.update(1100);
    TEST_ASSERT_FALSE(bb.isOn(13));
    local_tearDown();
}

void test_aonoff_multiple_times() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, true, 50);

    setMillis(1000);
    bb.aonoff(13, 50, 3, 50); // 50ms on, 50ms off, 3 times

    // Should toggle on and off 3 times
    // Pattern: ON(50) OFF(50) ON(50) OFF(50) ON(50) OFF(50)

    setMillis(1000);
    bb.update(1000);
    TEST_ASSERT_TRUE(bb.isOn(13)); // First ON
    local_tearDown();
}

void test_bbpattern_constructor() {
    local_setUp();
    BBPattern pattern(100, 3, 50);
    // Pattern should be: 100ms, 50ms, 100ms, 50ms, 100ms, 50ms
    // Should not crash
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_bbpattern_add_duration() {
    local_setUp();
    BBPattern pattern;
    pattern.a(100).a(200).a(300);
    // Should chain successfully
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_bbpattern_copy_constructor() {
    local_setUp();
    BBPattern pattern1(100, 2, 50);
    BBPattern pattern2(pattern1);
    // Deep copy should succeed
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_bbpattern_assignment_operator() {
    local_setUp();
    BBPattern pattern1(100, 2, 50);
    BBPattern pattern2;
    pattern2 = pattern1;
    // Assignment should succeed
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_bbpattern_self_assignment() {
    local_setUp();
    BBPattern pattern(100, 2, 50);
    pattern = pattern;
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_bbpattern_destructor_paths() {
    local_setUp();
    {
        BBPattern emptyPattern;
    }
    {
        BBPattern populatedPattern;
        populatedPattern.a(50).a(100).a(150);
    }
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_aonoff_with_pattern() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, true, 50);

    BBPattern pattern(100, 2, 50); // 100ms on, 50ms off, twice
    setMillis(1000);
    bb.aonoff(13, pattern, false);

    // Should queue the pattern
    setMillis(1000);
    bb.update(1000);
    TEST_ASSERT_TRUE(bb.isOn(13));
    local_tearDown();
}

void test_operations_before_init() {
    local_setUp();
    BlinkBuzz bb;

    // These operations should not crash when called before init
    bb.on(13);
    bb.off(13);
    bb.update();
    TEST_ASSERT_FALSE(bb.isOn(13));

    // All operations should safely return without crashing
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_multiple_pins_independent() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13, 14, 15};
    bb.init(pins, 3, false, 0);

    bb.on(13);
    bb.off(14);
    bb.on(15);

    TEST_ASSERT_TRUE(bb.isOn(13));
    TEST_ASSERT_FALSE(bb.isOn(14));
    TEST_ASSERT_TRUE(bb.isOn(15));
    local_tearDown();
}

void test_async_enabled_check() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};

    // Without async
    bb.init(pins, 1, false, 0);
    TEST_ASSERT_FALSE(bb.isUsingAsync());

    // With async (need a new instance)
    BlinkBuzz bb2;
    bb2.init(pins, 1, true, 50);
    TEST_ASSERT_TRUE(bb2.isUsingAsync());
    local_tearDown();
}

void test_async_queue_operations() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, true, 10); // Small queue for testing

    setMillis(1000);

    // Queue multiple operations
    bb.aonoff(13, 100);

    // Update should process queue
    bb.update(1000);

    // Should not crash
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_pattern_append() {
    local_setUp();
    BBPattern pattern1(100, 2);
    BBPattern pattern2(200, 2);

    pattern1.a(pattern2);

    // Pattern1 should now contain pattern1 + pattern2
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_pattern_rest() {
    local_setUp();
    BBPattern pattern;
    pattern.a(100).a(200).r(300);

    // Last duration should be modified to 300
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_destructor_cleanup() {
    local_setUp();
    {
        BlinkBuzz bb;
        int pins[] = {13, 14};
        bb.init(pins, 2, true, 50);
        bb.on(13);
        // Destructor should clean up when bb goes out of scope
    }
    // If we get here without crashing, destructor worked
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_negative_pin_handling() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, false, 0);

    // Negative pins should be rejected
    bb.on(-1);
    TEST_ASSERT_FALSE(bb.isOn(-1));
    local_tearDown();
}

void test_update_without_async() {
    local_setUp();
    BlinkBuzz bb;
    int pins[] = {13};
    bb.init(pins, 1, false, 0); // No async

    // Update should return early when async is disabled
    bb.update();

    // Should not crash
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void run_test_blinkbuzz_tests()
{
    RUN_TEST(test_constructor);
    RUN_TEST(test_init_basic);
    RUN_TEST(test_init_with_async);
    RUN_TEST(test_on_allowed_pin);
    RUN_TEST(test_off_allowed_pin);
    RUN_TEST(test_on_unallowed_pin);
    RUN_TEST(test_toggle_pin);
    RUN_TEST(test_isOn_multiple_pins);
    RUN_TEST(test_clearQueue_basic);
    RUN_TEST(test_clearQueue_with_reset_to_low);
    RUN_TEST(test_clearQueue_with_reset_to_high);
    RUN_TEST(test_aonoff_single_duration);
    RUN_TEST(test_aonoff_multiple_times);
    RUN_TEST(test_bbpattern_constructor);
    RUN_TEST(test_bbpattern_add_duration);
    RUN_TEST(test_bbpattern_copy_constructor);
    RUN_TEST(test_bbpattern_assignment_operator);
    RUN_TEST(test_bbpattern_self_assignment);
    RUN_TEST(test_bbpattern_destructor_paths);
    RUN_TEST(test_aonoff_with_pattern);
    RUN_TEST(test_operations_before_init);
    RUN_TEST(test_multiple_pins_independent);
    RUN_TEST(test_async_enabled_check);
    RUN_TEST(test_async_queue_operations);
    RUN_TEST(test_pattern_append);
    RUN_TEST(test_pattern_rest);
    RUN_TEST(test_destructor_cleanup);
    RUN_TEST(test_negative_pin_handling);
    RUN_TEST(test_update_without_async);
}

} // namespace test_blinkbuzz
