#include <unity.h>
#include "NativeTestHelper.h"

#include "test_circular_buffer/test_circular_buffer.inc"
#include "test_blinkbuzz/test_blinkbuzz.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_circular_buffer
    test_circular_buffer_setUp();
    RUN_TEST(test_circular_buffer_test_constructor_creates_empty_buffer);
    RUN_TEST(test_circular_buffer_test_constructor_different_sizes);
    RUN_TEST(test_circular_buffer_test_copy_constructor_empty_buffer);
    RUN_TEST(test_circular_buffer_test_copy_constructor_populated_buffer);
    RUN_TEST(test_circular_buffer_test_copy_constructor_full_buffer);
    RUN_TEST(test_circular_buffer_test_copy_constructor_deep_copy);
    RUN_TEST(test_circular_buffer_test_copy_assignment_operator);
    RUN_TEST(test_circular_buffer_test_copy_assignment_self_assignment);
    RUN_TEST(test_circular_buffer_test_copy_assignment_different_sizes);
    RUN_TEST(test_circular_buffer_test_push_single_element);
    RUN_TEST(test_circular_buffer_test_push_multiple_elements);
    RUN_TEST(test_circular_buffer_test_push_to_full_buffer_overwrites_oldest);
    RUN_TEST(test_circular_buffer_test_push_wrap_around);
    RUN_TEST(test_circular_buffer_test_push_different_types);
    RUN_TEST(test_circular_buffer_test_pop_single_element);
    RUN_TEST(test_circular_buffer_test_pop_multiple_elements_fifo);
    RUN_TEST(test_circular_buffer_test_pop_from_empty_buffer_returns_default);
    RUN_TEST(test_circular_buffer_test_pop_reduces_count);
    RUN_TEST(test_circular_buffer_test_pop_after_wraparound);
    RUN_TEST(test_circular_buffer_test_peek_does_not_remove_element);
    RUN_TEST(test_circular_buffer_test_peek_empty_buffer_returns_default);
    RUN_TEST(test_circular_buffer_test_peek_returns_oldest_element);
    RUN_TEST(test_circular_buffer_test_clear_empty_buffer);
    RUN_TEST(test_circular_buffer_test_clear_populated_buffer);
    RUN_TEST(test_circular_buffer_test_clear_resets_indices);
    RUN_TEST(test_circular_buffer_test_isEmpty_on_new_buffer);
    RUN_TEST(test_circular_buffer_test_isEmpty_after_push);
    RUN_TEST(test_circular_buffer_test_isEmpty_after_clear);
    RUN_TEST(test_circular_buffer_test_isFull_reaches_capacity);
    RUN_TEST(test_circular_buffer_test_isFull_after_overwrite);
    RUN_TEST(test_circular_buffer_test_isFull_after_pop);
    RUN_TEST(test_circular_buffer_test_indexing_operator_read);
    RUN_TEST(test_circular_buffer_test_indexing_operator_write);
    RUN_TEST(test_circular_buffer_test_indexing_operator_after_wraparound);
    RUN_TEST(test_circular_buffer_test_indexing_operator_const);
    RUN_TEST(test_circular_buffer_test_indexing_iterate_all_elements);
    RUN_TEST(test_circular_buffer_test_getCount_empty);
    RUN_TEST(test_circular_buffer_test_getCount_after_pushes);
    RUN_TEST(test_circular_buffer_test_getCount_after_pops);
    RUN_TEST(test_circular_buffer_test_getCount_max_is_size);
    RUN_TEST(test_circular_buffer_test_getSize_returns_capacity);
    RUN_TEST(test_circular_buffer_test_getSize_unchanged_by_operations);
    RUN_TEST(test_circular_buffer_test_single_element_buffer);
    RUN_TEST(test_circular_buffer_test_alternating_push_pop);
    RUN_TEST(test_circular_buffer_test_fill_empty_fill_pattern);
    RUN_TEST(test_circular_buffer_test_many_wraparounds);
    RUN_TEST(test_circular_buffer_test_partial_fill_operations);
    RUN_TEST(test_circular_buffer_test_double_precision);
    RUN_TEST(test_circular_buffer_test_negative_values);
    RUN_TEST(test_circular_buffer_test_zero_values);
    RUN_TEST(test_circular_buffer_test_large_number_of_operations);
    RUN_TEST(test_circular_buffer_test_rapid_fill_and_drain);
    RUN_TEST(test_circular_buffer_test_use_as_sliding_window);
    RUN_TEST(test_circular_buffer_test_use_as_queue);
    test_circular_buffer_tearDown();

    // Tests from test_blinkbuzz
    test_blinkbuzz_setUp();
    RUN_TEST(test_blinkbuzz_test_constructor);
    RUN_TEST(test_blinkbuzz_test_init_basic);
    RUN_TEST(test_blinkbuzz_test_init_with_async);
    RUN_TEST(test_blinkbuzz_test_on_allowed_pin);
    RUN_TEST(test_blinkbuzz_test_off_allowed_pin);
    RUN_TEST(test_blinkbuzz_test_on_unallowed_pin);
    RUN_TEST(test_blinkbuzz_test_toggle_pin);
    RUN_TEST(test_blinkbuzz_test_isOn_multiple_pins);
    RUN_TEST(test_blinkbuzz_test_multiple_pins_independent);
    RUN_TEST(test_blinkbuzz_test_clearQueue_basic);
    RUN_TEST(test_blinkbuzz_test_clearQueue_with_reset_to_low);
    RUN_TEST(test_blinkbuzz_test_clearQueue_with_reset_to_high);
    RUN_TEST(test_blinkbuzz_test_aonoff_single_duration);
    RUN_TEST(test_blinkbuzz_test_aonoff_multiple_times);
    RUN_TEST(test_blinkbuzz_test_aonoff_with_pattern);
    RUN_TEST(test_blinkbuzz_test_async_enabled_check);
    RUN_TEST(test_blinkbuzz_test_async_queue_operations);
    RUN_TEST(test_blinkbuzz_test_update_without_async);
    RUN_TEST(test_blinkbuzz_test_bbpattern_constructor);
    RUN_TEST(test_blinkbuzz_test_bbpattern_add_duration);
    RUN_TEST(test_blinkbuzz_test_bbpattern_copy_constructor);
    RUN_TEST(test_blinkbuzz_test_bbpattern_assignment_operator);
    RUN_TEST(test_blinkbuzz_test_pattern_append);
    RUN_TEST(test_blinkbuzz_test_pattern_rest);
    RUN_TEST(test_blinkbuzz_test_operations_before_init);
    RUN_TEST(test_blinkbuzz_test_destructor_cleanup);
    RUN_TEST(test_blinkbuzz_test_negative_pin_handling);
    test_blinkbuzz_tearDown();

    UNITY_END();
    return 0;
}
