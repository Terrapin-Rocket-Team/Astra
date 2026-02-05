#include <unity.h>
#include "NativeTestHelper.h"

#include "test_data_reporter/test_data_reporter.inc"
#include "test_logger/test_logger.inc"
#include "test_storage/test_storage.inc"
#include "test_hash/test_hash.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_data_reporter
    test_data_reporter_setUp();
    RUN_TEST(test_data_reporter_test_constructor_with_name);
    RUN_TEST(test_data_reporter_test_constructor_without_name);
    RUN_TEST(test_data_reporter_test_constructor_increments_counter);
    RUN_TEST(test_data_reporter_test_getName);
    RUN_TEST(test_data_reporter_test_setName);
    RUN_TEST(test_data_reporter_test_setName_multiple_times);
    RUN_TEST(test_data_reporter_test_addColumn_single);
    RUN_TEST(test_data_reporter_test_addColumn_multiple);
    RUN_TEST(test_data_reporter_test_getLastPoint);
    RUN_TEST(test_data_reporter_test_removeColumn_first);
    RUN_TEST(test_data_reporter_test_removeColumn_middle);
    RUN_TEST(test_data_reporter_test_removeColumn_last);
    RUN_TEST(test_data_reporter_test_removeColumn_nonexistent);
    RUN_TEST(test_data_reporter_test_removeColumn_empty_list);
    RUN_TEST(test_data_reporter_test_clearColumns);
    RUN_TEST(test_data_reporter_test_insertColumn_at_beginning);
    RUN_TEST(test_data_reporter_test_insertColumn_at_end);
    RUN_TEST(test_data_reporter_test_insertColumn_in_middle);
    RUN_TEST(test_data_reporter_test_emit_float);
    RUN_TEST(test_data_reporter_test_emit_int);
    RUN_TEST(test_data_reporter_test_emit_double);
    RUN_TEST(test_data_reporter_test_emit_with_different_formats);
    RUN_TEST(test_data_reporter_test_emit_negative_values);
    RUN_TEST(test_data_reporter_test_isInitialized_default);
    RUN_TEST(test_data_reporter_test_begin_sets_initialized);
    RUN_TEST(test_data_reporter_test_begin_error_does_not_set_initialized);
    RUN_TEST(test_data_reporter_test_bool_operator);
    RUN_TEST(test_data_reporter_test_update_called);
    RUN_TEST(test_data_reporter_test_update_with_time_parameter);
    RUN_TEST(test_data_reporter_test_autoUpdate_default);
    RUN_TEST(test_data_reporter_test_setAutoUpdate);
    RUN_TEST(test_data_reporter_test_empty_reporter);
    RUN_TEST(test_data_reporter_test_multiple_reporters_independent);
    RUN_TEST(test_data_reporter_test_column_chain_integrity_after_operations);
    RUN_TEST(test_data_reporter_test_large_number_of_columns);
    RUN_TEST(test_data_reporter_test_destructor_cleanup);
    test_data_reporter_tearDown();

    // Tests from test_logger
    test_logger_setUp();
    RUN_TEST(test_logger_test_header_single_reporter);
    RUN_TEST(test_logger_test_append_line_single_reporter_values_and_commas);
    RUN_TEST(test_logger_test_multi_reporter_header_and_row);
    RUN_TEST(test_logger_test_unhealthy_sink_is_skipped);
    RUN_TEST(test_logger_test_empty_reporter_is_handled);
    RUN_TEST(test_logger_test_global_configure_and_instance);
    RUN_TEST(test_logger_test_printHeaderTo_single_sink);
    RUN_TEST(test_logger_test_printHeaderTo_with_prefix);
    RUN_TEST(test_logger_test_printHeaderTo_without_prefix);
    RUN_TEST(test_logger_test_printHeaderTo_unhealthy_sink);
    RUN_TEST(test_logger_test_printHeaderTo_multi_reporter);
    test_logger_tearDown();

    // Tests from test_storage
    test_storage_setUp();
    RUN_TEST(test_storage_test_file_default_state);
    RUN_TEST(test_storage_test_file_open_close);
    RUN_TEST(test_storage_test_file_write_single_byte);
    RUN_TEST(test_storage_test_file_write_buffer);
    RUN_TEST(test_storage_test_file_write_when_closed);
    RUN_TEST(test_storage_test_file_read_single_byte);
    RUN_TEST(test_storage_test_file_read_bytes);
    RUN_TEST(test_storage_test_file_read_beyond_end);
    RUN_TEST(test_storage_test_file_available);
    RUN_TEST(test_storage_test_file_seek);
    RUN_TEST(test_storage_test_file_seek_beyond_end);
    RUN_TEST(test_storage_test_file_position);
    RUN_TEST(test_storage_test_file_flush);
    RUN_TEST(test_storage_test_file_bool_operator);
    RUN_TEST(test_storage_test_file_write_and_read_roundtrip);
    RUN_TEST(test_storage_test_storage_default_state);
    RUN_TEST(test_storage_test_storage_begin_end);
    RUN_TEST(test_storage_test_storage_begin_failure);
    RUN_TEST(test_storage_test_storage_open_write_new_file);
    RUN_TEST(test_storage_test_storage_open_read_existing_file);
    RUN_TEST(test_storage_test_storage_open_read_nonexistent_file);
    RUN_TEST(test_storage_test_storage_open_write_append);
    RUN_TEST(test_storage_test_storage_open_write_overwrite);
    RUN_TEST(test_storage_test_storage_exists);
    RUN_TEST(test_storage_test_storage_remove);
    RUN_TEST(test_storage_test_storage_remove_nonexistent);
    RUN_TEST(test_storage_test_storage_mkdir);
    RUN_TEST(test_storage_test_storage_rmdir);
    RUN_TEST(test_storage_test_storage_operations_when_not_begun);
    RUN_TEST(test_storage_test_storage_multiple_files);
    RUN_TEST(test_storage_test_storage_reopen_file_multiple_times);
    RUN_TEST(test_storage_test_storage_write_read_binary_data);
    RUN_TEST(test_storage_test_storage_large_file);
    test_storage_tearDown();

    // Tests from test_hash
    test_hash_setUp();
    RUN_TEST(test_hash_test_fnv1a_empty_string);
    RUN_TEST(test_hash_test_fnv1a_single_character);
    RUN_TEST(test_hash_test_fnv1a_short_string);
    RUN_TEST(test_hash_test_fnv1a_longer_string);
    RUN_TEST(test_hash_test_fnv1a_deterministic);
    RUN_TEST(test_hash_test_fnv1a_case_sensitive);
    RUN_TEST(test_hash_test_fnv1a_different_lengths);
    RUN_TEST(test_hash_test_fnv1a_partial_string);
    RUN_TEST(test_hash_test_literal_operator_basic);
    RUN_TEST(test_hash_test_literal_operator_empty_string);
    RUN_TEST(test_hash_test_literal_operator_same_string);
    RUN_TEST(test_hash_test_literal_operator_different_strings);
    RUN_TEST(test_hash_test_literal_operator_long_string);
    RUN_TEST(test_hash_test_literal_operator_numbers);
    RUN_TEST(test_hash_test_literal_operator_special_characters);
    RUN_TEST(test_hash_test_literal_operator_spaces);
    RUN_TEST(test_hash_test_constexpr_evaluation);
    RUN_TEST(test_hash_test_constexpr_literal_operator);
    RUN_TEST(test_hash_test_constexpr_switch_statement);
    RUN_TEST(test_hash_test_different_strings_produce_different_hashes);
    RUN_TEST(test_hash_test_similar_strings_produce_different_hashes);
    RUN_TEST(test_hash_test_reversed_strings_produce_different_hashes);
    RUN_TEST(test_hash_test_anagrams_produce_different_hashes);
    RUN_TEST(test_hash_test_command_routing);
    RUN_TEST(test_hash_test_state_machine_states);
    RUN_TEST(test_hash_test_sensor_type_identification);
    RUN_TEST(test_hash_test_message_type_discrimination);
    RUN_TEST(test_hash_test_null_character_in_string);
    RUN_TEST(test_hash_test_single_byte_differences);
    RUN_TEST(test_hash_test_unicode_or_extended_ascii);
    RUN_TEST(test_hash_test_very_long_string);
    RUN_TEST(test_hash_test_all_same_character);
    RUN_TEST(test_hash_test_binary_data);
    RUN_TEST(test_hash_test_hash_consistency_across_calls);
    RUN_TEST(test_hash_test_literal_operator_consistency);
    RUN_TEST(test_hash_test_mixed_usage_consistency);
    RUN_TEST(test_hash_test_known_fnv1a_values);
    RUN_TEST(test_hash_test_specific_command_hashes);
    RUN_TEST(test_hash_test_hash_distribution_simple);
    RUN_TEST(test_hash_test_avalanche_effect);
    RUN_TEST(test_hash_test_switch_case_message_routing);
    RUN_TEST(test_hash_test_prefix_matching);
    RUN_TEST(test_hash_test_enum_class_with_hashes);
    test_hash_tearDown();

    UNITY_END();
    return 0;
}
