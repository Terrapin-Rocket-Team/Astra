#include <unity.h>
#include "NativeTestHelper.h"

#include "test_serial_router/test_serial_router.inc"
#include "test_cmd_handler/test_cmd_handler.inc"
#include "test_hitl_parser/test_hitl_parser.inc"
#include "test_sitl/test_sitl.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_serial_router
    test_serial_router_setUp();
    RUN_TEST(test_serial_router_test_router_construction);
    RUN_TEST(test_serial_router_test_register_interface);
    RUN_TEST(test_serial_router_test_register_multiple_interfaces);
    RUN_TEST(test_serial_router_test_register_duplicate_interface);
    RUN_TEST(test_serial_router_test_register_listener);
    RUN_TEST(test_serial_router_test_register_multiple_listeners);
    RUN_TEST(test_serial_router_test_simple_message_routing);
    RUN_TEST(test_serial_router_test_multiple_messages_same_interface);
    RUN_TEST(test_serial_router_test_same_prefix_different_interfaces);
    RUN_TEST(test_serial_router_test_different_prefixes_different_interfaces);
    RUN_TEST(test_serial_router_test_unmatched_message_no_default);
    RUN_TEST(test_serial_router_test_unmatched_message_with_default);
    RUN_TEST(test_serial_router_test_custom_delimiter);
    RUN_TEST(test_serial_router_test_empty_message);
    RUN_TEST(test_serial_router_test_partial_message);
    RUN_TEST(test_serial_router_test_long_message);
    RUN_TEST(test_serial_router_test_no_delimiter_no_callback);
    RUN_TEST(test_serial_router_test_multiple_updates_no_data);
    RUN_TEST(test_serial_router_test_prefix_priority);
    RUN_TEST(test_serial_router_test_null_pointer_safety);
    RUN_TEST(test_serial_router_test_max_interfaces_limit);
    RUN_TEST(test_serial_router_test_max_prefixes_limit);
    RUN_TEST(test_serial_router_test_message_with_crlf);
    RUN_TEST(test_serial_router_test_consecutive_delimiters);
    RUN_TEST(test_serial_router_test_message_at_buffer_boundary);
    RUN_TEST(test_serial_router_test_empty_prefix);
    RUN_TEST(test_serial_router_test_destructor_cleanup);
    RUN_TEST(test_serial_router_test_interface_no_data_available);
    RUN_TEST(test_serial_router_test_prefix_with_special_chars);
    RUN_TEST(test_serial_router_test_hitl_scenario);
    test_serial_router_tearDown();

    // Tests from test_cmd_handler
    test_cmd_handler_setUp();
    RUN_TEST(test_cmd_handler_test_cmd_header_basic);
    RUN_TEST(test_cmd_handler_test_cmd_header_multiple_requests);
    RUN_TEST(test_cmd_handler_test_cmd_unknown_command);
    RUN_TEST(test_cmd_handler_test_cmd_header_with_multiple_reporters);
    RUN_TEST(test_cmd_handler_test_cmd_header_only_to_requesting_stream);
    test_cmd_handler_tearDown();

    // Tests from test_hitl_parser
    test_hitl_parser_setUp();
    RUN_TEST(test_hitl_parser_test_parse_valid_data_without_prefix);
    RUN_TEST(test_hitl_parser_test_parse_without_timestamp_extraction);
    RUN_TEST(test_hitl_parser_test_parse_null_data);
    RUN_TEST(test_hitl_parser_test_parse_null_data_no_timestamp);
    RUN_TEST(test_hitl_parser_test_parse_incomplete_data);
    RUN_TEST(test_hitl_parser_test_parse_incomplete_data_17_fields);
    RUN_TEST(test_hitl_parser_test_parse_malformed_data);
    RUN_TEST(test_hitl_parser_test_parse_partially_malformed_data);
    RUN_TEST(test_hitl_parser_test_parse_empty_string);
    RUN_TEST(test_hitl_parser_test_parse_whitespace_only);
    RUN_TEST(test_hitl_parser_test_parse_extra_fields);
    RUN_TEST(test_hitl_parser_test_parseAndInject_valid_with_prefix);
    RUN_TEST(test_hitl_parser_test_parseAndInject_without_timestamp);
    RUN_TEST(test_hitl_parser_test_parseAndInject_invalid_prefix);
    RUN_TEST(test_hitl_parser_test_parseAndInject_missing_prefix);
    RUN_TEST(test_hitl_parser_test_parseAndInject_case_sensitive_prefix);
    RUN_TEST(test_hitl_parser_test_parseAndInject_partial_prefix);
    RUN_TEST(test_hitl_parser_test_parseAndInject_null_line);
    RUN_TEST(test_hitl_parser_test_parse_extreme_values);
    RUN_TEST(test_hitl_parser_test_parse_negative_values);
    RUN_TEST(test_hitl_parser_test_parse_zero_values);
    RUN_TEST(test_hitl_parser_test_parse_very_small_values);
    RUN_TEST(test_hitl_parser_test_parse_scientific_notation);
    RUN_TEST(test_hitl_parser_test_parse_high_precision_gps);
    RUN_TEST(test_hitl_parser_test_parse_max_gps_satellites);
    RUN_TEST(test_hitl_parser_test_parse_no_gps_fix);
    RUN_TEST(test_hitl_parser_test_parse_extreme_temperature);
    RUN_TEST(test_hitl_parser_test_parse_extreme_pressure);
    RUN_TEST(test_hitl_parser_test_multiple_consecutive_parses);
    RUN_TEST(test_hitl_parser_test_parse_overwrites_previous_data);
    RUN_TEST(test_hitl_parser_test_parse_after_failed_parse);
    RUN_TEST(test_hitl_parser_test_parse_sets_data_ready_flag);
    RUN_TEST(test_hitl_parser_test_parse_updates_buffer_timestamp);
    RUN_TEST(test_hitl_parser_test_failed_parse_does_not_set_data_ready);
    RUN_TEST(test_hitl_parser_test_parse_takeoff_sequence);
    RUN_TEST(test_hitl_parser_test_parse_rotation_data);
    RUN_TEST(test_hitl_parser_test_parse_high_g_maneuver);
    RUN_TEST(test_hitl_parser_test_parse_compass_navigation);
    RUN_TEST(test_hitl_parser_test_parse_gps_fix_loss);
    RUN_TEST(test_hitl_parser_test_parse_magnetometer_calibration_data);
    test_hitl_parser_tearDown();

    // Tests from test_sitl
    test_sitl_setUp();
    RUN_TEST(test_sitl_test_sitl_initial_state);
    RUN_TEST(test_sitl_test_sitl_connection_attempt);
    RUN_TEST(test_sitl_test_sitl_write_without_connection);
    RUN_TEST(test_sitl_test_sitl_read_without_connection);
    RUN_TEST(test_sitl_test_sitl_disconnect_when_not_connected);
    RUN_TEST(test_sitl_test_simulateInput_still_works);
    test_sitl_tearDown();

    UNITY_END();
    return 0;
}
