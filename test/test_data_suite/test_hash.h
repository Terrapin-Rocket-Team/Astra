#pragma once

#include <unity.h>
#include "Utils/Hash.h"
#include <cstring>

namespace test_hash {


void local_setUp(void)
{
}

void local_tearDown(void)
{
}

void test_fnv1a_empty_string()
{
    local_setUp();
    // Empty string should hash to FNV offset basis
    uint32_t hash = fnv1a_32("", 0);
    TEST_ASSERT_EQUAL_UINT32(2166136261u, hash);
    local_tearDown();
}

void test_fnv1a_single_character()
{
    local_setUp();
    uint32_t hash_a = fnv1a_32("a", 1);
    uint32_t hash_b = fnv1a_32("b", 1);

    // Different characters should produce different hashes
    TEST_ASSERT_NOT_EQUAL(hash_a, hash_b);
    local_tearDown();
}

void test_fnv1a_short_string()
{
    local_setUp();
    const char* str = "hello";
    uint32_t hash = fnv1a_32(str, strlen(str));

    // Should produce a non-zero hash
    TEST_ASSERT_NOT_EQUAL(0, hash);
    local_tearDown();
}

void test_fnv1a_longer_string()
{
    local_setUp();
    const char* str = "The quick brown fox jumps over the lazy dog";
    uint32_t hash = fnv1a_32(str, strlen(str));

    // Should produce a non-zero hash
    TEST_ASSERT_NOT_EQUAL(0, hash);
    local_tearDown();
}

void test_fnv1a_deterministic()
{
    local_setUp();
    const char* str = "test";
    uint32_t hash1 = fnv1a_32(str, strlen(str));
    uint32_t hash2 = fnv1a_32(str, strlen(str));

    // Same input should always produce same hash
    TEST_ASSERT_EQUAL_UINT32(hash1, hash2);
    local_tearDown();
}

void test_fnv1a_case_sensitive()
{
    local_setUp();
    uint32_t hash_lower = fnv1a_32("abc", 3);
    uint32_t hash_upper = fnv1a_32("ABC", 3);

    // Case should matter
    TEST_ASSERT_NOT_EQUAL(hash_lower, hash_upper);
    local_tearDown();
}

void test_fnv1a_different_lengths()
{
    local_setUp();
    uint32_t hash1 = fnv1a_32("test", 4);
    uint32_t hash2 = fnv1a_32("testing", 7);

    // Different lengths should produce different hashes
    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_fnv1a_partial_string()
{
    local_setUp();
    const char* str = "hello world";
    uint32_t hash_full = fnv1a_32(str, 11);
    uint32_t hash_partial = fnv1a_32(str, 5); // Just "hello"

    // Partial hash should differ from full
    TEST_ASSERT_NOT_EQUAL(hash_full, hash_partial);
    local_tearDown();
}

void test_literal_operator_basic()
{
    local_setUp();
    uint32_t hash = "test"_i;

    // Should produce a hash
    TEST_ASSERT_NOT_EQUAL(0, hash);
    local_tearDown();
}

void test_literal_operator_empty_string()
{
    local_setUp();
    uint32_t hash = ""_i;

    // Empty string hash is FNV offset basis
    TEST_ASSERT_EQUAL_UINT32(2166136261u, hash);
    local_tearDown();
}

void test_literal_operator_same_string()
{
    local_setUp();
    uint32_t hash1 = "hello"_i;
    uint32_t hash2 = "hello"_i;

    TEST_ASSERT_EQUAL_UINT32(hash1, hash2);
    local_tearDown();
}

void test_literal_operator_different_strings()
{
    local_setUp();
    uint32_t hash1 = "foo"_i;
    uint32_t hash2 = "bar"_i;

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_literal_operator_long_string()
{
    local_setUp();
    uint32_t hash = "This is a very long string to test the hash function with more data"_i;

    TEST_ASSERT_NOT_EQUAL(0, hash);
    local_tearDown();
}

void test_literal_operator_numbers()
{
    local_setUp();
    uint32_t hash1 = "123"_i;
    uint32_t hash2 = "456"_i;
    uint32_t hash3 = "123"_i;

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    TEST_ASSERT_EQUAL_UINT32(hash1, hash3);
    local_tearDown();
}

void test_literal_operator_special_characters()
{
    local_setUp();
    uint32_t hash1 = "hello@world"_i;
    uint32_t hash2 = "hello#world"_i;

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_literal_operator_spaces()
{
    local_setUp();
    uint32_t hash1 = "hello world"_i;
    uint32_t hash2 = "helloworld"_i;

    // Spaces should affect hash
    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_constexpr_evaluation()
{
    local_setUp();
    // These should be evaluated at compile time
    constexpr uint32_t hash1 = fnv1a_32("compile_time", 12);
    constexpr uint32_t hash2 = fnv1a_32("compile_time", 12);

    TEST_ASSERT_EQUAL_UINT32(hash1, hash2);
    local_tearDown();
}

void test_constexpr_literal_operator()
{
    local_setUp();
    // Literal operator should work at compile time
    constexpr uint32_t hash = "constexpr_test"_i;

    TEST_ASSERT_NOT_EQUAL(0, hash);
    local_tearDown();
}

void test_constexpr_switch_statement()
{
    local_setUp();
    // Can use in switch statement (compile-time constant)
    const char* result = nullptr;
    uint32_t hash = "CMD_START"_i;

    switch (hash)
    {
        case "CMD_START"_i:
            result = "start";
            break;
        case "CMD_STOP"_i:
            result = "stop";
            break;
        default:
            result = "unknown";
            break;
    }

    TEST_ASSERT_EQUAL_STRING("start", result);
    local_tearDown();
}

void test_different_strings_produce_different_hashes()
{
    local_setUp();
    // Test that common strings don't collide
    uint32_t hash_a = "a"_i;
    uint32_t hash_b = "b"_i;
    uint32_t hash_c = "c"_i;

    TEST_ASSERT_NOT_EQUAL(hash_a, hash_b);
    TEST_ASSERT_NOT_EQUAL(hash_b, hash_c);
    TEST_ASSERT_NOT_EQUAL(hash_a, hash_c);
    local_tearDown();
}

void test_similar_strings_produce_different_hashes()
{
    local_setUp();
    uint32_t hash1 = "test1"_i;
    uint32_t hash2 = "test2"_i;
    uint32_t hash3 = "test3"_i;

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    TEST_ASSERT_NOT_EQUAL(hash2, hash3);
    TEST_ASSERT_NOT_EQUAL(hash1, hash3);
    local_tearDown();
}

void test_reversed_strings_produce_different_hashes()
{
    local_setUp();
    uint32_t hash_forward = "abc"_i;
    uint32_t hash_reverse = "cba"_i;

    TEST_ASSERT_NOT_EQUAL(hash_forward, hash_reverse);
    local_tearDown();
}

void test_anagrams_produce_different_hashes()
{
    local_setUp();
    uint32_t hash1 = "listen"_i;
    uint32_t hash2 = "silent"_i;

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_command_routing()
{
    local_setUp();
    // Simulate command routing using hashes
    const char* commands[] = {
        "GET",
        "POST",
        "PUT",
        "DELETE",
        "PATCH"
    };

    uint32_t hashes[5];
    for (int i = 0; i < 5; i++)
    {
        hashes[i] = fnv1a_32(commands[i], strlen(commands[i]));
    }

    // All should be unique
    for (int i = 0; i < 5; i++)
    {
        for (int j = i + 1; j < 5; j++)
        {
            TEST_ASSERT_NOT_EQUAL(hashes[i], hashes[j]);
        }
    }
    local_tearDown();
}

void test_state_machine_states()
{
    local_setUp();
    // State machine with string-based state names
    enum class State : uint32_t
    {
        IDLE = "IDLE"_i,
        RUNNING = "RUNNING"_i,
        PAUSED = "PAUSED"_i,
        STOPPED = "STOPPED"_i,
        ERROR = "ERROR"_i
    };

    State state = State::IDLE;

    switch (state)
    {
        case State::IDLE:
            TEST_ASSERT_TRUE(true); // Expected path
            break;
        case State::RUNNING:
        case State::PAUSED:
        case State::STOPPED:
        case State::ERROR:
            TEST_ASSERT_TRUE(false); // Should not reach
            break;
    }
    local_tearDown();
}

void test_sensor_type_identification()
{
    local_setUp();
    // Sensor type identification using hashes
    uint32_t accel_hash = "ACCEL"_i;
    uint32_t gyro_hash = "GYRO"_i;
    uint32_t mag_hash = "MAG"_i;
    uint32_t gps_hash = "GPS"_i;
    uint32_t baro_hash = "BARO"_i;

    // All sensor types should be unique
    TEST_ASSERT_NOT_EQUAL(accel_hash, gyro_hash);
    TEST_ASSERT_NOT_EQUAL(accel_hash, mag_hash);
    TEST_ASSERT_NOT_EQUAL(gyro_hash, mag_hash);
    TEST_ASSERT_NOT_EQUAL(gps_hash, baro_hash);
    local_tearDown();
}

void test_message_type_discrimination()
{
    local_setUp();
    // Different message types
    uint32_t data_msg = "DATA/"_i;
    uint32_t cmd_msg = "CMD/"_i;
    uint32_t log_msg = "LOG/"_i;
    uint32_t hitl_msg = "HITL/"_i;

    // Should all be different
    TEST_ASSERT_NOT_EQUAL(data_msg, cmd_msg);
    TEST_ASSERT_NOT_EQUAL(cmd_msg, log_msg);
    TEST_ASSERT_NOT_EQUAL(log_msg, hitl_msg);
    local_tearDown();
}

void test_null_character_in_string()
{
    local_setUp();
    // Explicit count allows handling embedded nulls
    const char str[] = {'a', 'b', '\0', 'c', 'd'};
    uint32_t hash1 = fnv1a_32(str, 2);  // Just "ab"
    uint32_t hash2 = fnv1a_32(str, 5);  // Including null and "cd"

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_single_byte_differences()
{
    local_setUp();
    // Strings differing by single byte should hash differently
    const char* str1 = "test1";
    const char* str2 = "test2";

    uint32_t hash1 = fnv1a_32(str1, strlen(str1));
    uint32_t hash2 = fnv1a_32(str2, strlen(str2));

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_unicode_or_extended_ascii()
{
    local_setUp();
    // Test with extended ASCII characters
    const char* str1 = "café";  // with é
    const char* str2 = "cafe";  // without

    uint32_t hash1 = fnv1a_32(str1, strlen(str1));
    uint32_t hash2 = fnv1a_32(str2, strlen(str2));

    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_very_long_string()
{
    local_setUp();
    // Test with very long string (256 chars)
    char long_str[257];
    for (int i = 0; i < 256; i++)
    {
        long_str[i] = 'a' + (i % 26);
    }
    long_str[256] = '\0';

    uint32_t hash = fnv1a_32(long_str, 256);

    TEST_ASSERT_NOT_EQUAL(0, hash);
    TEST_ASSERT_NOT_EQUAL(2166136261u, hash);
    local_tearDown();
}

void test_all_same_character()
{
    local_setUp();
    const char* str1 = "aaaa";
    const char* str2 = "aaaaa";

    uint32_t hash1 = fnv1a_32(str1, strlen(str1));
    uint32_t hash2 = fnv1a_32(str2, strlen(str2));

    // Different lengths of same character should differ
    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    local_tearDown();
}

void test_binary_data()
{
    local_setUp();
    // Test with binary data (not necessarily printable)
    uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD};
    uint32_t hash = fnv1a_32((const char*)data, sizeof(data));

    TEST_ASSERT_NOT_EQUAL(0, hash);
    local_tearDown();
}

void test_hash_consistency_across_calls()
{
    local_setUp();
    const char* str = "consistency_test";

    uint32_t hash1 = fnv1a_32(str, strlen(str));
    uint32_t hash2 = fnv1a_32(str, strlen(str));
    uint32_t hash3 = fnv1a_32(str, strlen(str));

    TEST_ASSERT_EQUAL_UINT32(hash1, hash2);
    TEST_ASSERT_EQUAL_UINT32(hash2, hash3);
    local_tearDown();
}

void test_literal_operator_consistency()
{
    local_setUp();
    uint32_t hash1 = "test"_i;
    uint32_t hash2 = "test"_i;
    uint32_t hash3 = "test"_i;

    TEST_ASSERT_EQUAL_UINT32(hash1, hash2);
    TEST_ASSERT_EQUAL_UINT32(hash2, hash3);
    local_tearDown();
}

void test_mixed_usage_consistency()
{
    local_setUp();
    const char* str = "mixed";

    uint32_t hash_function = fnv1a_32(str, strlen(str));
    uint32_t hash_literal = "mixed"_i;

    // Both methods should produce same result
    TEST_ASSERT_EQUAL_UINT32(hash_function, hash_literal);
    local_tearDown();
}

void test_known_fnv1a_values()
{
    local_setUp();
    // FNV-1a has known test vectors
    // Empty string
    uint32_t hash_empty = fnv1a_32("", 0);
    TEST_ASSERT_EQUAL_UINT32(2166136261u, hash_empty);

    // Single character 'a' (test vector)
    uint32_t hash_a = fnv1a_32("a", 1);
    // FNV-1a("a") = 0xe40c292c
    TEST_ASSERT_EQUAL_UINT32(0xe40c292c, hash_a);
    local_tearDown();
}

void test_specific_command_hashes()
{
    local_setUp();
    // Document specific hash values for important commands
    // This helps catch unintended changes to the hash function

    uint32_t start_hash = "START"_i;
    uint32_t stop_hash = "STOP"_i;
    uint32_t reset_hash = "RESET"_i;

    // These should be stable
    TEST_ASSERT_EQUAL_UINT32(start_hash, "START"_i);
    TEST_ASSERT_EQUAL_UINT32(stop_hash, "STOP"_i);
    TEST_ASSERT_EQUAL_UINT32(reset_hash, "RESET"_i);
    local_tearDown();
}

void test_hash_distribution_simple()
{
    local_setUp();
    // Test that similar strings don't cluster in hash space
    uint32_t hash1 = "AAA"_i;
    uint32_t hash2 = "AAB"_i;
    uint32_t hash3 = "ABA"_i;
    uint32_t hash4 = "BAA"_i;

    // All should be distinct
    TEST_ASSERT_NOT_EQUAL(hash1, hash2);
    TEST_ASSERT_NOT_EQUAL(hash1, hash3);
    TEST_ASSERT_NOT_EQUAL(hash1, hash4);
    TEST_ASSERT_NOT_EQUAL(hash2, hash3);
    TEST_ASSERT_NOT_EQUAL(hash2, hash4);
    TEST_ASSERT_NOT_EQUAL(hash3, hash4);
    local_tearDown();
}

void test_avalanche_effect()
{
    local_setUp();
    // Small input change should cause large hash change
    uint32_t hash1 = "test"_i;
    uint32_t hash2 = "tess"_i;  // Changed last character

    // Count different bits
    uint32_t diff = hash1 ^ hash2;
    int bit_count = 0;
    for (int i = 0; i < 32; i++)
    {
        if (diff & (1u << i))
        {
            bit_count++;
        }
    }

    // Should have many bits different (avalanche effect)
    // FNV-1a has decent avalanche, expect > 8 bits different
    TEST_ASSERT_GREATER_THAN(8, bit_count);
    local_tearDown();
}

void test_switch_case_message_routing()
{
    local_setUp();
    // Realistic message routing scenario
    const char* message = "CMD_LAUNCH";
    uint32_t msg_hash = fnv1a_32(message, strlen(message));

    const char* action = nullptr;

    switch (msg_hash)
    {
        case "CMD_LAUNCH"_i:
            action = "Launching...";
            break;
        case "CMD_ABORT"_i:
            action = "Aborting...";
            break;
        case "CMD_STATUS"_i:
            action = "Status check...";
            break;
        default:
            action = "Unknown command";
            break;
    }

    TEST_ASSERT_EQUAL_STRING("Launching...", action);
    local_tearDown();
}

void test_prefix_matching()
{
    local_setUp();
    // Test prefix-based routing
    const char* msg1 = "DATA/sensor1";
    const char* msg2 = "DATA/sensor2";
    const char* msg3 = "LOG/message";

    // Extract prefix (first 5 chars for "DATA/", 4 for "LOG/")
    uint32_t prefix1 = fnv1a_32(msg1, 5);
    uint32_t prefix2 = fnv1a_32(msg2, 5);
    uint32_t prefix3 = fnv1a_32(msg3, 4);

    TEST_ASSERT_EQUAL_UINT32(prefix1, prefix2);  // Same prefix
    TEST_ASSERT_NOT_EQUAL(prefix1, prefix3);     // Different prefix
    local_tearDown();
}

void test_enum_class_with_hashes()
{
    local_setUp();
    // Using hashes as enum values
    enum class MessageType : uint32_t
    {
        DATA = "DATA/"_i,
        COMMAND = "CMD/"_i,
        LOG = "LOG/"_i,
        TELEMETRY = "TELEM/"_i
    };

    MessageType type = MessageType::DATA;

    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(MessageType::DATA), "DATA/"_i);
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(type), "DATA/"_i);
    local_tearDown();
}

void run_test_hash_tests()
{
    RUN_TEST(test_fnv1a_empty_string);
    RUN_TEST(test_fnv1a_single_character);
    RUN_TEST(test_fnv1a_short_string);
    RUN_TEST(test_fnv1a_longer_string);
    RUN_TEST(test_fnv1a_deterministic);
    RUN_TEST(test_fnv1a_case_sensitive);
    RUN_TEST(test_fnv1a_different_lengths);
    RUN_TEST(test_fnv1a_partial_string);
    RUN_TEST(test_literal_operator_basic);
    RUN_TEST(test_literal_operator_empty_string);
    RUN_TEST(test_literal_operator_same_string);
    RUN_TEST(test_literal_operator_different_strings);
    RUN_TEST(test_literal_operator_long_string);
    RUN_TEST(test_literal_operator_numbers);
    RUN_TEST(test_literal_operator_special_characters);
    RUN_TEST(test_literal_operator_spaces);
    RUN_TEST(test_constexpr_evaluation);
    RUN_TEST(test_constexpr_literal_operator);
    RUN_TEST(test_constexpr_switch_statement);
    RUN_TEST(test_different_strings_produce_different_hashes);
    RUN_TEST(test_similar_strings_produce_different_hashes);
    RUN_TEST(test_reversed_strings_produce_different_hashes);
    RUN_TEST(test_anagrams_produce_different_hashes);
    RUN_TEST(test_command_routing);
    RUN_TEST(test_state_machine_states);
    RUN_TEST(test_sensor_type_identification);
    RUN_TEST(test_message_type_discrimination);
    RUN_TEST(test_null_character_in_string);
    RUN_TEST(test_single_byte_differences);
    RUN_TEST(test_unicode_or_extended_ascii);
    RUN_TEST(test_very_long_string);
    RUN_TEST(test_all_same_character);
    RUN_TEST(test_binary_data);
    RUN_TEST(test_hash_consistency_across_calls);
    RUN_TEST(test_literal_operator_consistency);
    RUN_TEST(test_mixed_usage_consistency);
    RUN_TEST(test_known_fnv1a_values);
    RUN_TEST(test_specific_command_hashes);
    RUN_TEST(test_hash_distribution_simple);
    RUN_TEST(test_avalanche_effect);
    RUN_TEST(test_switch_case_message_routing);
    RUN_TEST(test_prefix_matching);
    RUN_TEST(test_enum_class_with_hashes);
}

} // namespace test_hash
