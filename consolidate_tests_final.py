#!/usr/bin/env python3
"""
Script to consolidate unit tests into logical groups to reduce rebuild time.
Final simple version: wrap each module in a namespace and remove main().
"""

import os
import re

# Define test groupings
TEST_GROUPS = {
    'test_sensors_suite': [
        'test_accel',
        'test_gyro',
        'test_mag',
        'test_baro',
        'test_gps',
        'test_imu',
        'test_dual_range_accel',
        'test_rotatable_sensor',
        'test_voltage_sensor',
        'test_sensor',
        'test_sensor_manager',
    ],
    'test_math_suite': [
        'test_matrix',
        'test_vector',
        'test_quaternion',
    ],
    'test_filters_suite': [
        'test_mahony',
        'test_kalman_filter',
        'test_mounting_transform',
    ],
    'test_data_suite': [
        'test_data_reporter',
        'test_logger',
        'test_storage',
        'test_hash',
    ],
    'test_communication_suite': [
        'test_serial_router',
        'test_cmd_handler',
        'test_hitl_parser',
        'test_sitl',
    ],
    'test_state_suite': [
        'test_state',
        'test_default_state',
        'test_astra_config',
    ],
    'test_utils_suite': [
        'test_circular_buffer',
        'test_blinkbuzz',
    ],
    'test_integration_suite': [
        'test_astra',
    ],
}

def extract_tests_from_main(content):
    """Extract test function names from RUN_TEST calls in main()."""
    run_test_calls = re.findall(r'RUN_TEST\((test_\w+)\)', content)
    if not run_test_calls:
        # Fallback: find function definitions
        run_test_calls = re.findall(r'void\s+(test_\w+)\s*\(', content)
    return run_test_calls

def convert_test_to_module(cpp_path, module_name):
    """
    Wrap test module in a namespace and remove main().
    """
    with open(cpp_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    # Extract test function names from RUN_TEST before we modify anything
    original_test_functions = extract_tests_from_main(content)

    # Remove main function by finding it and deleting it
    # Find "int main" and then find matching closing brace
    main_pattern = r'int\s+main\s*\([^)]*\)\s*\{'
    main_match = re.search(main_pattern, content)

    if main_match:
        start_idx = main_match.start()
        # Count braces to find the end
        brace_count = 0
        idx = main_match.end() - 1  # Start from the opening brace
        while idx < len(content):
            if content[idx] == '{':
                brace_count += 1
            elif content[idx] == '}':
                brace_count -= 1
                if brace_count == 0:
                    # Found the closing brace
                    end_idx = idx + 1
                    break
            idx += 1
        else:
            end_idx = len(content)

        # Remove main function and any trailing whitespace
        content = content[:start_idx].rstrip() + '\n' + content[end_idx:].lstrip()

    # Now wrap the entire content in a namespace
    namespace = module_name
    wrapped_content = f'namespace {namespace} {{\n\n{content}\n\n}} // namespace {namespace}\n'

    # Create prefixed test names for the main file
    prefixed_test_functions = [f"{namespace}::{func}" for func in original_test_functions]

    return wrapped_content, prefixed_test_functions

def create_main_file(suite_name, modules_info):
    """Create the main test orchestrator file for a suite."""
    content = '#include <unity.h>\n'
    content += '#include "NativeTestHelper.h"\n\n'

    # Include all module files
    for module_name, _ in modules_info:
        content += f'#include "{module_name}/{module_name}.cpp"\n'

    content += '\n'

    # Global setUp/tearDown
    content += 'void setUp(void) {\n'
    content += '    // Global suite setup\n'
    content += '}\n\n'

    content += 'void tearDown(void) {\n'
    content += '    // Global suite teardown\n'
    content += '}\n\n'

    # Main function
    content += 'int main(int argc, char **argv) {\n'
    content += '    UNITY_BEGIN();\n\n'

    # Run all tests from all modules
    for module_name, test_functions in modules_info:
        if test_functions:
            content += f'    // Tests from {module_name}\n'
            for test_func in test_functions:
                content += f'    RUN_TEST({test_func});\n'
            content += '\n'

    content += '    UNITY_END();\n'
    content += '    return 0;\n'
    content += '}\n'

    return content

def consolidate_tests():
    """Main function to consolidate tests."""
    test_dir = 'test'

    for suite_name, test_modules in TEST_GROUPS.items():
        print(f"\nCreating {suite_name}...")
        suite_dir = os.path.join(test_dir, suite_name)
        os.makedirs(suite_dir, exist_ok=True)

        modules_info = []

        for test_module in test_modules:
            old_test_dir = os.path.join(test_dir, test_module)

            # Find the cpp file
            cpp_file = None
            if os.path.exists(old_test_dir):
                for file in os.listdir(old_test_dir):
                    if file.endswith('.cpp'):
                        cpp_file = os.path.join(old_test_dir, file)
                        break

            if not cpp_file:
                print(f"  Warning: No cpp file found for {test_module}, skipping")
                continue

            print(f"  Processing {test_module}...")

            # Create subdirectory for this module within the suite
            module_dir = os.path.join(suite_dir, test_module)
            os.makedirs(module_dir, exist_ok=True)

            # Convert the test file
            try:
                modified_content, test_functions = convert_test_to_module(cpp_file, test_module)

                # Write the modified file
                output_path = os.path.join(module_dir, f"{test_module}.cpp")
                with open(output_path, 'w', encoding='utf-8') as f:
                    f.write(modified_content)

                modules_info.append((test_module, test_functions))
            except Exception as e:
                print(f"    Error processing {test_module}: {e}")
                continue

        # Create main suite file
        if modules_info:
            main_content = create_main_file(suite_name, modules_info)
            main_path = os.path.join(suite_dir, f"{suite_name}.cpp")

            with open(main_path, 'w', encoding='utf-8') as f:
                f.write(main_content)

            print(f"  Created {suite_name}.cpp with {len(modules_info)} test modules")
            print(f"  Total tests: {sum(len(tests) for _, tests in modules_info)}")

    print("\nTest consolidation complete!")
    print(f"\nConsolidated {sum(len(modules) for modules in TEST_GROUPS.values())} test directories into {len(TEST_GROUPS)} suites")

if __name__ == '__main__':
    consolidate_tests()
