#!/usr/bin/env python3
"""
Refactor test suites to use proper modular design with runTests() functions.
Each module is self-contained with its own runTests() that can be called from the suite.
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
    """Extract test function names from RUN_TEST calls."""
    run_test_calls = re.findall(r'RUN_TEST\((test_\w+)\)', content)
    if not run_test_calls:
        run_test_calls = re.findall(r'void\s+(test_\w+)\s*\(', content)
    return run_test_calls

def create_module_file(cpp_path, module_name):
    """
    Convert a test file into a self-contained module with runTests() function.
    """
    with open(cpp_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    # Extract test function names
    test_functions = extract_tests_from_main(content)

    # Remove main function
    main_pattern = r'int\s+main\s*\([^)]*\)\s*\{'
    main_match = re.search(main_pattern, content)

    if main_match:
        start_idx = main_match.start()
        brace_count = 0
        idx = main_match.end() - 1
        while idx < len(content):
            if content[idx] == '{':
                brace_count += 1
            elif content[idx] == '}':
                brace_count -= 1
                if brace_count == 0:
                    end_idx = idx + 1
                    break
            idx += 1
        else:
            end_idx = len(content)

        content = content[:start_idx].rstrip() + '\n' + content[end_idx:].lstrip()

    # Wrap everything in an anonymous namespace to avoid conflicts
    content = 'namespace {\n\n' + content + '\n} // anonymous namespace\n'

    # Add runTests() function (outside namespace - needs to be called from main)
    run_tests_func = f'\nvoid {module_name}_runTests() {{\n'
    run_tests_func += f'    setUp();\n\n'

    for test_func in test_functions:
        run_tests_func += f'    RUN_TEST({test_func});\n'

    run_tests_func += f'\n    tearDown();\n'
    run_tests_func += '}\n'

    content += run_tests_func

    return content, test_functions

def create_suite_main_file(suite_name, modules_info):
    """Create the main suite file that calls each module's runTests()."""
    content = '#include <unity.h>\n'
    content += '#include "NativeTestHelper.h"\n\n'

    # Forward declare runTests functions
    for module_name, _ in modules_info:
        content += f'void {module_name}_runTests();\n'
    content += '\n'

    # Include all module files
    for module_name, _ in modules_info:
        content += f'#include "{module_name}.cpp"\n'
    content += '\n'

    # Global setUp/tearDown
    content += 'void setUp(void) {\n'
    content += '    // Suite-level setup (currently unused)\n'
    content += '}\n\n'

    content += 'void tearDown(void) {\n'
    content += '    // Suite-level teardown (currently unused)\n'
    content += '}\n\n'

    # Main function
    content += 'int main(int argc, char **argv) {\n'
    content += '    UNITY_BEGIN();\n\n'

    # Call each module's runTests
    for module_name, test_count in modules_info:
        content += f'    // {module_name} ({test_count} tests)\n'
        content += f'    {module_name}_runTests();\n\n'

    content += '    UNITY_END();\n'
    content += '    return 0;\n'
    content += '}\n'

    return content

def refactor_tests():
    """Main function to refactor tests."""
    test_dir = 'test'

    for suite_name, test_modules in TEST_GROUPS.items():
        print(f"\nRefactoring {suite_name}...")
        suite_dir = os.path.join(test_dir, suite_name)

        modules_info = []

        for test_module in test_modules:
            old_test_dir = os.path.join(test_dir, test_module)

            # Find the original cpp file
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

            try:
                # Create the module file
                modified_content, test_functions = create_module_file(cpp_file, test_module)

                # Write directly to suite directory (no subdirectories)
                output_path = os.path.join(suite_dir, f"{test_module}.cpp")
                with open(output_path, 'w', encoding='utf-8') as f:
                    f.write(modified_content)

                modules_info.append((test_module, len(test_functions)))
            except Exception as e:
                print(f"    Error processing {test_module}: {e}")
                import traceback
                traceback.print_exc()
                continue

        # Create main suite file
        if modules_info:
            main_content = create_suite_main_file(suite_name, modules_info)
            main_path = os.path.join(suite_dir, f"{suite_name}.cpp")

            with open(main_path, 'w', encoding='utf-8') as f:
                f.write(main_content)

            print(f"  Created {suite_name}.cpp with {len(modules_info)} modules")
            print(f"    Total tests: {sum(count for _, count in modules_info)}")

    print("\nRefactoring complete!")
    print(f"\nRefactored {sum(len(modules) for modules in TEST_GROUPS.values())} test modules into {len(TEST_GROUPS)} suites")

if __name__ == '__main__':
    refactor_tests()
