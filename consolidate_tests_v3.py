#!/usr/bin/env python3
"""
Script to consolidate unit tests into logical groups to reduce rebuild time.
This version properly removes main() and updates RUN_TEST calls.
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

def extract_tests_from_cpp(content):
    """Extract test function names from content."""
    # Find the RUN_TEST calls to preserve order
    run_test_calls = re.findall(r'RUN_TEST\((test_\w+)\)', content)

    # If no RUN_TEST calls, find function definitions
    if not run_test_calls:
        run_test_calls = re.findall(r'void\s+(test_\w+)\s*\(', content)

    return run_test_calls

def convert_test_to_module(cpp_path, module_name):
    """
    Convert a standalone test file into a module with prefixed function names.
    Returns the modified content and the list of prefixed test functions.
    """
    with open(cpp_path, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    content = ''.join(lines)

    # Extract test function names before prefixing
    original_test_functions = extract_tests_from_cpp(content)

    # Find all test function definitions and rename them
    test_func_names = set(re.findall(r'void\s+(test_\w+)\s*\(', content))

    for func_name in test_func_names:
        new_name = f"{module_name}_{func_name}"
        # Replace function definition
        content = re.sub(
            rf'\bvoid\s+{func_name}\s*\(',
            f'void {new_name}(',
            content
        )

    # Wrap setUp and tearDown in namespace
    namespace_name = module_name + "_ns"

    # Find and wrap setUp
    setup_pattern = r'void\s+setUp\s*\([^)]*\)\s*\{[^}]*\}'
    setup_match = re.search(setup_pattern, content, re.DOTALL)
    if setup_match:
        setup_func = setup_match.group(0)
        indented_setup = '\n'.join(['    ' + line for line in setup_func.split('\n')])
        content = content.replace(setup_func, f'namespace {namespace_name} {{\n{indented_setup}\n}}')

    # Find and wrap tearDown (it might already be in the namespace)
    teardown_pattern = r'(?<!namespace\s+\w+\s*\{\s*)void\s+tearDown\s*\([^)]*\)\s*\{[^}]*\}'
    teardown_match = re.search(teardown_pattern, content, re.DOTALL)
    if teardown_match:
        teardown_func = teardown_match.group(0)
        # Check if it's already inside the namespace we just created
        namespace_start = content.find(f'namespace {namespace_name} {{')
        teardown_start = content.find(teardown_func)

        if namespace_start == -1 or teardown_start < namespace_start or teardown_start > content.find('}}', namespace_start):
            indented_teardown = '\n'.join(['    ' + line for line in teardown_func.split('\n')])
            # If there's already a namespace, put tearDown inside it
            if namespace_start != -1 and teardown_start > namespace_start:
                # tearDown is after namespace, merge them
                content = content.replace(teardown_func, indented_teardown.lstrip())
                # Close the namespace after tearDown
                teardown_end = content.find(teardown_func) + len(indented_teardown.strip())
                if content[teardown_end:teardown_end+3] != '\n}}':
                    pass  # already handled
            else:
                # Separate namespace for tearDown
                content = content.replace(teardown_func, f'namespace {namespace_name} {{\n{indented_teardown}\n}}')

    # Remove main function - find from "int main" to the final closing brace
    # Use a more robust method: find "int main" and count braces
    main_start = re.search(r'\bint\s+main\s*\([^)]*\)\s*\{', content)
    if main_start:
        start_pos = main_start.start()
        brace_count = 0
        in_main = False
        end_pos = start_pos

        for i in range(start_pos, len(content)):
            if content[i] == '{':
                brace_count += 1
                in_main = True
            elif content[i] == '}':
                brace_count -= 1
                if in_main and brace_count == 0:
                    end_pos = i + 1
                    break

        # Remove the main function
        content = content[:start_pos] + content[end_pos:]

    # Create list of new (prefixed) test function names
    prefixed_test_functions = [f"{module_name}_{func}" for func in original_test_functions]

    return content, prefixed_test_functions

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

            # Convert the test file with prefixing
            modified_content, test_functions = convert_test_to_module(cpp_file, test_module)

            # Write the modified file
            output_path = os.path.join(module_dir, f"{test_module}.cpp")
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(modified_content)

            modules_info.append((test_module, test_functions))

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
