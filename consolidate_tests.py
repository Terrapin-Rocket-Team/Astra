#!/usr/bin/env python3
"""
Script to consolidate unit tests into logical groups to reduce rebuild time.
"""

import os
import shutil
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

def extract_test_content(cpp_file_path):
    """Extract test functions and fixture from a test cpp file."""
    with open(cpp_file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract includes (skip unity.h and common ones)
    includes = []
    for line in content.split('\n'):
        if line.strip().startswith('#include') and \
           'unity.h' not in line and \
           'NativeTestHelper.h' not in line:
            includes.append(line.strip())

    # Extract using namespace declarations
    using_namespaces = re.findall(r'using namespace .*?;', content)

    # Extract test fixture variables (global variables before setUp)
    fixture_match = re.search(r'((?:(?:class|struct)\s+\w+.*?{.*?};|(?:\w+(?:<[^>]+>)?\s+\w+(?:\s*=\s*[^;]+)?;))\s*)+?(?=\s*void\s+setUp)', content, re.DOTALL)
    fixture = fixture_match.group(1).strip() if fixture_match else ''

    # Extract setUp and tearDown
    setup_match = re.search(r'void\s+setUp\s*\([^)]*\)\s*{[^}]*}', content, re.DOTALL)
    teardown_match = re.search(r'void\s+tearDown\s*\([^)]*\)\s*{[^}]*}', content, re.DOTALL)

    setup = setup_match.group(0) if setup_match else ''
    teardown = teardown_match.group(0) if teardown_match else ''

    # Extract all test functions (functions starting with test_)
    test_functions = re.findall(r'void\s+(test_\w+)\s*\([^)]*\)\s*{.*?(?=\nvoid\s+(?:test_|setUp|tearDown|main)|\nint\s+main|$)', content, re.DOTALL)
    test_func_names = [name for name, _ in re.findall(r'void\s+(test_\w+)\s*\([^)]*\)', content)]

    # Extract full test function definitions
    test_defs = []
    for func_name in test_func_names:
        func_match = re.search(rf'void\s+{func_name}\s*\([^)]*\)\s*{{.*?^}}', content, re.DOTALL | re.MULTILINE)
        if func_match:
            test_defs.append(func_match.group(0))

    return {
        'includes': includes,
        'using_namespaces': using_namespaces,
        'fixture': fixture,
        'setup': setup,
        'teardown': teardown,
        'test_functions': test_defs,
        'test_names': test_func_names,
    }

def generate_header_file(test_name, test_data):
    """Generate a header file for a test module."""
    header = f"#pragma once\n\n"
    header += f"// Test module for {test_name}\n\n"

    # Add includes
    for inc in test_data['includes']:
        header += inc + '\n'
    header += '\n'

    # Add using namespaces
    for using in test_data['using_namespaces']:
        header += using + '\n'
    if test_data['using_namespaces']:
        header += '\n'

    # Add fixture
    if test_data['fixture']:
        header += f"// Test fixture for {test_name}\n"
        header += f"namespace {test_name}_ns {{\n"
        for line in test_data['fixture'].split('\n'):
            header += '    ' + line + '\n'
        header += "}\n\n"

    # Add forward declarations for test functions
    for test_func in test_data['test_names']:
        header += f"void {test_func}();\n"
    header += '\n'

    # Add run function declaration
    header += f"void run_{test_name}_suite();\n"

    return header

def generate_cpp_file(test_name, test_data):
    """Generate a cpp file for a test module."""
    cpp = f'#include "{test_name}_suite.h"\n'
    cpp += '#include <unity.h>\n\n'

    # Add test fixture to namespace if it exists
    if test_data['fixture']:
        cpp += f"using namespace {test_name}_ns;\n\n"

    # Add setUp and tearDown with namespace prefix
    if test_data['setup']:
        setup_body = re.search(r'{(.*)}', test_data['setup'], re.DOTALL)
        if setup_body and test_data['fixture']:
            # Modify setUp to use namespace
            cpp += f"namespace {test_name}_ns {{\n"
            cpp += f"    void setUp() {{\n"
            for line in setup_body.group(1).strip().split('\n'):
                cpp += '        ' + line + '\n'
            cpp += "    }\n"

            teardown_body = re.search(r'{(.*)}', test_data['teardown'], re.DOTALL)
            if teardown_body:
                cpp += f"    void tearDown() {{\n"
                for line in teardown_body.group(1).strip().split('\n'):
                    cpp += '        ' + line + '\n'
                cpp += "    }\n"
            cpp += "}\n\n"

    # Add test functions
    for test_func in test_data['test_functions']:
        cpp += test_func + '\n\n'

    # Add run function
    cpp += f"void run_{test_name}_suite() {{\n"
    if test_data['fixture'] and test_data['setup']:
        cpp += f"    {test_name}_ns::setUp();\n\n"

    for test_name_func in test_data['test_names']:
        cpp += f"    RUN_TEST({test_name_func});\n"

    if test_data['fixture'] and test_data['teardown']:
        cpp += f"\n    {test_name}_ns::tearDown();\n"
    cpp += "}\n"

    return cpp

def create_suite_main(suite_name, test_modules):
    """Create the main test file for a suite."""
    main_cpp = '#include <unity.h>\n'
    main_cpp += '#include "NativeTestHelper.h"\n\n'

    # Include all test module headers
    for module in test_modules:
        main_cpp += f'#include "{module}_suite.h"\n'
    main_cpp += '\n'

    # Global setUp/tearDown
    main_cpp += 'void setUp(void) {\n'
    main_cpp += '    // Global setup for the suite\n'
    main_cpp += '}\n\n'

    main_cpp += 'void tearDown(void) {\n'
    main_cpp += '    // Global teardown for the suite\n'
    main_cpp += '}\n\n'

    # Main function
    main_cpp += 'int main(int argc, char **argv) {\n'
    main_cpp += '    UNITY_BEGIN();\n\n'

    for module in test_modules:
        main_cpp += f'    run_{module}_suite();\n'

    main_cpp += '\n    UNITY_END();\n'
    main_cpp += '    return 0;\n'
    main_cpp += '}\n'

    return main_cpp

def consolidate_tests():
    """Main function to consolidate tests."""
    test_dir = 'test'

    for suite_name, test_modules in TEST_GROUPS.items():
        print(f"\nCreating {suite_name}...")
        suite_dir = os.path.join(test_dir, suite_name)
        os.makedirs(suite_dir, exist_ok=True)

        valid_modules = []

        for test_module in test_modules:
            test_path = os.path.join(test_dir, test_module)

            # Find the cpp file in the test directory
            cpp_files = []
            if os.path.exists(test_path):
                for file in os.listdir(test_path):
                    if file.endswith('.cpp'):
                        cpp_files.append(os.path.join(test_path, file))

            if not cpp_files:
                print(f"  Warning: No cpp file found for {test_module}, skipping")
                continue

            cpp_file = cpp_files[0]
            print(f"  Processing {test_module}...")

            # Extract test content
            test_data = extract_test_content(cpp_file)

            # Generate header and cpp files
            header_content = generate_header_file(test_module, test_data)
            cpp_content = generate_cpp_file(test_module, test_data)

            # Write files
            header_path = os.path.join(suite_dir, f"{test_module}_suite.h")
            cpp_path = os.path.join(suite_dir, f"{test_module}_suite.cpp")

            with open(header_path, 'w', encoding='utf-8') as f:
                f.write(header_content)

            with open(cpp_path, 'w', encoding='utf-8') as f:
                f.write(cpp_content)

            valid_modules.append(test_module)

        # Create main suite file
        if valid_modules:
            main_content = create_suite_main(suite_name, valid_modules)
            main_path = os.path.join(suite_dir, f"{suite_name}.cpp")

            with open(main_path, 'w', encoding='utf-8') as f:
                f.write(main_content)

            print(f"  Created {suite_name} with {len(valid_modules)} test modules")

if __name__ == '__main__':
    consolidate_tests()
    print("\nTest consolidation complete!")
