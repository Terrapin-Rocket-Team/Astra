# Test Suite Consolidation

## Overview

The unit tests have been consolidated from 31 individual test suites into 8 logical groups to dramatically reduce rebuild times when running tests.

## Why This Change?

Previously, each test suite was a separate directory that PlatformIO would build independently. This meant:
- Running all 31 tests required 31 separate builds
- Each build recompiled the same source code
- Total test time was dominated by compilation, not actual testing

Now:
- Tests are grouped into 8 logical suites based on the system they test
- Each suite builds once and runs multiple related tests
- Significant reduction in total test time

## Test Suite Organization

### 1. `test_sensors_suite` (153 tests)
Tests for all sensor types:
- Accelerometer (test_accel)
- Gyroscope (test_gyro)
- Magnetometer (test_mag)
- Barometer (test_baro)
- GPS (test_gps)
- IMU (test_imu)
- Dual-range accelerometer (test_dual_range_accel)
- Rotatable sensor (test_rotatable_sensor)
- Voltage sensor (test_voltage_sensor)
- Sensor base class (test_sensor)
- Sensor manager (test_sensor_manager)

### 2. `test_math_suite` (100 tests)
Mathematical utilities:
- Matrix operations (test_matrix)
- Vector operations (test_vector)
- Quaternion operations (test_quaternion)

### 3. `test_filters_suite` (81 tests)
Filtering algorithms:
- Mahony filter (test_mahony)
- Kalman filter (test_kalman_filter)
- Mounting transform (test_mounting_transform)

### 4. `test_data_suite` (123 tests)
Data recording and reporting:
- Data reporter (test_data_reporter)
- Logger (test_logger)
- Storage (test_storage)
- Hash utilities (test_hash)

### 5. `test_communication_suite` (81 tests)
Communication components:
- Serial router (test_serial_router)
- Command handler (test_cmd_handler)
- HITL parser (test_hitl_parser)
- SITL connection (test_sitl)

### 6. `test_state_suite` (102 tests)
State management:
- State machine (test_state)
- Default state (test_default_state)
- Astra configuration (test_astra_config)

### 7. `test_utils_suite` (81 tests)
Utility components:
- Circular buffer (test_circular_buffer)
- Blink/buzz indicator (test_blinkbuzz)

### 8. `test_integration_suite` (34 tests)
High-level integration:
- Astra system integration (test_astra)

**Total: 755 tests across 8 suites**

## Running Tests

### Run all consolidated tests
```bash
pio test -e native
```

### Run a specific suite
```bash
pio test -e native -f test_math_suite
pio test -e native -f test_sensors_suite
# etc.
```

### Run tests in verbose mode
```bash
pio test -e native -f test_math_suite -v
```

## Technical Implementation

### Structure
Each consolidated suite follows this pattern:
```
test_sensors_suite/
├── test_sensors_suite.cpp          # Main orchestrator file
├── test_accel/
│   └── test_accel.inc               # Individual test module
├── test_gyro/
│   └── test_gyro.inc                # Individual test module
└── ...
```

### Key Changes
1. **Function Renaming**: All test functions are prefixed with their module name to avoid naming conflicts
   - `test_default_constructor()` → `test_matrix_test_default_constructor()`

2. **setUp/tearDown**: Each module's setUp and tearDown functions are renamed:
   - `setUp()` → `test_matrix_setUp()`
   - `tearDown()` → `test_matrix_tearDown()`

3. **File Extension**: Individual test files use `.inc` extension to prevent separate compilation

4. **Main Function**: Each suite has a single `main()` function that runs all tests from all included modules

### Example Suite File
```cpp
#include <unity.h>
#include "NativeTestHelper.h"

#include "test_matrix/test_matrix.inc"
#include "test_vector/test_vector.inc"
#include "test_quaternion/test_quaternion.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_matrix
    test_matrix_setUp();
    RUN_TEST(test_matrix_test_default_constructor);
    RUN_TEST(test_matrix_test_allocating_constructor);
    // ... more tests ...
    test_matrix_tearDown();

    // Tests from test_vector
    test_vector_setUp();
    RUN_TEST(test_vector_test_default_constructor);
    // ... more tests ...
    test_vector_tearDown();

    UNITY_END();
    return 0;
}
```

## Maintaining the Tests

### Adding New Tests to an Existing Module
1. Add your test function to the appropriate `.inc` file
2. The function will be automatically prefixed with the module name
3. Add a `RUN_TEST()` call in the suite's main `.cpp` file

### Creating a New Test Module
1. Create the test file in the original `test/test_your_module/` directory
2. Run the consolidation script to add it to the appropriate suite:
   ```bash
   python consolidate_tests_simple_rename.py
   ```
3. Delete old `.cpp` files from suite subdirectories:
   ```bash
   find test/*_suite -name "*.cpp" -not -name "*_suite.cpp" -delete
   ```

### Creating a New Suite
1. Edit `consolidate_tests_simple_rename.py`
2. Add your new suite to the `TEST_GROUPS` dictionary
3. Run the script and cleanup as described above

## Files in This Directory

- `consolidate_tests_simple_rename.py` - The main consolidation script
- `test_*_suite/` - The 8 consolidated test suite directories
- `test_*/` - Original individual test directories (kept for reference and new test development)
- Individual test directories are ignored via `test_ignore` in `platformio.ini`

## Rollback

If you need to revert to individual test suites:
1. Remove the `test_ignore` section from `platformio.ini`
2. Delete the `test_*_suite` directories
3. The original individual tests will be used again

## Performance Improvement

Before consolidation:
- 31 separate builds required
- Each test rebuilt the entire codebase
- Significant overhead from repeated compilation

After consolidation:
- Only 8 builds required
- ~75% reduction in total build count
- Dramatically faster test execution times

## Troubleshooting

### "Multiple definition" errors
This means old `.cpp` files weren't deleted from suite subdirectories. Run:
```bash
find test/*_suite -name "*.cpp" -not -name "*_suite.cpp" -delete
```

### Test not found
Make sure the test is listed in `test_ignore` in `platformio.ini` if it's an old individual test, or use the suite name if it's consolidated.

### Build errors after adding new tests
Re-run the consolidation script and cleanup to ensure all test functions are properly renamed.
