#include <unity.h>

#include "test_accel.h"
#include "test_gyro.h"
#include "test_mag.h"
#include "test_baro.h"
#include "test_gps.h"
#include "test_imu.h"
#include "test_dual_range_accel.h"
#include "test_rotatable_sensor.h"
#include "test_hitl_sensors.h"
#include "test_voltage_sensor.h"
#include "test_sensor.h"
#include "test_sensor_manager.h"

void setUp(void)
{
    // Called before each test
}

void tearDown(void)
{
    // Called after each test
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    test_accel::run_test_accel_tests();
    test_gyro::run_test_gyro_tests();
    test_mag::run_test_mag_tests();
    test_baro::run_test_baro_tests();
    test_gps::run_test_gps_tests();
    test_imu::run_test_imu_tests();
    test_dual_range_accel::run_test_dual_range_accel_tests();
    test_rotatable_sensor::run_test_rotatable_sensor_tests();
    test_hitl_sensors::run_test_hitl_sensors_tests();
    test_voltage_sensor::run_test_voltage_sensor_tests();
    test_sensor::run_test_sensor_tests();
    test_sensor_manager::run_test_sensor_manager_tests();

    UNITY_END();
    return 0;
}
