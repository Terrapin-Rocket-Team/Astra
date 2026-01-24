#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"
#include <Sensors/MountingTransform.h>

using namespace astra;

// Set up and global variables or mocks for testing here
FakeIMU imu;

// These two functions are called before and after each test function, and are required in unity, even if empty.
void setUp(void)
{
    imu.begin(); // reset the imu before each test
    imu.setMountingOrientation(MountingOrientation::IDENTITY); // reset orientation to identity
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
}

// Test that IMU initializes correctly
void test_imu_begin()
{
    TEST_ASSERT_TRUE(imu.begin());
    TEST_ASSERT_TRUE(imu.isInitialized());

    // Check default values after initialization
    TEST_ASSERT_EQUAL_FLOAT(0, imu.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(0, imu.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(-9.81, imu.getAccel().z());
    TEST_ASSERT_EQUAL_FLOAT(0, imu.getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(0, imu.getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(0, imu.getAngVel().z());
}

// Test setting IMU data
void test_imu_set()
{
    imu.set(Vector<3>{0, 0, 9.8}, Vector<3>{0.01, 0.1, 0.05});
    imu.update();

    TEST_ASSERT_EQUAL_FLOAT(0, imu.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(0, imu.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(9.8, imu.getAccel().z());
    TEST_ASSERT_EQUAL_FLOAT(0.01, imu.getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(0.1, imu.getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(0.05, imu.getAngVel().z());
}

// Test getting individual sensor components
void test_imu_get_sensors()
{
    Accel* accel = imu.getAccelSensor();
    Gyro* gyro = imu.getGyroSensor();

    TEST_ASSERT_NOT_NULL(accel);
    TEST_ASSERT_NOT_NULL(gyro);

    imu.set(Vector<3>{1.0, 2.0, 3.0}, Vector<3>{0.1, 0.2, 0.3});
    imu.update();

    // Test that component sensors return the same data as the IMU
    TEST_ASSERT_EQUAL_FLOAT(1.0, accel->getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, accel->getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, accel->getAccel().z());

    TEST_ASSERT_EQUAL_FLOAT(0.1, gyro->getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, gyro->getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, gyro->getAngVel().z());
}

// Test IMU with mounting orientation
void test_imu_mounting_orientation()
{
    imu.set(Vector<3>{1.0, 2.0, 3.0}, Vector<3>{0.1, 0.2, 0.3});

    // Set FLIP_YZ orientation (Y and Z flip)
    imu.setMountingOrientation(MountingOrientation::FLIP_YZ);
    imu.update();

    Vector<3> accel = imu.getAccel();
    Vector<3> gyro = imu.getAngVel();

    // X unchanged, Y and Z flipped
    TEST_ASSERT_EQUAL_FLOAT(1.0, accel.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, accel.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, accel.z());

    TEST_ASSERT_EQUAL_FLOAT(0.1, gyro.x());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, gyro.y());
    TEST_ASSERT_EQUAL_FLOAT(-0.3, gyro.z());
}

// Test that mounting orientation is applied to component sensors
void test_imu_component_orientation()
{
    imu.set(Vector<3>{1.0, 2.0, 3.0}, Vector<3>{0.1, 0.2, 0.3});
    imu.setMountingOrientation(MountingOrientation::FLIP_XY);
    imu.update();

    Accel* accel = imu.getAccelSensor();
    Gyro* gyro = imu.getGyroSensor();

    // Component sensors should also have the orientation applied
    Vector<3> accelData = accel->getAccel();
    Vector<3> gyroData = gyro->getAngVel();

    // X and Y flipped, Z unchanged
    TEST_ASSERT_EQUAL_FLOAT(-1.0, accelData.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, accelData.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, accelData.z());

    TEST_ASSERT_EQUAL_FLOAT(-0.1, gyroData.x());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, gyroData.y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, gyroData.z());
}

// Test multiple updates with changing data
void test_imu_multiple_updates()
{
    // First update
    imu.set(Vector<3>{1.0, 0.0, 0.0}, Vector<3>{0.1, 0.0, 0.0});
    imu.update();

    TEST_ASSERT_EQUAL_FLOAT(1.0, imu.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(0.1, imu.getAngVel().x());

    // Second update with different values
    imu.set(Vector<3>{0.0, 2.0, 0.0}, Vector<3>{0.0, 0.2, 0.0});
    imu.update();

    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, imu.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, imu.getAngVel().y());

    // Third update
    imu.set(Vector<3>{0.0, 0.0, 3.0}, Vector<3>{0.0, 0.0, 0.3});
    imu.update();

    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, imu.getAccel().z());
    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, imu.getAngVel().z());
}

// Test zero values
void test_imu_zero_values()
{
    imu.set(Vector<3>{0.0, 0.0, 0.0}, Vector<3>{0.0, 0.0, 0.0});
    imu.update();

    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAccel().z());
    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, imu.getAngVel().z());
}

// Test large values
void test_imu_large_values()
{
    imu.set(Vector<3>{100.0, 200.0, 300.0}, Vector<3>{10.0, 20.0, 30.0});
    imu.update();

    TEST_ASSERT_EQUAL_FLOAT(100.0, imu.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(200.0, imu.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(300.0, imu.getAccel().z());
    TEST_ASSERT_EQUAL_FLOAT(10.0, imu.getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(20.0, imu.getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(30.0, imu.getAngVel().z());
}

// Test negative values
void test_imu_negative_values()
{
    imu.set(Vector<3>{-1.0, -2.0, -3.0}, Vector<3>{-0.1, -0.2, -0.3});
    imu.update();

    TEST_ASSERT_EQUAL_FLOAT(-1.0, imu.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, imu.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, imu.getAccel().z());
    TEST_ASSERT_EQUAL_FLOAT(-0.1, imu.getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, imu.getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(-0.3, imu.getAngVel().z());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_imu_begin);
    RUN_TEST(test_imu_set);
    RUN_TEST(test_imu_get_sensors);
    RUN_TEST(test_imu_mounting_orientation);
    RUN_TEST(test_imu_component_orientation);
    RUN_TEST(test_imu_multiple_updates);
    RUN_TEST(test_imu_zero_values);
    RUN_TEST(test_imu_large_values);
    RUN_TEST(test_imu_negative_values);

    UNITY_END();
    return 0;
}