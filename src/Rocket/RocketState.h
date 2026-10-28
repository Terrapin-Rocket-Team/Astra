#ifndef ROCKET_STATE_H
#define ROCKET_STATE_H

#include <State/State.h>
#include <Filters/LinearKalmanFilter.h>
#include <Sensors/Sensor.h>
#include <Sensors/SensorManager/SensorManager.h>
#include "FlightStage.h"

using namespace astra;

namespace astra_rocket {
class AstraRocketConfig;

/**
 * RocketState: Rocket-specific state estimator
 *
 * Extends Astra's State class with rocket flight-specific variables:
 * - Flight stage detection and tracking
 * - Apogee estimation and prediction
 * - Vertical acceleration and velocity tracking
 * - Off-vertical angle for stability analysis
 */
class RocketState : public State {
public:
    /**
     * Constructor
     * @param filter Optional Kalman filter for state estimation
     * @param orientationFilter Optional Mahony AHRS filter for orientation estimation
     */
    RocketState(LinearKalmanFilter *filter, MahonyAHRS *orientationFilter, const AstraRocketConfig *config = nullptr);

    /**
     * Get current flight stage
     */
    FlightStage getFlightStage() const { return currentStage; }

    /**
     * Get off-vertical angle (degrees from vertical)
     */
    double getOffVerticalAngle() const { return offVerticalAngle; }

    /**
     * Get rocket-frame orientation in earth coordinates.
     * This differs from State::getOrientation(), which is kept as board->earth
     * for compatibility with older Astra plumbing.
     */
    Quaternion getRocketOrientation() const;

    /**
     * Get altitude above ground level (meters)
     */
    double getAltitudeAGL() const { return position.z(); }

    /**
     * Get the barometric ground reference used for AGL calculations (meters MSL)
     */
    double getGroundLevelMSL() const { return origin.z(); }

    /**
     * Set ground level altitude (for AGL calculations)
     * Typically called during pad idle to establish reference
     */
    void setGroundLevel(double altitudeMSL);

    /**
     * Manually set flight stage (for testing or overrides)
     */
    void setFlightStage(FlightStage stage);

    /**
     * Get time in current stage (seconds)
     */
    double getTimeInStage() const { return timeInCurrentStage; }
    double getTimeSinceLaunch() const { return currentStage != PAD_IDLE ? currentTimeSeconds - timeOfLaunch : 0;}

    /**
     * Update orientation estimate from gyro and accel data
     * Allows RocketState to enforce gyro-only mode after liftoff
     */
    void updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt) override;

    /**
     * Update orientation estimate from gyro, accel, and mag data (9-DOF)
     */
    void updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, const Vector<3> &mag, double dt) override;

    int update() override;
    void predict(double dt) override;
    void updateGPSMeasurement(const Vector<3> &gpsPos, const Vector<3> &gpsVel) override;
    void updateBaroMeasurement(double baroAlt) override;

    // Test/simulation helper: pull KF state into RocketState without full Astra loop
    void predictState(double currentTimeSec = -1);

    // Test helper: accepts a SensorManager to match existing tests
    RocketState &withSensorManager(SensorManager *sm);

    // Test helper: force frame lock without waiting for liftoff detection
    void lockFrameForTest();

private:
    enum UpAxis
    {
        POS_X,
        NEG_X,
        POS_Y,
        NEG_Y,
        POS_Z,
        NEG_Z
    };

    // Flight stage tracking
    FlightStage currentStage;
    FlightStage previousStage;
    double timeInCurrentStage;
    unsigned long stageStartTime;

    // Orientation tracking
    double offVerticalAngle;     // Angle from vertical axis
    Quaternion mountQuat_rb;     // Rocket->Board mounting rotation (R->B)
    UpAxis currentUpAxis;
    UpAxis pendingUpAxis;
    int axisStableCount;
    bool frameLocked;
    bool forceGyroOnly;
    SensorManager *sensorManager;
    double timeOfLaunch = 0;

    // Helper methods
    void detectFlightStage();
    void calculateTilt();
    void updateMountingAlignment(const Vector<3> &accel);
    void snapPadOrientation(const Vector<3> &accel);
    void computeMountingQuaternion(UpAxis axis);
    UpAxis chooseUpAxis(const Vector<3> &accel, double &bestDot) const;

    // Alignment tuning
    static constexpr double AXIS_SWITCH_DOT_THRESHOLD = 0.90;
    static constexpr int AXIS_SWITCH_STABLE_COUNT = 10;
    // Stage detection thresholds (config-driven with safe defaults)
    static constexpr double DEFAULT_LIFTOFF_ACCEL_THRESHOLD_G = 3.0;
    static constexpr unsigned long DEFAULT_LIFTOFF_DURATION_MS = 100;
    static constexpr double DEFAULT_BURNOUT_ACCEL_THRESHOLD_G = 1.5;
    static constexpr unsigned long DEFAULT_BURNOUT_DURATION_MS = 200;
    static constexpr double DEFAULT_APOGEE_VELOCITY_THRESHOLD = 2.0;    // m/s
    static constexpr double DEFAULT_LANDING_VELOCITY_THRESHOLD = 1.0;   // m/s
    static constexpr unsigned long DEFAULT_LANDING_DURATION_MS = 3000;   // ms

    // Descent rate detection thresholds
    static constexpr double DROGUE_DESCENT_MIN = 5.0;           // m/s minimum under drogue
    static constexpr double DROGUE_DESCENT_MAX = 120.0;          // m/s maximum under drogue
    static constexpr double MAIN_DESCENT_MIN = 2.0;             // m/s minimum under main
    static constexpr double MAIN_DESCENT_MAX = 10.0;            // m/s maximum under main
    static constexpr double MAIN_DEPLOY_ALTITUDE = 400.0;       // m AGL to start expecting main
    static constexpr unsigned long DROGUE_DETECT_DURATION = 2000; // ms to confirm drogue
    static constexpr unsigned long MAIN_DETECT_DURATION = 2000;   // ms to confirm main

    // Config-resolved thresholds.
    double liftoffAccelThresholdMs2() const;
    unsigned long liftoffDurationMs() const;
    double burnoutAccelThresholdMs2() const;
    unsigned long burnoutDurationMs() const;
    double apogeeVelocityThresholdMs() const;
    double landingVelocityThresholdMs() const;
    unsigned long landingDurationMs() const;

    // Tracking for sustained condition detection
    unsigned long highAccelStartTime;
    unsigned long lowAccelStartTime;
    unsigned long lowVelocityStartTime;
    unsigned long drogueDetectStartTime;
    unsigned long mainDetectStartTime;
    bool highAccelDetected;
    bool lowAccelDetected;
    bool lowVelocityDetected;
    bool drogueRateDetected;
    bool mainRateDetected;
    bool padFilterZeroed;
    const AstraRocketConfig *flightConfig;
};

} // namespace astra_rocket

#endif // ROCKET_STATE_H
