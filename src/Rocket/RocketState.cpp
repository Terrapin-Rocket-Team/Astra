#include "RocketState.h"
#include "AstraRocketConfig.h"
#include <RecordData/Logging/EventLogger.h>
#include <Math/Vector.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <Math/Matrix.h>

using namespace astra;

namespace astra_rocket
{

    RocketState::RocketState(LinearKalmanFilter *filter, MahonyAHRS *orientationFilter, const AstraRocketConfig *config)
        : State(filter, orientationFilter),
          currentStage(PAD_IDLE),
          previousStage(PAD_IDLE),
          timeInCurrentStage(0),
          stageStartTime(0),
          offVerticalAngle(0),
          mountQuat_rb(1.0, 0.0, 0.0, 0.0),
          currentUpAxis(POS_Z),
          pendingUpAxis(POS_Z),
          axisStableCount(0),
          frameLocked(false),
          forceGyroOnly(false),
          sensorManager(nullptr),
          highAccelStartTime(0),
          lowAccelStartTime(0),
          lowVelocityStartTime(0),
          drogueDetectStartTime(0),
          mainDetectStartTime(0),
          highAccelDetected(false),
          lowAccelDetected(false),
          lowVelocityDetected(false),
          drogueRateDetected(false),
          mainRateDetected(false),
          padFilterZeroed(false),
          flightConfig(config)
    {
        // Add rocket-specific columns to DataReporter
        insertColumn(0, "%d", &currentStage, "Flight Stage");
        addColumn("%0.3f", &offVerticalAngle, "Off-Vert Angle (deg)");
        addColumn("%0.3f", &timeInCurrentStage, "Time in Stage (s)");
    }

    void RocketState::setGroundLevel(double altitudeMSL)
    {
        origin.z() = altitudeMSL;
        LOGI("Ground level set to %0.2f m MSL (baro offset applied)", origin.z());
    }

    void RocketState::setFlightStage(FlightStage stage)
    {
        if (stage != currentStage)
        {
            previousStage = currentStage;
            currentStage = stage;
            stageStartTime = (unsigned long)(currentTimeSeconds * 1000.0);
            timeInCurrentStage = 0;
            highAccelDetected = false;
            lowAccelDetected = false;
            lowVelocityDetected = false;
            drogueRateDetected = false;
            mainRateDetected = false;
            highAccelStartTime = 0;
            lowAccelStartTime = 0;
            lowVelocityStartTime = 0;
            drogueDetectStartTime = 0;
            mainDetectStartTime = 0;
            LOGI("Flight stage transition: %s -> %s",
                 flightStageToString(previousStage),
                 flightStageToString(currentStage));
        }
    }

    void RocketState::updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt)
    {
        if (!orientationFilter)
            return;

        if (!frameLocked && currentStage == PAD_IDLE)
        {
            updateMountingAlignment(accel);
            snapPadOrientation(accel);
        }

        if (forceGyroOnly)
        {
            orientationFilter->update(gyro, dt);
        }
        else
        {
            // High-G switching logic (preserve base State behavior)
            double accelMag = accel.magnitude();
            double accelError = fabs(accelMag - 9.81);

            if (accelError < 1.0)
            {
                // Low acceleration - trust accelerometer for tilt correction
                orientationFilter->update(accel, gyro, dt);
            }
            else
            {
                // High-G or freefall - gyro-only mode
                orientationFilter->update(gyro, dt);
            }
        }

        // Keep State::orientation in board->earth for compatibility with existing logic.
        Quaternion boardToRocket = mountQuat_rb.conjugate();
        Quaternion qRocketToEarth = orientationFilter->getQuaternion();
        orientation = qRocketToEarth * boardToRocket;

        // Earth-frame acceleration from board-frame accel (Mahony applies board->body internally).
        Vector<3> earthAccel = orientationFilter->getEarthAcceleration(accel);
        acceleration.x() = earthAccel.x();
        acceleration.y() = earthAccel.y();
        acceleration.z() = earthAccel.z();
    }

    void RocketState::updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, const Vector<3> &mag, double dt)
    {
        if (!orientationFilter)
            return;

        if (!frameLocked && currentStage == PAD_IDLE)
        {
            updateMountingAlignment(accel);
            snapPadOrientation(accel);
        }

        if (forceGyroOnly)
        {
            orientationFilter->update(gyro, dt);
        }
        else
        {
            // High-G switching logic (preserve base State behavior)
            double accelMag = accel.magnitude();
            double accelError = fabs(accelMag - 9.81);

            if (accelError < 1.0)
            {
                // Low acceleration - trust accelerometer (and magnetometer) for correction
                orientationFilter->update(accel, gyro, mag, dt);
            }
            else
            {
                // High-G or freefall - gyro-only mode
                orientationFilter->update(gyro, dt);
            }
        }

        // Keep State::orientation in board->earth for compatibility with existing logic.
        Quaternion boardToRocket = mountQuat_rb.conjugate();
        Quaternion qRocketToEarth = orientationFilter->getQuaternion();
        orientation = qRocketToEarth * boardToRocket;

        // Earth-frame acceleration from board-frame accel (Mahony applies board->body internally).
        Vector<3> earthAccel = orientationFilter->getEarthAcceleration(accel);
        acceleration.x() = earthAccel.x();
        acceleration.y() = earthAccel.y();
        acceleration.z() = earthAccel.z();
    }

    int RocketState::update()
    {
        // Call parent implementation first - handles measurement update
        int success = State::update();

        // Update time in current stage
        unsigned long currentMillis = (unsigned long)(currentTimeSeconds * 1000.0);
        timeInCurrentStage = (currentMillis - stageStartTime) / 1000.0;

        // Calculate rocket-specific derived values that depend on orientation
        calculateTilt();
        detectFlightStage();

        return success;
    }

    void RocketState::predict(double dt)
    {
        if (currentStage == PAD_IDLE)
        {
            if (filter && !padFilterZeroed)
            {
                filter->initialize();
                padFilterZeroed = true;
            }

            position = Vector<3>(0, 0, 0);
            velocity = Vector<3>(0, 0, 0);
            return;
        }

        padFilterZeroed = false;
        State::predict(dt);
    }

    void RocketState::updateGPSMeasurement(const Vector<3> &gpsPos, const Vector<3> &gpsVel)
    {
        if (currentStage == PAD_IDLE)
            return;

        State::updateGPSMeasurement(gpsPos, gpsVel);
    }

    void RocketState::updateBaroMeasurement(double baroAlt)
    {
        if (currentStage == PAD_IDLE)
            return;

        State::updateBaroMeasurement(baroAlt);
    }

    void RocketState::predictState(double currentTimeSec)
    {
        if (currentTimeSec == -1)
            currentTimeSec = millis() / 1000.0;

        setCurrentTime(currentTimeSec);

        if (orientationFilter && orientationFilter->isReady())
        {
            Quaternion boardToRocket = mountQuat_rb.conjugate();
            Quaternion qRocketToEarth = orientationFilter->getQuaternion();
            orientation = qRocketToEarth * boardToRocket;
        }

        if (filter)
        {
            Matrix state = filter->getState();
            position.x() = state(0, 0);
            position.y() = state(1, 0);
            position.z() = state(2, 0);
            velocity.x() = state(3, 0);
            velocity.y() = state(4, 0);
            velocity.z() = state(5, 0);
        }

        if (orientationFilter)
        {
            Vector<3> earthAccel = orientationFilter->getEarthAcceleration(Vector<3>(0, 0, 0));
            acceleration.x() = earthAccel.x();
            acceleration.y() = earthAccel.y();
            acceleration.z() = earthAccel.z();
        }
    }

    RocketState &RocketState::withSensorManager(SensorManager *sm)
    {
        sensorManager = sm;
        return *this;
    }

    void RocketState::lockFrameForTest()
    {
        computeMountingQuaternion(currentUpAxis);
        frameLocked = true;
        forceGyroOnly = true;
    }

    void RocketState::calculateTilt()
    {
        // Calculate off-vertical angle using orientation quaternion
        // This is more accurate than using acceleration alone
        MahonyAHRS *ahrs = getOrientationFilter();

        // Updated to use isReady() (was isInitialized)
        if (ahrs && ahrs->isReady())
        {
            Quaternion q = getRocketOrientation();

            // Rocket body Z-axis in rocket frame (pointing along rocket, up in ENU)
            Vector<3> rocketZ(0, 0, 1);

            // Rotate rocket Z into earth frame to see which way rocket is pointing
            Vector<3> rocketDir = q.rotateVector(rocketZ);

            // Earth vertical reference (up in ENU is +Z)
            Vector<3> verticalRef(0, 0, 1);

            // Calculate angle between rocket direction and vertical
            double cosAngle = rocketDir.dot(verticalRef);
            offVerticalAngle = acos(fmax(-1.0, fmin(1.0, cosAngle))) * 180.0 / M_PI;
        }
    }

    void RocketState::updateMountingAlignment(const Vector<3> &accel)
    {
        const double accelMag = accel.magnitude();
        if (!isfinite(accelMag) || accelMag <= 1e-6)
        {
            return;
        }

        double bestDot = 0.0;
        UpAxis candidate = chooseUpAxis(accel, bestDot);

        if (candidate == currentUpAxis)
        {
            axisStableCount = 0;
            pendingUpAxis = currentUpAxis;
            return;
        }

        if (bestDot < AXIS_SWITCH_DOT_THRESHOLD)
        {
            axisStableCount = 0;
            pendingUpAxis = currentUpAxis;
            return;
        }

        if (candidate == pendingUpAxis)
        {
            axisStableCount++;
        }
        else
        {
            pendingUpAxis = candidate;
            axisStableCount = 1;
        }

        if (axisStableCount >= AXIS_SWITCH_STABLE_COUNT)
        {
            currentUpAxis = candidate;
            axisStableCount = 0;
            computeMountingQuaternion(currentUpAxis);
        }
    }

    void RocketState::snapPadOrientation(const Vector<3> &accel)
    {
        if (!orientationFilter)
        {
            return;
        }

        Vector<3> boardUp = accel;
        boardUp.normalize();
        if (boardUp.magnitude() <= 1e-6)
        {
            return;
        }

        const Quaternion boardToRocket = mountQuat_rb.conjugate();
        Vector<3> rocketUp = boardToRocket.rotateVector(boardUp);
        rocketUp.normalize();

        Vector<3> earthUp(0, 0, 1);
        double dot = rocketUp.dot(earthUp);
        dot = fmax(-1.0, fmin(1.0, dot));

        Quaternion qTilt;
        if (dot < -0.999999)
        {
            Vector<3> axis = Vector<3>(1, 0, 0).cross(rocketUp);
            if (axis.magnitude() <= 1e-6)
            {
                axis = Vector<3>(0, 1, 0).cross(rocketUp);
            }
            axis.normalize();
            qTilt.fromAxisAngle(axis, M_PI);
        }
        else if (dot < 0.999999)
        {
            Vector<3> axis = rocketUp.cross(earthUp);
            axis.normalize();
            qTilt.fromAxisAngle(axis, acos(dot));
        }

        qTilt.normalize();

        Quaternion qDesired = qTilt;
        Quaternion qCurrent = orientationFilter->getQuaternion();
        Vector<3> currentForward = qCurrent.rotateVector(Vector<3>(1, 0, 0));
        Vector<3> targetForward = qTilt.rotateVector(Vector<3>(1, 0, 0));

        currentForward = currentForward - earthUp * currentForward.dot(earthUp);
        targetForward = targetForward - earthUp * targetForward.dot(earthUp);

        if (currentForward.magnitude() > 1e-6 && targetForward.magnitude() > 1e-6)
        {
            currentForward.normalize();
            targetForward.normalize();

            double yawDot = fmax(-1.0, fmin(1.0, targetForward.dot(currentForward)));
            double yawAngle = atan2(targetForward.cross(currentForward).dot(earthUp), yawDot);

            Quaternion qYaw;
            qYaw.fromAxisAngle(earthUp, yawAngle);
            qYaw.normalize();
            qDesired = qYaw * qTilt;
            qDesired.normalize();
        }

        orientationFilter->setQuaternion(qDesired);
    }

    RocketState::UpAxis RocketState::chooseUpAxis(const Vector<3> &accel, double &bestDot) const
    {
        Vector<3> boardUp = accel;
        boardUp.normalize();

        struct AxisDot
        {
            UpAxis axis;
            double dot;
        };

        AxisDot candidates[6] = {
            {POS_X, boardUp.x()},
            {NEG_X, -boardUp.x()},
            {POS_Y, boardUp.y()},
            {NEG_Y, -boardUp.y()},
            {POS_Z, boardUp.z()},
            {NEG_Z, -boardUp.z()}};

        UpAxis bestAxis = candidates[0].axis;
        bestDot = candidates[0].dot;
        for (int i = 1; i < 6; i++)
        {
            if (candidates[i].dot > bestDot)
            {
                bestDot = candidates[i].dot;
                bestAxis = candidates[i].axis;
            }
        }

        return bestAxis;
    }

    void RocketState::computeMountingQuaternion(UpAxis axis)
    {
        Vector<3> bodyZ;
        switch (axis)
        {
        case POS_X:
            bodyZ = Vector<3>(1, 0, 0);
            break;
        case NEG_X:
            bodyZ = Vector<3>(-1, 0, 0);
            break;
        case POS_Y:
            bodyZ = Vector<3>(0, 1, 0);
            break;
        case NEG_Y:
            bodyZ = Vector<3>(0, -1, 0);
            break;
        case POS_Z:
            bodyZ = Vector<3>(0, 0, 1);
            break;
        case NEG_Z:
            bodyZ = Vector<3>(0, 0, -1);
            break;
        }

        Vector<3> bodyX(1, 0, 0);
        // If body X is the up axis, use body Y as rocket forward
        if (fabs(bodyZ.dot(bodyX)) > 0.9)
        {
            bodyX = Vector<3>(0, 1, 0);
        }

        Vector<3> bodyY = bodyZ.cross(bodyX);
        bodyY.normalize();
        bodyX = bodyY.cross(bodyZ);
        bodyX.normalize();

        Matrix m(3, 3);
        m(0, 0) = bodyX.x();
        m(0, 1) = bodyY.x();
        m(0, 2) = bodyZ.x();
        m(1, 0) = bodyX.y();
        m(1, 1) = bodyY.y();
        m(1, 2) = bodyZ.y();
        m(2, 0) = bodyX.z();
        m(2, 1) = bodyY.z();
        m(2, 2) = bodyZ.z();

        Quaternion q;
        q.fromMatrix(m);
        q.normalize();
        mountQuat_rb = q;

        if (orientationFilter)
        {
            // Mahony expects board->body; mountQuat_rb is rocket->board.
            orientationFilter->setBoardToBodyQuaternion(mountQuat_rb.conjugate(), true);
            Quaternion boardToRocket = mountQuat_rb.conjugate();
            Quaternion qRocketToEarth = orientationFilter->getQuaternion();
            orientation = qRocketToEarth * boardToRocket;
        }
    }

    Quaternion RocketState::getRocketOrientation() const
    {
        return orientation * mountQuat_rb;
    }

    double RocketState::liftoffAccelThresholdMs2() const
    {
        const double g = (flightConfig ? flightConfig->getLiftoffAccelThreshold() : DEFAULT_LIFTOFF_ACCEL_THRESHOLD_G);
        return g * 9.81;
    }

    unsigned long RocketState::liftoffDurationMs() const
    {
        return flightConfig ? flightConfig->getLiftoffDetectDuration() : DEFAULT_LIFTOFF_DURATION_MS;
    }

    double RocketState::burnoutAccelThresholdMs2() const
    {
        const double g = (flightConfig ? flightConfig->getBurnoutAccelThreshold() : DEFAULT_BURNOUT_ACCEL_THRESHOLD_G);
        return g * 9.81;
    }

    unsigned long RocketState::burnoutDurationMs() const
    {
        // Burnout duration is not yet exposed in AstraRocketConfig.
        return DEFAULT_BURNOUT_DURATION_MS;
    }

    double RocketState::apogeeVelocityThresholdMs() const
    {
        return flightConfig ? flightConfig->getApogeeVelocityThreshold() : DEFAULT_APOGEE_VELOCITY_THRESHOLD;
    }

    double RocketState::landingVelocityThresholdMs() const
    {
        return flightConfig ? flightConfig->getLandingVelocityThreshold() : DEFAULT_LANDING_VELOCITY_THRESHOLD;
    }

    unsigned long RocketState::landingDurationMs() const
    {
        return flightConfig ? flightConfig->getLandingDetectDuration() : DEFAULT_LANDING_DURATION_MS;
    }

    void RocketState::detectFlightStage()
    {
        unsigned long now = (unsigned long)(currentTimeSeconds * 1000.0);
        FlightStage newStage = currentStage;
        const double liftoffAccelThresh = liftoffAccelThresholdMs2();
        const unsigned long liftoffDuration = liftoffDurationMs();
        const double burnoutAccelThresh = burnoutAccelThresholdMs2();
        const unsigned long burnoutDuration = burnoutDurationMs();
        const double apogeeVelThresh = apogeeVelocityThresholdMs();
        const double landingVelThresh = landingVelocityThresholdMs();
        const unsigned long landingDuration = landingDurationMs();

        switch (currentStage)
        {
        case PAD_IDLE:
        {
            // Detect liftoff: sustained high vertical acceleration from KF state
            double accelMag = acceleration.magnitude();
            // Optional debug spam reduction
            // fprintf(stderr, "DEBUG RocketState: PAD_IDLE - accelMag=%0.3f\n", accelMag);

            if (accelMag > liftoffAccelThresh)
            {
                if (!highAccelDetected)
                {
                    highAccelStartTime = now;
                    highAccelDetected = true;
                    fprintf(stderr, "DEBUG RocketState: High accel detected! Starting timer...\n");
                }
                else if (now - highAccelStartTime > liftoffDuration)
                {
                    timeOfLaunch = currentTimeSeconds;
                    newStage = BOOST;
                    LOGI("LIFTOFF DETECTED! Vertical accel: %0.2f G", acceleration.z());

                    if (!frameLocked)
                    {
                        computeMountingQuaternion(currentUpAxis);
                        frameLocked = true;
                        forceGyroOnly = true;
                        LOGI("Orientation frame locked at launch (gyro-only).");
                    }
                }
            }
            else
            {
                highAccelDetected = false;
            }
            break;
        }

        case BOOST:
            // Detect motor burnout once signed earth-frame vertical acceleration
            // drops to or below the configured threshold.
            {
                const double verticalAccel = acceleration.z();

                if (verticalAccel <= burnoutAccelThresh)
                {
                    if (!lowAccelDetected)
                    {
                        lowAccelStartTime = now;
                        lowAccelDetected = true;
                    }
                    else if (now - lowAccelStartTime > burnoutDuration)
                    {
                        newStage = COAST;
                        LOGI("MOTOR BURNOUT DETECTED at %0.2f m AGL, vertical accel: %0.2f m/s^2",
                             position.z(), verticalAccel);
                    }
                }
                else
                {
                    lowAccelDetected = false;
                }
            }
            break;

        case COAST:
            // Detect apogee once descent is clearly underway.
            if (velocity.z() <= -apogeeVelThresh)
            {
                newStage = APOGEE;
                LOGI("APOGEE DETECTED at %0.2f m AGL", position.z());
            }
            else if (timeInCurrentStage > 120 && velocity.z() < apogeeVelThresh)
            {
                // Safety fallback: if state estimator is degraded, avoid getting
                // stuck in COAST forever, but do not trigger while strongly climbing.
                newStage = APOGEE;
                LOGW("APOGEE FALLBACK after long coast at %0.2f m AGL, vz=%0.2f m/s",
                     position.z(), velocity.z());
            }
            break;

        case APOGEE:
            // Transition to expecting drogue - starting descent
            if (velocity.z() < -2.0)
            { // Descending at > 2 m/s
                newStage = EXPECTING_DROGUE;
                LOGI("EXPECTING DROGUE DEPLOYMENT at %0.2f m AGL, descent rate: %0.2f m/s",
                     position.z(), -velocity.z());
            }
            break;

        case EXPECTING_DROGUE:
            // Detect drogue deployment by sustained descent rate in valid range
            if (velocity.z() < -DROGUE_DESCENT_MIN && velocity.z() > -DROGUE_DESCENT_MAX && timeInCurrentStage > 5.0)
            {
                if (!drogueRateDetected)
                {
                    drogueDetectStartTime = now;
                    drogueRateDetected = true;
                }
                else if (now - drogueDetectStartTime > DROGUE_DETECT_DURATION)
                {
                    newStage = UNDER_DROGUE;
                    LOGI("UNDER DROGUE detected at %0.2f m AGL, descent rate: %0.2f m/s",
                         position.z(), -velocity.z());
                }
            }
            else
            {
                drogueRateDetected = false;
            }
            break;

        case UNDER_DROGUE:
            // Check if we're below the main deployment altitude threshold
            if (position.z() < MAIN_DEPLOY_ALTITUDE)
            {
                newStage = EXPECTING_MAIN;
                LOGI("EXPECTING MAIN DEPLOYMENT at %0.2f m AGL", position.z());
            }
            break;

        case EXPECTING_MAIN:
            // Detect main deployment by sustained descent rate in valid range
            if (velocity.z() < -MAIN_DESCENT_MIN && velocity.z() > -MAIN_DESCENT_MAX)
            {
                if (!mainRateDetected)
                {
                    mainDetectStartTime = now;
                    mainRateDetected = true;
                }
                else if (now - mainDetectStartTime > MAIN_DETECT_DURATION)
                {
                    newStage = UNDER_MAIN;
                    LOGI("UNDER MAIN detected at %0.2f m AGL, descent rate: %0.2f m/s",
                         position.z(), -velocity.z());
                }
            }
            else
            {
                mainRateDetected = false;
            }
            break;

        case UNDER_MAIN:
            // Detect landing: low velocity sustained
            if (fabs(velocity.z()) < landingVelThresh)
            {
                if (!lowVelocityDetected)
                {
                    lowVelocityStartTime = now;
                    lowVelocityDetected = true;
                }
                else if (now - lowVelocityStartTime > landingDuration)
                {
                    newStage = LANDED;
                    LOGI("LANDING DETECTED at %0.2f m AGL", position.z());
                }
            }
            else
            {
                lowVelocityDetected = false;
            }
            break;

        case LANDED:
            // Terminal state
            break;
        }

        // Update stage if changed
        if (newStage != currentStage)
        {
            setFlightStage(newStage);
        }
    }

} // namespace astra_rocket
