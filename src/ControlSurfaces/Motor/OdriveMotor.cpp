#include "OdriveMotor.h"
namespace astra
{

    OdriveMotor::OdriveMotor(const char *name)
        : ControlSurface(name),
          serial(nullptr),
          odrive(nullptr),
          baudrate(115200),
          topLimitSwitchPin(1),
          minPosition(0.0f),
          maxPosition(26.0f),
          minAngle(0.0f),
          maxAngle(80.0f),
          invert(-1.0f),
          useClosedLoop(true),
          idleOnEStop(false),
          initTimeoutMs(10000),
          stateRetryDelayMs(1000),
          zeroVelocity(1.0f),
          stallSampleCount(10),
          stallPositionEpsilon(0.0001f),
          feedback(),
          positionHistory(1),
          position(0.0f),
          angle(0.0f),
          velocity(0.0f),
          initPositionOffset(0.0f),
          targetPosition(0.0f),
          targetVelocity(0.0f),
          voltage(0.0f),
          encoderPosition(0.0f),
          stalledState(OdriveStalledState::STOPPED),
          stalledStateTelemetry(static_cast<int>(OdriveStalledState::STOPPED))
    {
        addColumn("%0.3f", &position, "ODrive Position (Native Units)");
        addColumn("%0.3f", &velocity, "ODrive Velocity (Native Units)");
        addColumn("%0.2f", &voltage, "ODrive Voltage");
        addColumn("%d", &stalledStateTelemetry, "odrive stall state");
    }

    int OdriveMotor::init(const ControlSurfaceConfig *config)
    {
        const OdriveMotorConfig *odriveConfig = static_cast<const OdriveMotorConfig *>(config);

        if (!odriveConfig || !odriveConfig->serial)
        {
            LOGE("%s: Invalid configuration", getName());
            return -1;
        }

        serial = odriveConfig->serial;
        baudrate = odriveConfig->baudrate;
        topLimitSwitchPin = odriveConfig->topLimitSwitchPin;

        minPosition = odriveConfig->minPosition;
        maxPosition = odriveConfig->maxPosition;
        minAngle = odriveConfig->minAngle;
        maxAngle = odriveConfig->maxAngle;

        invert = odriveConfig->invertDirection ? -1.0f : 1.0f;
        useClosedLoop = odriveConfig->useClosedLoop;
        idleOnEStop = odriveConfig->idleOnEStop;

        initTimeoutMs = odriveConfig->initTimeoutMs;
        stateRetryDelayMs = odriveConfig->stateRetryDelayMs;
        zeroVelocity = odriveConfig->zeroVelocity;

        stallSampleCount = odriveConfig->stallSampleCount;
        stallPositionEpsilon = odriveConfig->stallPositionEpsilon;

        if(odriveConfig->usingEncoder){
            addColumn("%0.2f", &encoderPosition, "odrive_encoder_position");
        }

        if (stallSampleCount <= 0)
        {
            stallSampleCount = 1;
        }

        positionHistory = CircBuffer<float>(stallSampleCount);

        if (odriveConfig->odrive)
        {
            odrive = odriveConfig->odrive;
        }
        else
        {
            static ODriveUART defaultOdrive(*serial);
            odrive = &defaultOdrive;
        }
        serial->begin(baudrate);
        delay(1000);

        unsigned long startTime = millis();
        while (odrive->getState() == AXIS_STATE_UNDEFINED)
        {
            if (millis() - startTime > initTimeoutMs)
            {
                LOGE("%s: ODrive init timeout", getName());
                return -1;
            }
            delay(100);
        }

        voltage = odrive->getParameterAsFloat("vbus_voltage");

        if (useClosedLoop)
        {
            while (odrive->getState() != AXIS_STATE_CLOSED_LOOP_CONTROL)
            {
                if (millis() - startTime > initTimeoutMs)
                {
                    LOGE("%s: Closed loop enable timeout", getName());
                    return -1;
                }
                odrive->clearErrors();
                odrive->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
                delay(stateRetryDelayMs);
            }
        }

        if (topLimitSwitchPin >= 0)
        {
            pinMode(topLimitSwitchPin, INPUT_PULLUP);
        }
        return 0;
    }
 
    bool OdriveMotor::setNormalizedPosition(float normalized)
    {
        if (!initialized)
        {
            LOGE("%s: Not initialized", getName());
            return false;
        }

        if (normalized < -1.0f)
            normalized = -1.0f;
        if (normalized > 1.0f)
            normalized = 1.0f;

        float pos = minPosition + (normalized * (maxPosition - minPosition));
        return setPosition(pos);
    }

    float OdriveMotor::getNormalizedPosition() const
    {
        float range = maxPosition - minPosition;
        if (range == 0.0f)
        {
            return 0.0f;
        }
        return (position - minPosition) / range;
    }

    bool OdriveMotor::setRawPosition(float raw)
    {
        return setPosition(raw);
    }

    float OdriveMotor::getRawPosition() const
    {
        return position;
    }

    bool OdriveMotor::zero()
    {
        return zeroMotor();
    }

    bool OdriveMotor::isWithinLimits() const
    {
        return position >= minPosition && position <= maxPosition;
    }

    bool OdriveMotor::EStop()
    {
        if (!odrive)
        {
            return false;
        }
        odrive->setPosition(0.0f);
        odrive->setVelocity(0.0f);
        odrive->setTorque(0.0f);
        if (idleOnEStop)
        {
            odrive->setState(AXIS_STATE_IDLE);
        }
        return true;
    }

    void OdriveMotor::updateSensors()
    {
        if (!odrive)
        {
            return;
        }

        feedback = odrive->getFeedback();
        position = initPositionOffset - (invert * feedback.pos);
        velocity = invert * feedback.vel;

        voltage = odrive->getParameterAsFloat("vbus_voltage");
        angle = posToAngle(position);

        positionHistory.push(position);
    }

    float OdriveMotor::getPosition() const
    {
        return position;
    }

    float OdriveMotor::getVelocity() const
    {
        return velocity;
    }

    float OdriveMotor::getTorque() const {
        return targetTorque;
    }

    float OdriveMotor::getAngle() const
    {
        return angle;
    }
    
    void OdriveMotor::setControl(MotorControlMode mode, float val){
        switch(mode){
            case MotorControlMode::POSITION:
                setPosition(val);
                break;
            case MotorControlMode::VELOCITY:
                setVelocity(val);
                break;
            case MotorControlMode::TORQUE:
                setTorque(val);
                break;
        }
    }


    bool OdriveMotor::setPosition(float pos)
    {
        if (!odrive)
        {
            return false;
        }

        if (pos < minPosition || pos > maxPosition)
        {
            LOGI("%s: Position out of bounds", getName());
            return false;
        }
        
        targetPosition = initPositionOffset - (invert * pos);

        if (!isStalled())
        {
            odrive->setPosition(targetPosition);
            return true;
        }

        if ((stalledState == OdriveStalledState::TOP || stalledState == OdriveStalledState::STOPPED) && pos > position)
        {
            odrive->setPosition(targetPosition);
            return true;
        }

        if (stalledState == OdriveStalledState::BOTTOM && pos < position)
        {
            odrive->setPosition(targetPosition);
            return true;
        }

        LOGI("%s: Stalled, cannot move", getName());
        LOGI("%s: Stalled State: %d", getName(), stalledStateTelemetry);
        LOGI("%s: Current Position: %0.3f", getName(), position);
        LOGI("%s: Target Position: %0.3f", getName(), pos);
        return false;
    }

    bool OdriveMotor::setVelocity(float vel)
    {
        if (!odrive)
        {
            return false;
        }

        if (!isStalled())
        {
            targetVelocity = vel;
            odrive->setVelocity(invert * vel);
            return true;
        }

        if (stalledState == OdriveStalledState::TOP && vel < 0.0f)
        {
            targetVelocity = vel;
            odrive->setVelocity(invert * vel);
            return true;
        }

        if (stalledState == OdriveStalledState::BOTTOM && vel > 0.0f)
        {
            targetVelocity = vel;
            odrive->setVelocity(invert * vel);
            return true;
        }

        targetVelocity = 0.0f;
        odrive->setVelocity(0.0f);
        return false;
    }

    bool OdriveMotor::setTorque(float torque){
        if(!odrive){
            return false;
        }

        if(!isStalled()){
            targetTorque = torque;
            odrive->setTorque(invert * torque);
            return true;
        }
        if (stalledState == OdriveStalledState::TOP && torque < 0.0f)
        {
            targetTorque = torque;
            odrive->setTorque(invert * torque);
            return true;
        }

        if (stalledState == OdriveStalledState::BOTTOM && torque > 0.0f)
        {
            targetTorque = torque;
            odrive->setTorque(invert * torque);
            return true;
        }

        targetTorque = 0.0f;
        odrive->setTorque(0.0f);
        return false;
        
    }

    float OdriveMotor::angleToPos(float angle) const
    {
        if (maxAngle == minAngle)
        {
            return minPosition;
        }

        float clampedAngle = angle;
        if (clampedAngle < minAngle)
            clampedAngle = minAngle;
        if (clampedAngle > maxAngle)
            clampedAngle = maxAngle;

        float ratio = (clampedAngle - minAngle) / (maxAngle - minAngle);

        return minPosition + ratio * (maxPosition - minPosition);
    }

    float OdriveMotor::posToAngle(float pos) const
    {
        if (maxPosition == minPosition)
        {
            return minAngle;
        }

        float clampedPos = pos;
        if (clampedPos < minPosition)
            clampedPos = minPosition;
        if (clampedPos > maxPosition)
            clampedPos = maxPosition;

        float ratio = (clampedPos - minPosition) / (maxPosition - minPosition);
        return minAngle + ratio * (maxAngle - minAngle);
    }

    bool OdriveMotor::isStalled()
    {
        if (topLimitSwitchPin >= 0)
        {
            bool topLimitSwitchState = digitalRead(topLimitSwitchPin) == LOW;
            if (topLimitSwitchState)
            {
                stalledState = OdriveStalledState::TOP;
                stalledStateTelemetry = static_cast<int>(stalledState);
                return true;
            }
        }

        if (positionHistory.getCount() < stallSampleCount)
        {
            stalledState = OdriveStalledState::MOVING;
            stalledStateTelemetry = static_cast<int>(stalledState);
            return false;
        }

        bool stalled = true;
        for (int i = 0; i < stallSampleCount; i++)
        {
            if (std::fabs(positionHistory[i] - position) > stallPositionEpsilon)
            {
                stalled = false;
                break;
            }
        }

        if (!stalled)
        {
            stalledState = OdriveStalledState::MOVING;
            stalledStateTelemetry = static_cast<int>(stalledState);
            return false;
        }

        if (position <= minPosition + stallPositionEpsilon)
        {
            stalledState = OdriveStalledState::BOTTOM;
        }
        else
        {
            stalledState = OdriveStalledState::STOPPED;
        }

        stalledStateTelemetry = static_cast<int>(stalledState);
        return true;
    }

    bool OdriveMotor::zeroMotor()
    {
        if (!initialized)
        {
            LOGE("%s: Not initialized, cannot zero", getName());
            return false;
        }

        if (!odrive)
        {
            return false;
        }
        odrive->setVelocity(0.0f);

        while (!isStalled())
        {
            odrive->setVelocity(invert * zeroVelocity);
            delay(100);
        }

        odrive->setVelocity(0.0f);
        delay(500);

        initPositionOffset = -getPosition();
        return true;
    }

} // namespace astra
