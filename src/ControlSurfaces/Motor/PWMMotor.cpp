#include "PWMMotor.h"

namespace astra
{

    PWMMotor::PWMMotor(const char *name) : ControlSurface(name)
    {
        pwmPin = -1;
        dirPin = -1;
        pwmMax = 255;
        invertDirection = false;
        deadband = 0.0f;
        currentPosition = 0.0f;
        currentPWM = 0;

        // Register telemetry columns
        addColumn("%.3f", &currentPosition, "position");
        addColumn("%d", &currentPWM, "pwm");
    }

    int PWMMotor::init(const ControlSurfaceConfig *config)
    {
        const PWMMotorConfig *motorConfig = static_cast<const PWMMotorConfig *>(config);

        if (!motorConfig || motorConfig->pwmPin < 0)
        {
            LOGE("%s: Invalid configuration", getName());
            return -1;
        }

        pwmPin = motorConfig->pwmPin;
        dirPin = motorConfig->dirPin;
        invertDirection = motorConfig->invert;
        deadband = motorConfig->deadband;

        // Calculate max PWM value from resolution
        pwmMax = (1 << motorConfig->pwmResolution) - 1;

        // Configure PWM pin
        pinMode(pwmPin, OUTPUT);

#ifdef ESP32
        // ESP32 uses ledcSetup for PWM
        ledcSetup(0, motorConfig->pwmFrequency, motorConfig->pwmResolution);
        ledcAttachPin(pwmPin, 0);
#elif defined(ARDUINO_ARCH_STM32)
        // STM32 may need analogWriteFrequency
        analogWriteFrequency(motorConfig->pwmFrequency);
#endif

        // Initialize to stopped state
        EStop();

        LOGI("%s: Initialized on pin %d", getName(), pwmPin);
        return 0;
    }

    bool PWMMotor::setNormalizedPosition(float normalized)
    {
        // Apply limits based on mode

        if (normalized < -1.0f)
            normalized = -1.0f;
        if (normalized > 1.0f)
            normalized = 1.0f;

        // Apply deadband
        float adjusted = applyDeadband(normalized);

        currentPosition = adjusted;

        // Convert to PWM value
        bool forward = (adjusted >= 0.0f);
        setDirection(forward);

        currentPWM = (int)(abs(adjusted) * pwmMax);
        analogWrite(pwmPin, currentPWM);

        return true;
    }

    float PWMMotor::getNormalizedPosition() const
    {
        return currentPosition;
    }

    bool PWMMotor::setRawPosition(float raw)
    {
        // Convert raw PWM value to normalized
        float normalized;

        normalized = raw / (float)pwmMax;

        return setNormalizedPosition(normalized);
    }

    float PWMMotor::getRawPosition() const
    {
        return currentPWM;
    }

    bool PWMMotor::zero()
    {
        return setNormalizedPosition(0.0f);
    }

    bool PWMMotor::isWithinLimits() const
    {

        return currentPosition >= -1.0f && currentPosition <= 1.0f;
    }

    bool PWMMotor::EStop()
    {
        currentPosition = 0.0f;
        currentPWM = 0;
        analogWrite(pwmPin, 0);

        if (dirPin >= 0)
        {
            digitalWrite(dirPin, LOW);
        }

        return true;
    }

    bool PWMMotor::setSpeed(float speed)
    {
        return setNormalizedPosition(speed);
    }

    float PWMMotor::getSpeed() const
    {
        return getNormalizedPosition();
    }

    float PWMMotor::applyDeadband(float value) const
    {
        if (deadband <= 0.0f)
        {
            return value;
        }

        if (abs(value) < deadband)
        {
            return 0.0f;
        }

        // Scale value outside deadband
        if (value > 0.0f)
        {
            return (value - deadband) / (1.0f - deadband);
        }
        else
        {
            return (value + deadband) / (1.0f - deadband);
        }
    }

    void PWMMotor::setDirection(bool forward)
    {
        if (dirPin < 0)
            return;

        bool pinState = forward;
        if (invertDirection)
            pinState = !pinState;

        digitalWrite(dirPin, pinState ? HIGH : LOW);
    }

} // namespace astra
