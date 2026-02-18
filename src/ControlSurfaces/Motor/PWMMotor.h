#ifndef MOTOR_H
#define MOTOR_H

#include "ControlSurfaces/ControlSurface.h"
#include <Arduino.h>

namespace astra
{
    /**
     * Configuration for motor control
     */
    struct PWMMotorConfig : public ControlSurfaceConfig
    {
        int pwmPin = -1;         // PWM output pin for speed control
        int dirPin = -1;         // Direction pin (optional, for bidirectional)
        int pwmFrequency = 1000; // PWM frequency in Hz
        int pwmResolution = 8;   // PWM resolution in bits (8 = 0-255)
        bool invert = false;
        float deadband = 0.0f; // Deadband near zero (0.0 to 0.1 typical)
    };

    /**
     * Motor - PWM-based motor control
     *
     * Controls DC motors with PWM speed control and optional direction control.
     */
    class PWMMotor : public ControlSurface
    {
    public:
        PWMMotor(const char *name = "Motor");
        virtual ~PWMMotor() = default;

        // ControlSurface interface implementation
        bool setNormalizedPosition(float normalized) override;
        float getNormalizedPosition() const override;
        bool setRawPosition(float raw) override;
        float getRawPosition() const override;
        bool zero() override;
        bool isWithinLimits() const override;
        bool EStop() override;

        /**
         * Convenience methods for motor control
         */
        bool setSpeed(float speed); // Alias for setNormalizedPosition
        float getSpeed() const;     // Alias for getNormalizedPosition

    protected:
        int init(const ControlSurfaceConfig *config) override;

        int pwmPin;
        int dirPin;
        int pwmMax; // Maximum PWM value (2^pwmResolution - 1)
        bool invertDirection;
        float deadband;

        float currentPosition; // Current normalized position
        int currentPWM;        // Current PWM value

        /**
         * Apply deadband to input value
         */
        float applyDeadband(float value) const;

        /**
         * Set motor direction (bidirectional mode only)
         */
        void setDirection(bool forward);
    };

} // namespace astra

#endif // MOTOR_H
