#ifndef SERVO_H
#define SERVO_H

#include "ControlSurfaces/ControlSurface.h"
#include <Arduino.h>
#include <Servo.h>

namespace astra
{
    /**
     * Configuration for servo control
     */
    struct ServoConfig : public ControlSurfaceConfig
    {
        int pin = -1;              // PWM pin for servo signal
        int minPulse = 1000;       // Minimum pulse width in microseconds (default 1000)
        int maxPulse = 2000;       // Maximum pulse width in microseconds (default 2000)
        float minAngle = 0.0f;     // Minimum angle in degrees
        float maxAngle = 180.0f;   // Maximum angle in degrees
        float neutralAngle = 90.0f; // Neutral/zero position angle
    };

    /**
     * Servo - Standard PWM servo control
     * 
     * Controls hobby servos using PWM signals. Supports angle-based control.
     * Normalized position: 0.0 = minAngle, 1.0 = maxAngle
     * Raw position: angle in degrees
     * 
     */
    class Servo : public ControlSurface
    {
    public:
        Servo(const char* name = "Servo");
        virtual ~Servo();

        // ControlSurface interface implementation
        bool setNormalizedPosition(float normalized) override;
        float getNormalizedPosition() const override;
        bool setRawPosition(float raw) override;
        float getRawPosition() const override;
        bool zero() override;
        bool isWithinLimits() const override;
        bool EStop() override;

        /**
         * Convenience methods for servo control
         */
        bool setAngle(float degrees);  // Set angle in degrees
        float getAngle() const;        // Get current angle in degrees
        bool detach();                 // Detach servo (stops sending pulses)
        bool attach();                 // Re-attach servo

    protected:
        int init(const ControlSurfaceConfig* config);

        ::Servo servo;  // Arduino Servo object
        int pin;
        int minPulse;
        int maxPulse;
        float minAngle;
        float maxAngle;
        float neutralAngle;
        float currentAngle;
        bool attached;
    };

} // namespace astra

#endif // SERVO_H
