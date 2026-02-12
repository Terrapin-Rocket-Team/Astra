#include "Servo.h"

namespace astra
{

Servo::Servo(const char* name) : ControlSurface(name)
{
    pin = -1;
    minPulse = 1000;
    maxPulse = 2000;
    minAngle = 0.0f;
    maxAngle = 180.0f;
    neutralAngle = 90.0f;
    currentAngle = 90.0f;
    attached = false;
    
    // Register telemetry columns
    addColumn("%.2f", &currentAngle, "angle_deg");
}

Servo::~Servo()
{
    if (attached)
    {
        servo.detach();
    }
}

int Servo::init(const ControlSurfaceConfig* config)
{
    const ServoConfig* servoConfig = static_cast<const ServoConfig*>(config);
    
    if (!servoConfig || servoConfig->pin < 0)
    {
        LOGE("%s: Invalid configuration", getName());
        return -1;
    }

    pin = servoConfig->pin;
    minPulse = servoConfig->minPulse;
    maxPulse = servoConfig->maxPulse;
    minAngle = servoConfig->minAngle;
    maxAngle = servoConfig->maxAngle;
    neutralAngle = servoConfig->neutralAngle;
    currentAngle = neutralAngle;

    // Attach servo
    servo.attach(pin, minPulse, maxPulse);
    attached = true;

    // Set to neutral position
    setAngle(neutralAngle);

    LOGI("%s: Initialized on pin %d (%.1f° to %.1f°)", 
         getName(), pin, minAngle, maxAngle);
    return 0;
}

bool Servo::setNormalizedPosition(float normalized)
{
    // Clamp to valid range
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    // Convert to angle
    float angle = minAngle + (normalized * (maxAngle - minAngle));
    return setAngle(angle);
}

float Servo::getNormalizedPosition() const
{
    if (maxAngle == minAngle) return 0.0f;
    return (currentAngle - minAngle) / (maxAngle - minAngle);
}

bool Servo::setRawPosition(float raw)
{
    return setAngle(raw);
}

float Servo::getRawPosition() const
{
    return currentAngle;
}

bool Servo::zero()
{
    return setAngle(neutralAngle);
}

bool Servo::isWithinLimits() const
{
    return currentAngle >= minAngle && currentAngle <= maxAngle;
}

bool Servo::EStop()
{
    // For servos, "stop" means go to neutral and detach
    setAngle(neutralAngle);
    delay(100);  // Give time to reach position
    return detach();
}

bool Servo::setAngle(float degrees)
{
    // Clamp to configured range
    if (degrees < minAngle) degrees = minAngle;
    if (degrees > maxAngle) degrees = maxAngle;

    currentAngle = degrees;

    if (attached)
    {
        servo.write((int)degrees);
        return true;
    }
    
    return false;
}

float Servo::getAngle() const
{
    return currentAngle;
}

bool Servo::detach()
{
    if (attached)
    {
        servo.detach();
        attached = false;
        LOGD("%s: Detached", getName());
    }
    return true;
}

bool Servo::attach()
{
    if (!attached && pin >= 0)
    {
        servo.attach(pin, minPulse, maxPulse);
        attached = true;
        setAngle(currentAngle);  // Restore position
        LOGD("%s: Attached", getName());
        return true;
    }
    return false;
}

} // namespace astra
