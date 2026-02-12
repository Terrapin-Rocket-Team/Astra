#include "PyroCharge.h"

namespace astra
{

PyroCharge::PyroCharge(const char* name) : Relay(name)
{
    continuityPin = -1;
    maxFireDuration = 1000;
    minFireDuration = 50;
    continuityCurrent = 0.0f;
    requireContinuity = true;
    oneShot = true;
    
    pyroState = PyroState::SAFED;
    continuityGood = false;
    fireStartTime = 0;
    fireDuration = 0;
    
    // Add additional telemetry columns
    addColumn("%d", &pyroState, "pyro_state");
    addColumn("%d", &continuityGood, "continuity");
}

int PyroCharge::init(const ControlSurfaceConfig* config)
{
    const PyroChargeConfig* pyroConfig = static_cast<const PyroChargeConfig*>(config);
    
    if (!pyroConfig)
    {
        LOGE("%s: Invalid configuration", getName());
        return -1;
    }

    // Initialize base relay
    if (!Relay::init(config))
    {
        return -1;
    }

    continuityPin = pyroConfig->continuityPin;
    maxFireDuration = pyroConfig->maxFireDuration;
    minFireDuration = pyroConfig->minFireDuration;
    continuityCurrent = pyroConfig->continuityCurrent;
    requireContinuity = pyroConfig->requireContinuity;
    oneShot = pyroConfig->oneShot;

    // Configure continuity pin if provided
    if (continuityPin >= 0)
    {
        pinMode(continuityPin, INPUT);
    }

    // Ensure we start in safed state
    doSafe();
    
    LOGI("%s: Initialized (pyro mode, max fire: %dms)", getName(), maxFireDuration);
    return 0;
}

bool PyroCharge::setNormalizedPosition(float normalized)
{
    // For pyro charges, setting position to 1.0 means fire
    // This is only allowed if armed
    if (normalized >= 0.5f)
    {
        return fire(500);  // Default 500ms fire duration
    }
    else
    {
        return safe();
    }
}

bool PyroCharge::zero()
{
    return safe();
}

bool PyroCharge::EStop()
{
    doSafe();
    return true;
}

bool PyroCharge::arm()
{
    // Cannot arm if already fired (one-shot mode)
    if (oneShot && pyroState == PyroState::FIRED)
    {
        LOGW("%s: Cannot arm - already fired", getName());
        return false;
    }

    // Check continuity if required
    if (requireContinuity)
    {
        if (!checkContinuity())
        {
            LOGW("%s: Cannot arm - no continuity", getName());
            pyroState = PyroState::CONTINUITY_FAIL;
            return false;
        }
    }

    pyroState = PyroState::ARMED;
    LOGI("%s: ARMED", getName());
    return true;
}

bool PyroCharge::safe()
{
    doSafe();
    return true;
}

bool PyroCharge::fire(int durationMs)
{
    // Safety checks
    if (pyroState != PyroState::ARMED)
    {
        LOGE("%s: Cannot fire - not armed (state: %d)", getName(), (int)pyroState);
        return false;
    }

    // Validate duration
    if (durationMs < minFireDuration || durationMs > maxFireDuration)
    {
        LOGE("%s: Fire duration %dms out of range (%d-%d)", 
             getName(), durationMs, minFireDuration, maxFireDuration);
        return false;
    }

    // Final continuity check
    if (requireContinuity && !checkContinuity())
    {
        LOGE("%s: Cannot fire - continuity lost", getName());
        pyroState = PyroState::CONTINUITY_FAIL;
        return false;
    }

    // FIRE!
    pyroState = PyroState::FIRING;
    fireDuration = durationMs;
    fireStartTime = millis();
    
    Relay::setPinState(RelayState::ON);  // Bypass safety override, use base class
    
    LOGI("%s: FIRING for %dms", getName(), durationMs);
    return true;
}

bool PyroCharge::checkContinuity()
{
    continuityGood = measureContinuity();
    
    if (!continuityGood)
    {
        LOGD("%s: Continuity check FAILED", getName());
    }
    
    return continuityGood;
}

void PyroCharge::updateState()
{
    // Handle firing timeout
    if (pyroState == PyroState::FIRING)
    {
        unsigned long elapsed = millis() - fireStartTime;
        
        if (elapsed >= (unsigned long)fireDuration)
        {
            // Fire complete
            Relay::setPinState(RelayState::OFF);
            pyroState = PyroState::FIRED;
            LOGI("%s: Fire complete", getName());
        }
    }

    // Periodic continuity check when armed
    if (pyroState == PyroState::ARMED)
    {
        static unsigned long lastCheck = 0;
        if (millis() - lastCheck > 1000)  // Check every second
        {
            checkContinuity();
            lastCheck = millis();
        }
    }
}

void PyroCharge::setPinState(RelayState state)
{
    // Override to prevent direct pin manipulation
    // Only allow firing through proper fire() method
    if (state == RelayState::ON && pyroState != PyroState::FIRING)
    {
        LOGE("%s: Attempted unsafe pin activation - blocked", getName());
        return;
    }
    
    Relay::setPinState(state);
}

bool PyroCharge::measureContinuity()
{
    if (continuityPin < 0)
    {
        // No continuity check configured, assume good
        return true;
    }

    // Read analog value
    int reading = analogRead(continuityPin);
    
    // Simple threshold check
    // In a real implementation, you might:
    // - Use a voltage divider circuit
    // - Check for expected resistance range
    // - Account for ADC resolution
    // For now, just check if there's any signal
    
    // Threshold depends on your circuit
    // Example: If > 100 counts (out of 1024 on 10-bit ADC), consider good
    return reading > 100;
}

void PyroCharge::doSafe()
{
    // Turn off the relay
    Relay::setPinState(RelayState::OFF);
    
    // Set to safed state (unless already fired)
    if (pyroState != PyroState::FIRED)
    {
        pyroState = PyroState::SAFED;
        LOGD("%s: SAFED", getName());
    }
    else
    {
        LOGD("%s: Safed (already fired)", getName());
    }
}

} // namespace astra
