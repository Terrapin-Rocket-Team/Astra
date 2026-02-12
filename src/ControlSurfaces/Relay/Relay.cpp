#include "Relay.h"

namespace astra
{

Relay::Relay(const char* name) : ControlSurface(name), pin(-1), invert(false), state(RelayState::OFF)
{    
    // Register telemetry columns
    addColumn("%d", &state, "Relay State");
}

int Relay::init(const ControlSurfaceConfig* config)
{
    const RelayConfig* relayConfig = static_cast<const RelayConfig*>(config);
    
    if (!relayConfig || relayConfig->pin < 0)
    {
        LOGE("%s: Invalid configuration", getName());
        return -1;
    }

    pin = relayConfig->pin;
    invert = relayConfig->invert;
    state = relayConfig->defaultState;

    // Configure pin
    pinMode(pin, OUTPUT);
    setPinState(state);

    LOGI("%s: Initialized on pin %d (active %s)", 
         getName(), pin, invert ? "HIGH" : "LOW");
    return 0;
}

bool Relay::setNormalizedPosition(float normalized)
{
    // Clamp to valid range
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    // Threshold at 0.5
    RelayState newState = (normalized >= 0.5f) ? RelayState::ON : RelayState::OFF;
    
    if (newState != state)
    {
        state = newState;
        setPinState(state);
        LOGD("%s: Set to %s", getName(), state == RelayState::ON ? "ON" : "OFF");
    }

    return true;
}

float Relay::getNormalizedPosition() const
{
    return state == RelayState::ON ? 1.0f : 0.0f;
}

bool Relay::setRawPosition(float raw)
{
    return setNormalizedPosition(raw);
}

float Relay::getRawPosition() const
{
    return state == RelayState::ON ? 1.0f : 0.0f;
}

bool Relay::zero()
{
    return turnOff();
}

bool Relay::isWithinLimits() const
{
    return true;
}

bool Relay::EStop()
{
    return turnOff();
}

bool Relay::turnOn()
{
    return setNormalizedPosition(1.0f);
}

bool Relay::turnOff()
{
    return setNormalizedPosition(0.0f);
}

RelayState Relay::getState() const
{
    return state;
}

void Relay::setPinState(RelayState state)
{
    if(invert){
        digitalWrite(pin, state == RelayState::OFF ? HIGH : LOW);
    } else {
        digitalWrite(pin, state == RelayState::ON ? HIGH : LOW);
    }
}

}
