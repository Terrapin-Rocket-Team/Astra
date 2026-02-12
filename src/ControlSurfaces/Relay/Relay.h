#ifndef RELAY_H
#define RELAY_H

#include "ControlSurfaces/ControlSurface.h"
#include <Arduino.h>

namespace astra
{
    /**
     * Configuration for relay/solenoid control
     */
    enum class RelayState{
        OFF,
        ON 
    };

    inline constexpr bool operator==(RelayState lhs, RelayState rhs) noexcept
    {
        return static_cast<unsigned char>(lhs) == static_cast<unsigned char>(rhs);
    }
    inline constexpr bool operator!=(RelayState lhs, RelayState rhs) noexcept
    {
        return static_cast<unsigned char>(lhs) != static_cast<unsigned char>(rhs);
    }
    struct RelayConfig : public ControlSurfaceConfig
    {
        int pin = -1;                               // Digital output pin
        bool invert = false;                        // false = HIGH activates relay, true = LOW activates
        RelayState defaultState = RelayState::OFF;  // Default state on initialization (false = off)
    };

    /**
     * Relay - Digital on/off control for relays and solenoids
     * 
     * Controls simple on/off devices like relays, solenoids, and valves.
     * Normalized position: 0.0 = off, 1.0 = on
     * Raw position: 0 = off, 1 = on
     * 
     */
    class Relay : public ControlSurface
    {
    public:
        Relay(const char* name = "Relay");
        virtual ~Relay() = default;

        // ControlSurface interface implementation
        bool setNormalizedPosition(float normalized) override;
        float getNormalizedPosition() const override;
        bool setRawPosition(float raw) override;
        float getRawPosition() const override;
        bool zero() override;
        bool isWithinLimits() const override;
        bool EStop() override;

        /**
         * Convenience methods for relay control
         */
        bool turnOn();
        bool turnOff();
        RelayState getState() const;

    protected:
        int init(const ControlSurfaceConfig* config) override;

        int pin;
        bool invert;
        RelayState state;

        /**
         * Set the physical pin state
         * @param on true to activate, false to deactivate
         */
        virtual void setPinState(RelayState state);
    };

} // namespace astra

#endif // RELAY_H
