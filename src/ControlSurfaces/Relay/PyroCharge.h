#ifndef PYRO_CHARGE_H
#define PYRO_CHARGE_H

#include "Relay.h"
#include <Arduino.h>

namespace astra
{
    /**
     * Pyro charge states
     */
    enum class PyroState
    {
        SAFED,           // Not armed, cannot fire
        ARMED,           // Armed, ready to fire
        FIRING,          // Currently firing
        FIRED,           // Has been fired (one-shot)
        CONTINUITY_FAIL  // Continuity check failed
    };

    /**
     * Configuration for pyrotechnic charge
     */
    struct PyroChargeConfig : public RelayConfig
    {
        int continuityPin = -1;         // Analog pin for continuity sensing
        int maxFireDuration = 1000;     // Maximum firing duration in milliseconds (safety)
        int minFireDuration = 50;       // Minimum firing duration in milliseconds
        float continuityCurrent = 0.0f; // Expected continuity current (implementation specific)
        bool requireContinuity = true;  // Require continuity check before arming
        bool oneShot = true;            // Can only fire once
    };

    /**
     * PyroCharge - Pyrotechnic charge control with safety features
     * 
     * Controls pyrotechnic charges (e-matches, igniters) with additional safety:
     * - Continuity sensing to verify connection
     * - Armed/safed states
     * - Fire duration limits
     * - One-shot firing (optional)
     * - Automatic safety timeout
     * 
     * Normalized position: 0.0 = safe, 1.0 = fire (when armed)
     *   checkContinuity() - Verify e-match connected
     *   arm() - Arm the charge (only if continuity good)
     *   fire(duration) - Fire for specified duration
     *   Automatically safes after firing
     */
    class PyroCharge : public Relay
    {
    public:
        PyroCharge(const char* name = "PyroCharge");
        virtual ~PyroCharge() = default;

        // Override ControlSurface methods for safety
        bool setNormalizedPosition(float normalized) override;
        bool zero() override;
        bool EStop() override;

        /**
         * Pyro-specific control methods
         */
        bool arm();                           // Arm the charge (requires continuity)
        bool safe();                          // Safe the charge (disarm)
        bool fire(int durationMs = 500);     // Fire for specified duration
        bool checkContinuity();               // Check continuity
        
        /**
         * State queries
         */
        PyroState getState() const { return pyroState; }
        bool isArmed() const { return pyroState == PyroState::ARMED; }
        bool isFired() const { return pyroState == PyroState::FIRED; }
        bool hasContinuity() const { return continuityGood; }

        /**
         * Update method - must be called regularly to handle timing
         */
        void updateState();

    protected:
        int init(const ControlSurfaceConfig* config) override;
        void setPinState(RelayState state) override;  

    private:
        int continuityPin;
        int maxFireDuration;
        int minFireDuration;
        float continuityCurrent;
        bool requireContinuity;
        bool oneShot;
        
        PyroState pyroState;
        bool continuityGood;
        unsigned long fireStartTime;
        int fireDuration;

        /**
         * Measure continuity
         */
        bool measureContinuity();

        /**
         * Safe the charge (internal)
         */
        void doSafe();
    };

} // namespace astra

#endif // PYRO_CHARGE_H
