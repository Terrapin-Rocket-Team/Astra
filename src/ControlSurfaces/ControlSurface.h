#ifndef CONTROL_SURFACE_H
#define CONTROL_SURFACE_H

#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/Logging/EventLogger.h"

namespace astra
{
    /**
     * Base configuration for all control surfaces
     */
    struct ControlSurfaceConfig
    {
        const char* name = nullptr;
        virtual ~ControlSurfaceConfig() = default;
    };

    /**
     * ControlSurface - Base interface for motors, servos, and actuators
     * 
     * Provides a unified interface for controlling various actuation devices
     * on a rocket, including servos, motors, relays, and pyrotechnic charges.
     * 
     * All control surfaces support:
     * - Normalized position control (0.0 to 1.0 or -1.0 to 1.0)
     * - Raw position control (device-specific units)
     * - Zeroing/calibration
     * - Safety limits checking
     * - Telemetry integration via DataReporter
     */
    class ControlSurface : public DataReporter
    {
    public:
        virtual ~ControlSurface() = default;

        /**
         * Initialize the control surface with configuration
         * @param config Device-specific configuration (cast to appropriate type)
         * @return true if initialization successful
         */
        int begin(const ControlSurfaceConfig* config)
        {
            return initialized = init(config);
        }

        /**
         * Set position using normalized value (0.0 to 1.0 or -1.0 to 1.0)
         * @param normalized Normalized position value
         * @return true if position set successfully
         */
        virtual bool setNormalizedPosition(float normalized) = 0;

        /**
         * Get current normalized position
         * @return Normalized position value
         */
        virtual float getNormalizedPosition() const = 0;

        /**
         * Set position using raw device-specific units
         * @param raw Raw position value (e.g., degrees for servo, PWM for motor)
         * @return true if position set successfully
         */
        virtual bool setRawPosition(float raw) = 0;

        /**
         * Get current raw position
         * @return Raw position value in device-specific units
         */
        virtual float getRawPosition() const = 0;

        /**
         * Zero/calibrate the control surface to neutral position
         * @return true if zeroing successful
         */
        virtual bool zero() = 0;

        /**
         * Check if current position is within safe limits
         * @return true if within limits
         */
        virtual bool isWithinLimits() const = 0;

        /**
         * Emergency stop - immediately halt all motion/power
         * @return true if stopped successfully
         */
        virtual bool EStop() = 0;

        /**
         * Check if control surface is initialized and ready
         */
        bool isInitialized() const { return initialized; }

        /**
         * Explicit bool conversion - returns initialization state
         */
        explicit operator bool() const { return initialized; }

    protected:
        /**
         * Protected constructor - only derived classes can instantiate
         * @param name Name for telemetry/logging
         */
        ControlSurface(const char* name) : DataReporter(name) {}

        /**
         * Device-specific initialization implementation
         * @param config Device configuration
         * @return true if successful
         */
        virtual int init(const ControlSurfaceConfig* config) = 0;

        bool initialized = false;
    };

} // namespace astra

#endif // CONTROL_SURFACE_H