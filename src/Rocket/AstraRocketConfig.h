#ifndef ASTRA_ROCKET_CONFIG_H
#define ASTRA_ROCKET_CONFIG_H
#include <Utils/AstraConfig.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Mag/Mag.h>
#include <RecordData/Storage/IStorage.h>

using namespace astra;

namespace astra_rocket {
class RocketState;

/**
 * AstraRocketConfig: Configuration builder for rocket flight computers
 *
 * Extends AstraConfig with rocket-specific configuration options:
 * - Sensor selection and wiring
 * - Flight detection thresholds
 * - Pyro channel configuration
 * - Deployment logic parameters
 * - Logging rates for different flight phases
 */
class AstraRocketConfig : public AstraConfig {
public:
    using AstraConfig::withState;
    AstraRocketConfig();

    AstraRocketConfig& withState(RocketState *state);

    // ===== Flight Detection Thresholds =====

    /**
     * Set liftoff acceleration threshold (G's)
     * Default: 3.0 G
     */
    AstraRocketConfig& withLiftoffAccelThreshold(double accelG);

    /**
     * Set liftoff detection duration (milliseconds)
     * Acceleration must exceed threshold for this duration
     * Default: 100 ms
     */
    AstraRocketConfig& withLiftoffDetectDuration(unsigned long durationMs);

    /**
     * Set motor burnout acceleration threshold (G's)
     * Burnout is detected once signed earth-frame vertical acceleration
     * falls to or below this threshold
     * Default: 1.5 G
     */
    AstraRocketConfig& withBurnoutAccelThreshold(double accelG);

    /**
     * Set apogee velocity threshold (m/s)
     * Apogee is detected once vertical velocity is at or below -threshold
     * Default: 2.0 m/s
     */
    AstraRocketConfig& withApogeeVelocityThreshold(double velocityMs);

    /**
     * Set landing velocity threshold (m/s)
     * Default: 1.0 m/s
     */
    AstraRocketConfig& withLandingVelocityThreshold(double velocityMs);

    /**
     * Set landing detection duration (milliseconds)
     * Low velocity must be sustained for this duration
     * Default: 3000 ms
     */
    AstraRocketConfig& withLandingDetectDuration(unsigned long durationMs);

    // ===== Storage Configuration =====

    /**
     * Set storage backend type
     * Platform defaults:
     * - STM32: EMMC
     * - Teensy: SD_CARD (SDIO via SdFat)
     * - ESP32: SD_CARD (SDMMC 4-bit)
     */
    AstraRocketConfig& withStorageBackend(StorageBackend backend);

    // ===== Logging Configuration =====

    /**
     * Set SD card chip select pin
     * Default: BUILTIN_SDCARD (for Teensy)
     */
    AstraRocketConfig& withSDCardCS(int csPin);

    /**
     * Set preflight logging rate (Hz)
     * Default: 1.0 Hz
     */
    AstraRocketConfig& withPreflightLogRate(double rateHz);

    /**
     * Set in-flight logging rate (Hz)
     * Default: 50.0 Hz
     */
    AstraRocketConfig& withFlightLogRate(double rateHz);

    /**
     * Set postflight logging rate (Hz)
     * Default: 1.0 Hz
     */
    AstraRocketConfig& withPostflightLogRate(double rateHz);

    /**
     * Enable onboard flash memory as backup logging
     * Default: false
     */
    AstraRocketConfig& withFlashBackup(bool enable);

    // ===== Radio Configuration =====

    /**
     * Set radio serial port
     * Default: Serial2
     */
    AstraRocketConfig& withRadioSerial(SerialUART_t &serial);

    /**
     * Enable ARC protocol command handling on a stream.
     *
     * This is intentionally separate from HITL. It gives the FC an ARC address
     * and lets it respond to ARC NETMGMT/FC_COORD commands on the configured
     * transport before simulator sample injection is wired in.
     */
    AstraRocketConfig& withArcProtocol(uint8_t addr, Stream &stream);

    // ===== Base Astra Configuration Pass-through =====
    // Note: All AstraConfig methods are now directly available through inheritance

    // Getters for all configuration parameters

    double getLiftoffAccelThreshold() const { return liftoffAccelThreshold; }
    unsigned long getLiftoffDetectDuration() const { return liftoffDetectDuration; }
    double getBurnoutAccelThreshold() const { return burnoutAccelThreshold; }
    double getApogeeVelocityThreshold() const { return apogeeVelocityThreshold; }
    double getLandingVelocityThreshold() const { return landingVelocityThreshold; }
    unsigned long getLandingDetectDuration() const { return landingDetectDuration; }

    StorageBackend getStorageBackend() const { return storageBackend; }
    int getSDCardCS() const { return sdCardCS; }
    double getPreflightLogRate() const { return preflightLogRate; }
    double getFlightLogRate() const { return flightLogRate; }
    double getPostflightLogRate() const { return postflightLogRate; }
    bool getFlashBackup() const { return flashBackup; }

    SensorManager *getSensorManager() { return &sensorManager; }
    State *getConfiguredState() const { return state; }
    RocketState *getConfiguredRocketState() const { return configuredRocketState; }
    uint8_t getConfiguredDataLogCount() const { return numLogs; }
    uint8_t getConfiguredEventLogCount() const { return numEventLogs; }

    SerialUART_t* getRadioSerial() const { return radioSerial; }
    Stream* getArcStream() const { return arcStream; }
    uint8_t getArcAddress() const { return arcAddress; }
    bool getArcEnabled() const { return arcEnabled && arcStream != nullptr; }

    RuntimeMode getRuntimeMode() const { return runtimeMode; }
    bool getSimulationEnabled() const { return runtimeMode != RuntimeMode::Hardware; }
    bool getHITLEnabled() const { return getSimulationEnabled(); }
    bool getSITLEnabled() const { return runtimeMode == RuntimeMode::SITL; }

private:
    // Flight detection thresholds
    double liftoffAccelThreshold;
    unsigned long liftoffDetectDuration;
    double burnoutAccelThreshold;
    double apogeeVelocityThreshold;
    double landingVelocityThreshold;
    unsigned long landingDetectDuration;

    // Storage configuration
    StorageBackend storageBackend;

    // Logging configuration
    int sdCardCS;
    double preflightLogRate;
    double flightLogRate;
    double postflightLogRate;
    bool flashBackup;

    // Radio configuration
    SerialUART_t *radioSerial;
    Stream *arcStream;
    uint8_t arcAddress;
    bool arcEnabled;
    RocketState *configuredRocketState = nullptr;
};

} // namespace astra_rocket

#endif // ASTRA_ROCKET_CONFIG_H
