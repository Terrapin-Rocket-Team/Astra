#include "AstraRocketConfig.h"
#include "RocketState.h"
#include <arc_protocol.h>

namespace astra_rocket {

AstraRocketConfig::AstraRocketConfig()
    : liftoffAccelThreshold(3.0),
      liftoffDetectDuration(100),
      burnoutAccelThreshold(1.5),
      apogeeVelocityThreshold(2.0),
      landingVelocityThreshold(1.0),
      landingDetectDuration(3000),
      #if defined(ENV_STM)
      storageBackend(StorageBackend::EMMC),
      #elif defined(ENV_TEENSY)
      storageBackend(StorageBackend::SD_CARD),
      #elif defined(ENV_ESP)
      storageBackend(StorageBackend::SD_CARD),
      #elif defined(NATIVE)
      storageBackend(StorageBackend::NONE),
      #else
      #error "Unsupported platform: No storage backend default defined"
      #endif
      #if defined(ENV_TEENSY)
      // On Teensy, BUILTIN_SDCARD is typically defined as 254
      // If not defined, we'll use 254 as a sensible default for Teensy 4.1
      #ifdef BUILTIN_SDCARD
      sdCardCS(BUILTIN_SDCARD),
      #else
      sdCardCS(254),  // Teensy 4.1 built-in SD card default
      #endif
      #else
      sdCardCS(-1),  // Will be set by user if needed for other platforms
      #endif
      preflightLogRate(1.0),
      flightLogRate(50.0),
      postflightLogRate(1.0),
      flashBackup(false),
      radioSerial(nullptr),
      arcStream(nullptr),
      arcAddress(ARC_ADDR_FC_N),
      arcEnabled(false)
{
#ifdef ENV_TEENSY
    radioSerial = &Serial2;
#endif
}

AstraRocketConfig& AstraRocketConfig::withState(RocketState *state) {
    configuredRocketState = state;
    AstraConfig::withState(state);
    return *this;
}

// Flight detection thresholds
AstraRocketConfig& AstraRocketConfig::withLiftoffAccelThreshold(double accelG) {
    liftoffAccelThreshold = accelG;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withLiftoffDetectDuration(unsigned long durationMs) {
    liftoffDetectDuration = durationMs;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withBurnoutAccelThreshold(double accelG) {
    burnoutAccelThreshold = accelG;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withApogeeVelocityThreshold(double velocityMs) {
    apogeeVelocityThreshold = velocityMs;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withLandingVelocityThreshold(double velocityMs) {
    landingVelocityThreshold = velocityMs;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withLandingDetectDuration(unsigned long durationMs) {
    landingDetectDuration = durationMs;
    return *this;
}

// Storage configuration
AstraRocketConfig& AstraRocketConfig::withStorageBackend(StorageBackend backend) {
    storageBackend = backend;
    return *this;
}

// Logging configuration
AstraRocketConfig& AstraRocketConfig::withSDCardCS(int csPin) {
    sdCardCS = csPin;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withPreflightLogRate(double rateHz) {
    preflightLogRate = rateHz;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withFlightLogRate(double rateHz) {
    flightLogRate = rateHz;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withPostflightLogRate(double rateHz) {
    postflightLogRate = rateHz;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withFlashBackup(bool enable) {
    flashBackup = enable;
    return *this;
}

// Radio configuration
AstraRocketConfig& AstraRocketConfig::withRadioSerial(SerialUART_t &serial) {
    radioSerial = &serial;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withArcProtocol(uint8_t addr, Stream &stream) {
    arcAddress = addr;
    arcStream = &stream;
    arcEnabled = true;
    return *this;
}

} // namespace astra_rocket
