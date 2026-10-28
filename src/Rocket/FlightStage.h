#ifndef FLIGHT_STAGE_H
#define FLIGHT_STAGE_H

namespace astra_rocket {

/**
 * Flight stage enumeration for rocket flight phases
 */
enum FlightStage {
    PAD_IDLE = 0,           // On pad, system armed, waiting for liftoff
    BOOST = 1,              // Motor firing, high acceleration
    COAST = 2,              // Coasting to apogee, motor burnout
    APOGEE = 3,             // At or near apogee
    EXPECTING_DROGUE = 4,   // Past apogee, expecting drogue deployment (high descent rate)
    UNDER_DROGUE = 5,       // Slower descent detected (drogue deployed)
    EXPECTING_MAIN = 6,     // Below main deployment altitude, expecting main deployment
    UNDER_MAIN = 7,         // Very slow descent detected (main deployed)
    LANDED = 8              // Landed, recovery mode
};

/**
 * Convert flight stage to human-readable string
 */
inline const char* flightStageToString(FlightStage stage) {
    switch(stage) {
        case PAD_IDLE:          return "PAD_IDLE";
        case BOOST:             return "BOOST";
        case COAST:             return "COAST";
        case APOGEE:            return "APOGEE";
        case EXPECTING_DROGUE:  return "EXPECTING_DROGUE";
        case UNDER_DROGUE:      return "UNDER_DROGUE";
        case EXPECTING_MAIN:    return "EXPECTING_MAIN";
        case UNDER_MAIN:        return "UNDER_MAIN";
        case LANDED:            return "LANDED";
        default:                return "UNKNOWN";
    }
}

} // namespace astra_rocket

#endif // FLIGHT_STAGE_H
