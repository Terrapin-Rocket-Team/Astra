#ifndef HITL_H
#define HITL_H

/**
 * HITL (Hardware-In-The-Loop) Framework
 *
 * This header provides a convenient include for all HITL components.
 * Include this to enable HITL simulation testing with your flight computer.
 *
 * Quick Start:
 * 1. Include this header
 * 2. Create HITL sensors instead of hardware sensors
 * 3. In your main loop, parse incoming HITL/ messages
 * 4. Call astraSys->update(simTime) with simulation time
 * 5. DataLogger automatically outputs TELEM/ messages
 *
 * Example:
 *   #include <Sensors/HITL/HITL.h>
 *
 *   // Create HITL sensors
 *   HITLBarometer* baro = new HITLBarometer();
 *   HITLAccel* accel = new HITLAccel();
 *   HITLGyro* gyro = new HITLGyro();
 *   HITLMag* mag = new HITLMag();
 *   HITLGPS* gps = new HITLGPS();
 *
 *   // In loop:
 *   if (Serial.available()) {
 *       String line = Serial.readStringUntil('\n');
 *       if (line.startsWith("HITL/")) {
 *           double simTime;
 *           if (HITLParser::parseAndInject(line.c_str(), simTime)) {
 *               astraSys->update(simTime);
 *               // TELEM/ output happens automatically via DataLogger
 *           }
 *       }
 *   }
 */

#include "HITLSensorBuffer.h"
#include "HITLBarometer.h"
#include "HITLAccel.h"
#include "HITLGyro.h"
#include "HITLMag.h"
#include "HITLGPS.h"
#include "../../Testing/HITLParser.h"

#endif // HITL_H
