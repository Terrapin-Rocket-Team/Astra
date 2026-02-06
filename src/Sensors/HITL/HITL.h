#ifndef HITL_H
#define HITL_H

/**
 * HITL (Hardware-In-The-Loop) Framework
 *
 * This header provides a convenient include for all HITL components.
 * Include this to enable HITL simulation testing with your flight computer.
 *
 * Quick Start:
 * 1. Include this header and SerialMessageRouter
 * 2. Create HITL sensors instead of hardware sensors
 * 3. Register HITL handler with SerialMessageRouter
 * 4. Router automatically parses HITL/ messages and calls your handler
 * 5. DataLogger automatically outputs TELEM/ messages
 *
 * Example (Recommended - with SerialMessageRouter):
 *   #include <Sensors/HITL/HITL.h>
 *   #include <Communication/SerialMessageRouter.h>
 *
 *   using namespace astra;
 *
 *   // Create HITL sensors
 *   HITLBarometer* baro = new HITLBarometer();
 *   HITLAccel* accel = new HITLAccel();
 *   HITLGyro* gyro = new HITLGyro();
 *   HITLMag* mag = new HITLMag();
 *   HITLGPS* gps = new HITLGPS();
 *
 *   SerialMessageRouter router;
 *
 *   void setup() {
 *       router.withInterface(&Serial)
 *             .withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
 *                 double simTime;
 *                 if (HITLParser::parse(msg, simTime)) {
 *                     astraSys->update(simTime);
 *                 }
 *             });
 *   }
 *
 *   void loop() {
 *       router.update();  // Handles all serial routing including HITL
 *   }
 *
 * Legacy Example (Manual parsing):
 *   if (Serial.available()) {
 *       String line = Serial.readStringUntil('\n');
 *       if (line.startsWith("HITL/")) {
 *           double simTime;
 *           if (HITLParser::parseAndInject(line.c_str(), simTime)) {
 *               astraSys->update(simTime);
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
