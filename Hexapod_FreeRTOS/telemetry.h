#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "robot_config.h"

extern char teleBuffer[1024];

void sendFastTelemetry();
void sendSlowTelemetry();
void sendEvent(const char* eventType, const char* message);
void sendCommTimeout();

#endif
