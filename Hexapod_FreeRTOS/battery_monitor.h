#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include "robot_config.h"

enum BatteryState : uint8_t {
  BATT_NORMAL   = 0,
  BATT_WARNING  = 1,
  BATT_CRITICAL = 2,
  BATT_SHUTDOWN = 3
};

extern float battVoltage;
extern float battPercent;
extern BatteryState battFSMState;

void batteryInit();
float battReadVoltage();
void battLoop();
void battEmergencyAction();
void battShutdown();

#endif
