#ifndef WATCHDOG_SAFETY_H
#define WATCHDOG_SAFETY_H

#include <Arduino.h>
#include "robot_config.h"

#define FAULT_NONE     0x00
#define FAULT_I2C      0x01
#define FAULT_IMU      0x02
#define FAULT_WIFI     0x04
#define FAULT_BATTERY  0x08
#define FAULT_SERVO    0x10
#define FAULT_ALL      0xFF

struct SystemState {
  uint32_t lastCmdMs;
  uint8_t  faultCode;
  bool     otaActive;
  bool     commTimeout;
  bool     moving;
  int16_t  wifiRSSI;
  uint32_t freeHeap;
};

extern SystemState sysState;

void wdtInit();
void wdtSubscribeTask(TaskHandle_t taskHandle, const char* taskName);
void wdtFeed();
bool hasFault(uint8_t faultBit);
void handleFault(uint8_t faultBit);
void safeStop();
void safeResume();
void watchdogPoll();

#endif
