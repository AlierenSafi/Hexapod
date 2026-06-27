#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

enum OTAState : uint8_t {
  OTA_STATE_IDLE       = 0,
  OTA_STATE_CONNECTING = 1,
  OTA_STATE_READY      = 2,
  OTA_STATE_STARTING   = 3,
  OTA_STATE_FLASHING   = 4,
  OTA_STATE_DONE       = 5,
  OTA_STATE_ERROR      = 6
};

struct OTAContext {
  volatile OTAState state;
  volatile uint8_t  lastProgress;
  volatile uint32_t lastErrorCode;
  volatile bool     wifiEnabled;
  uint32_t          lastConnectAttemptMs;
  uint32_t          totalBytesToFlash;
  uint32_t          bytesDoneFlash;
  char              lastErrorMsg[64];
};

extern OTAContext otaCtx;
extern TaskHandle_t hOTATask;

void setupOTA();
void otaEnable();
void otaDisable();
void otaPrintStatus();
void taskOTA(void* param);

#endif
