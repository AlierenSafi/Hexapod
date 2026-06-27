#include "watchdog_safety.h"
#include "servo_drivers.h"
#include "kinematics.h"
#include "telemetry.h"
#include "communication.h"
#include <esp_task_wdt.h>
#include <Wire.h>

#define WDT_TIMEOUT_SECONDS   5
#define WDT_PANIC_ON_TIMEOUT  true
#define COMM_TIMEOUT_CHECK_MS 100

SystemState sysState = {0, FAULT_NONE, false, false, false, -90, 0};

static bool wdtInitialized = false;
static uint32_t lastCommCheckMs = 0;
static uint32_t commTimeoutCount = 0;

void wdtInit() {
  Serial.println(F("\n[WDT] Initializing Watchdog..."));

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = (1 << 0) | (1 << 1),
    .trigger_panic = WDT_PANIC_ON_TIMEOUT
  };

  esp_err_t err = esp_task_wdt_init(&twdt_config);
  if (err != ESP_OK) {
    Serial.printf("[WDT] ERROR: TWDT init failed (err=%d)\n", err);
    return;
  }

  wdtInitialized = true;
  Serial.printf("[WDT] TWDT active: %d sec timeout, panic=%s\n",
                WDT_TIMEOUT_SECONDS, WDT_PANIC_ON_TIMEOUT ? "yes" : "no");
}

void wdtSubscribeTask(TaskHandle_t taskHandle, const char* taskName) {
  if (!wdtInitialized) return;

  esp_err_t err = esp_task_wdt_add(taskHandle);
  if (err == ESP_OK) {
    Serial.printf("[WDT] '%s' subscribed to TWDT.\n", taskName);
  } else {
    Serial.printf("[WDT] ERROR: '%s' subscription failed (err=%d)\n", taskName, err);
  }
}

void wdtFeed() {
  if (!wdtInitialized) return;
  esp_task_wdt_reset();
}

static void commTimeoutCheck() {
  uint32_t now = millis();
  if (now - lastCommCheckMs < COMM_TIMEOUT_CHECK_MS) return;
  lastCommCheckMs = now;

  RobotSettings snap;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  snap = cfg;
  xSemaphoreGive(configMutex);

  uint32_t elapsed = now - sysState.lastCmdMs;

  if (elapsed > snap.commTimeoutMs) {
    if (!sysState.commTimeout) {
      sysState.commTimeout = true;
      commTimeoutCount++;
      Serial.printf("[COMM] Timeout! %lu ms since last command. Motion stop.\n", elapsed);

      RadioPacket stopPkt;
      memset(&stopPkt, 0, sizeof(stopPkt));
      stopPkt.motion.type = 0x00; // STOP
      dispatchMotionCmd(stopPkt, SRC_NONE);

      sendCommTimeout();
    }
  } else {
    if (sysState.commTimeout) {
      sysState.commTimeout = false;
      Serial.println(F("[COMM] Timeout cleared."));
    }
  }
}

bool hasFault(uint8_t faultBit) {
  return (sysState.faultCode & faultBit) != 0;
}

void handleFault(uint8_t faultBit) {
  static uint32_t lastRecoveryMs[8] = {};

  int bitIdx = 0;
  for (int i = 0; i < 8; i++) {
    if (faultBit == (1 << i)) {
      bitIdx = i;
      break;
    }
  }

  uint32_t now = millis();
  if (now - lastRecoveryMs[bitIdx] < 5000) {
    return; // Cooldown actively prevents infinite recovery loops
  }
  lastRecoveryMs[bitIdx] = now;

  switch (faultBit) {
    case FAULT_I2C:
      Serial.println(F("[FAULT] I2C error - MPU/Servos affected. Recovery attempt..."));
      Wire.end();
      delay(10);
      Wire.begin(21, 22); // I2C_SDA=21, I2C_SCL=22
      Wire.setClock(400000);
      break;

    case FAULT_IMU:
      Serial.println(F("[FAULT] IMU error - Auto-leveling disabled."));
      sysState.faultCode |= FAULT_IMU;
      break;

    case FAULT_WIFI:
      Serial.println(F("[FAULT] WiFi error - Handled by WiFi task."));
      break;

    case FAULT_BATTERY:
      Serial.println(F("[FAULT] Battery error - Handled by Battery task."));
      break;

    case FAULT_SERVO:
      Serial.println(F("[FAULT] Servo error - PCA9685 recovery attempt..."));
      digitalWrite(OE_PIN, HIGH);
      delay(100);
      pca9685Init(PCA9685_RIGHT);
      pca9685Init(PCA9685_LEFT);
      digitalWrite(OE_PIN, LOW);
      break;
  }
}

void safeStop() {
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  [SAFETY] SAFE STOP TRIGGERED          ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));

  RadioPacket stopPkt;
  memset(&stopPkt, 0, sizeof(stopPkt));
  stopPkt.motion.type = 0x00; // STOP
  dispatchMotionCmd(stopPkt, SRC_NONE);

  delay(100);

  if (hKinTask != nullptr) {
    vTaskSuspend(hKinTask);
    Serial.println(F("[SAFETY] KinTask suspended."));
  }
  delay(50);

  sitDown();
  delay(500);

  digitalWrite(OE_PIN, HIGH);

  sendEvent("safe_stop", "Safe stop complete");
  Serial.println(F("[SAFETY] Servos disabled. System safe."));
}

void safeResume() {
  Serial.println(F("[SAFETY] System resuming..."));

  digitalWrite(OE_PIN, LOW);

  if (hKinTask != nullptr) {
    vTaskResume(hKinTask);
    Serial.println(F("[SAFETY] KinTask resumed."));
  }

  delay(200);
  sendEvent("safe_resume", "System resumed");
}

void watchdogPoll() {
  commTimeoutCheck();
}
