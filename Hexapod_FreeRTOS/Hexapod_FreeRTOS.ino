// ════════════════════════════════════════════════════════════════
//  Hexapod_FreeRTOS.ino  ·  Main Entry & Task Scheduler  ·  v3.2.0
// ════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include "robot_config.h"
#include "servo_drivers.h"
#include "kinematics.h"
#include "imu_leveling.h"
#include "communication.h"
#include "battery_monitor.h"
#include "watchdog_safety.h"
#include "telemetry.h"
#include "ota_manager.h"

#define I2C_SDA          21
#define I2C_SCL          22
#define OE_PIN            4

// Contact switch pins
const uint8_t SWITCH_PINS[6] = {13, 14, 25, 26, 32, 33};

// Task Handles
TaskHandle_t hSensorTask = nullptr;
TaskHandle_t hKinTask = nullptr;

// Event Group for Leg Phase Resets (Bits 0-5 correspond to Legs 0-5)
EventGroupHandle_t xEventGroupPhaseReset = nullptr;

// ── Sensor Comm Task (Core 0, 100Hz) ───────────────────────────
void taskSensorComm(void* param) {
  wdtSubscribeTask(NULL, "SensorCommTask");

  if (mpuAvailable) {
    Serial.println(F("[CORE0] IMU warming up (2 sec)..."));
    for (int i = 0; i < 20; i++) {
      wdtFeed();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.println(F("[CORE0] IMU warm-up complete."));
  }

  batteryInit();
  uint32_t lastMs = millis();
  uint8_t teleCounter = 0;

  while (true) {
    uint32_t now = millis();
    float dt = (float)(now - lastMs) * 0.001f;
    dt = constrain(dt, 0.001f, 0.100f);
    lastMs = now;

    // Feed watchdog & poll safety check
    wdtFeed();
    watchdogPoll();

    // WiFi & WebSockets Loop
    wifiLoop();

    // Telemetry Loop
    sendFastTelemetry();
    teleCounter++;
    if (teleCounter >= 10) {
      teleCounter = 0;
      sendSlowTelemetry();
    }

    // Battery Monitor Loop
    battLoop();

    // IMU complementary filter update @ 50Hz
    static uint8_t imuSkipCount = 0;
    static float imuDtAccum = 0.0f;
    imuDtAccum += dt;
    imuSkipCount++;

    if (mpuAvailable && imuSkipCount >= 2) {
      float alpha;
      xSemaphoreTake(configMutex, portMAX_DELAY);
      alpha = cfg.compAlpha;
      xSemaphoreGive(configMutex);

      RobotSettings imuSnap;
      imuSnap.compAlpha = alpha;
      updateCompFilter(imuDtAccum, &imuSnap);

      imuSkipCount = 0;
      imuDtAccum = 0.0f;
    }

    // NRF24 polling
    nrfPoll();

    // Leg Switch contact polling & Phase Reset trigger via Event Group
    bool phaseRstEn = false;
    xSemaphoreTake(configMutex, portMAX_DELAY);
    phaseRstEn = cfg.phaseResetEnabled;
    xSemaphoreGive(configMutex);

    if (phaseRstEn) {
      for (uint8_t i = 0; i < 6; i++) {
        bool contact = (digitalRead(SWITCH_PINS[i]) == LOW);
        if (contact && !legs[i].switchContact && legs[i].isSwing) {
          // Set leg bit in Phase Reset event group
          xEventGroupSetBits(xEventGroupPhaseReset, (1 << i));
        }
        legs[i].switchContact = contact;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ── Kinematics & Walking Engine Task (Core 1, 50Hz) ──────────
void taskKinematics(void* param) {
  wdtSubscribeTask(NULL, "KinematicsTask");

  RobotSettings snap;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  snap = cfg;
  xSemaphoreGive(configMutex);

  // Initialize leg states
  for (uint8_t i = 0; i < 6; i++) {
    float off = getPhaseOffset(i, snap.gaitType);
    legs[i].phase         = off + snap.swingRatio;
    if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
    legs[i].isSwing       = false;
    legs[i].switchContact = false;
    legs[i].footPos       = {0.0f, snap.stanceRadius, snap.stanceHeight};
    legs[i].stancePos     = legs[i].footPos;
    legs[i].angles        = {0.0f, 0.0f, 0.0f};
  }

  bool prevSwing[6] = {};
  TickType_t lastWake = xTaskGetTickCount();
  RadioPacket activePkt;
  memset(&activePkt, 0, sizeof(activePkt)); // Default to STOP

  while (true) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));

    wdtFeed();

    // Check configuration updates via FreeRTOS Task Notifications
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
      xSemaphoreTake(configMutex, portMAX_DELAY);
      snap = cfg;
      xSemaphoreGive(configMutex);

      if (snap.gaitType != currentGait) {
        currentGait = snap.gaitType;
        for (uint8_t i = 0; i < 6; i++) {
          float off = getPhaseOffset(i, currentGait);
          legs[i].phase = off + snap.swingRatio;
          if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
          prevSwing[i] = false;
        }
        const char* gn[] = {"Tripod", "Ripple", "Wave"};
        Serial.printf("[KIN] Gait changed -> %s\n", gn[(int)currentGait]);
      }
    }

    // Non-blocking queue check for incoming commands
    RadioPacket queuePkt;
    if (xQueueReceive(xQueueCmd, &queuePkt, 0) == pdPASS) {
      activePkt = queuePkt;
    }

    float velX   = (float)activePkt.motion.xRate   / 100.0f;
    float velY   = (float)activePkt.motion.yRate   / 100.0f;
    float velYaw = (float)activePkt.motion.yawRate / 100.0f;

    bool moving = (activePkt.motion.type == 0x01) &&
                  (fabsf(velX)   > 0.05f ||
                   fabsf(velY)   > 0.05f ||
                   fabsf(velYaw) > 0.05f);

    sysState.moving = moving; // Publish status to core 0 cleanly

    if (activePkt.motion.type == 0x03) {
      GaitType ng = (GaitType)(abs((int)activePkt.motion.xRate) % 3);
      if (ng != currentGait) {
        currentGait = ng;
        xSemaphoreTake(configMutex, portMAX_DELAY);
        cfg.gaitType = currentGait;
        snap.gaitType = currentGait;
        xSemaphoreGive(configMutex);
        for (uint8_t i = 0; i < 6; i++) {
          float off = getPhaseOffset(i, currentGait);
          legs[i].phase = off + snap.swingRatio;
          if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
          prevSwing[i] = false;
        }
        const char* gn[] = {"Tripod", "Ripple", "Wave"};
        Serial.printf("[KIN] Gait cmd -> %s\n", gn[(int)currentGait]);
      }
    }

    // Auto-leveling offsets
    static float zOff[6];
    memset(zOff, 0, sizeof(zOff));
    if (mpuAvailable) {
      computeLevelingOffset(0.020f, &snap, zOff);
    }

    // Check phase reset events from Core 0 via Event Group
    EventBits_t rstBits = xEventGroupClearBits(xEventGroupPhaseReset, 0x3F);

    for (uint8_t i = 0; i < 6; i++) {
      if (rstBits & (1 << i)) {
        float off = getPhaseOffset(i, currentGait);
        legs[i].phase   = off + snap.swingRatio;
        if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
        legs[i].isSwing = false;
        prevSwing[i]    = false;
      }

      Vec3 target;

      if (!moving) {
        legs[i].isSwing = false;
        prevSwing[i]    = false;
        target = {0.0f, snap.stanceRadius, snap.stanceHeight + zOff[i]};
      } else {
        legs[i].phase += snap.gaitSpeed;
        if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;

        float off      = getPhaseOffset(i, currentGait);
        float relPhase = legs[i].phase - off;
        if (relPhase < 0.0f) relPhase += 1.0f;

        bool nowSwing = (relPhase < snap.swingRatio);
        legs[i].isSwing = nowSwing;

        if (!prevSwing[i] && nowSwing) {
          legs[i].stancePos = legs[i].footPos;
        }

        if (nowSwing) {
          float swingT = relPhase / snap.swingRatio;
          swingT = constrain(swingT, 0.0f, 1.0f);
          Vec3 swingEnd = {
            velX * snap.stepLength * 0.55f,
            snap.stanceRadius + velY * snap.stepLength * 0.55f,
            snap.stanceHeight
          };
          target = cycloidTrajectory(swingT, legs[i].stancePos, swingEnd, snap.stepHeight);
        } else {
          float stanceT = (relPhase - snap.swingRatio) / (1.0f - snap.swingRatio);
          stanceT = constrain(stanceT, 0.0f, 1.0f);
          target  = computeStanceFootPos(i, stanceT, velX, velY, velYaw, &snap);
        }

        target.z += zOff[i];
        prevSwing[i] = nowSwing;
      }

      legs[i].footPos = target;

      if (solveIK(i, target, &snap)) {
        writeAngles(i, &snap);
      } else {
        static uint32_t ikErrMs[6] = {};
        if ((millis() - ikErrMs[i]) > 500) {
          Serial.printf("[IK] Leg %d unreachable: X=%.1f Y=%.1f Z=%.1f\n",
                        i, target.x, target.y, target.z);
          ikErrMs[i] = millis();
        }

        Vec3 safe = {0.0f, snap.stanceRadius, snap.stanceHeight};
        if (solveIK(i, safe, &snap)) {
          writeAngles(i, &snap);
          legs[i].footPos = safe;
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=============================================="));
  Serial.println(F(" Hexapod ESP32 Controller - FreeRTOS v3.2.0   "));
  Serial.println(F("=============================================="));

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
  }

  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH); // Torque off during startup

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(10);

  // Initialize FreeRTOS Primitives
  configMutex = xSemaphoreCreateMutex();
  imuMutex    = xSemaphoreCreateMutex();
  wireMutex   = xSemaphoreCreateMutex();

  xEventGroupPhaseReset = xEventGroupCreate();
  xQueueCmd = xQueueCreate(10, sizeof(RadioPacket));

  if (!configMutex || !imuMutex || !wireMutex || !xEventGroupPhaseReset || !xQueueCmd) {
    Serial.println(F("[FATAL] FreeRTOS primitives creation failed! Restarting..."));
    esp_restart();
  }
  Serial.println(F("[RTOS] Created Mutexes, Queue, and Event Group."));

  wdtInit();

  loadConfiguration();

  mpuAvailable = mpuInit();
  if (mpuAvailable) {
    Serial.println(F("[HW] MPU6050 available."));
  } else {
    Serial.println(F("[HW] WARNING: MPU6050 not found. Deneyimli denge devre dışı."));
  }

  pca9685Init(PCA9685_RIGHT);
  pca9685Init(PCA9685_LEFT);

  wifiInit();

  #ifdef USE_BLE
    bleInit();
  #endif

  nrfAvailable = nrfInit();
  if (nrfAvailable) {
    Serial.println(F("[HW] NRF24L01+ available."));
  } else {
    Serial.println(F("[HW] WARNING: NRF24L01+ receiver not found."));
  }

  setupOTA();

  // Create Pinned Core Tasks
  xTaskCreatePinnedToCore(
    taskSensorComm,
    "SensorCommTask",
    8192,
    NULL,
    2,
    &hSensorTask,
    0
  );

  xTaskCreatePinnedToCore(
    taskKinematics,
    "KinematicsTask",
    8192,
    NULL,
    2,
    &hKinTask,
    1
  );

  xTaskCreatePinnedToCore(
    taskOTA,
    "OTATask",
    8192,
    NULL,
    1,
    &hOTATask,
    0
  );

  // Restore active low torque
  digitalWrite(OE_PIN, LOW);
  Serial.println(F("[RTOS] Tasks running on cores. Ready."));
}

void loop() {
  // Empty. FreeRTOS handles task execution.
  vTaskDelete(NULL);
}
