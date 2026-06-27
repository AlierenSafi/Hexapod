#include "battery_monitor.h"
#include "telemetry.h"     // For sendEvent
#include "servo_drivers.h" // For writeAngles, OE_PIN
#include "kinematics.h"    // For sitDown
#include <freertos/task.h>

#define BATT_PIN              34
#define BATT_ADC_SAMPLES      8
#define BATT_FILTER_SIZE      8
#define BATT_READ_INTERVAL_MS 1000
#define BATT_HYSTERESIS_V     0.15f

static const float BATT_CURVE_V[] = {6.0f, 6.4f, 7.0f, 7.4f, 8.2f, 8.4f};
static const float BATT_CURVE_P[] = {0.0f, 5.0f, 20.0f, 50.0f, 90.0f, 100.0f};

float battVoltage = 8.4f;
float battPercent = 100.0f;
BatteryState battFSMState = BATT_NORMAL;

static float battFilterBuf[BATT_FILTER_SIZE];
static uint8_t battFilterIdx = 0;
static uint32_t lastBattReadMs = 0;

static float voltToPercent(float v) {
  if (v <= BATT_CURVE_V[0]) return 0.0f;
  if (v >= BATT_CURVE_V[5]) return 100.0f;

  for (int i = 0; i < 5; i++) {
    if (v >= BATT_CURVE_V[i] && v <= BATT_CURVE_V[i+1]) {
      float t = (v - BATT_CURVE_V[i]) / (BATT_CURVE_V[i+1] - BATT_CURVE_V[i]);
      return BATT_CURVE_P[i] + t * (BATT_CURVE_P[i+1] - BATT_CURVE_P[i]);
    }
  }
  return 0.0f;
}

void batteryInit() {
  pinMode(BATT_PIN, INPUT);

  float vInit = battReadVoltage();
  for (int i = 0; i < BATT_FILTER_SIZE; i++) {
    battFilterBuf[i] = vInit;
  }
  battVoltage = vInit;
  battPercent = voltToPercent(vInit);
  Serial.printf("[BATT] Init: %.2fV (%u%%)\n", battVoltage, (unsigned int)battPercent);
}

float battReadVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < BATT_ADC_SAMPLES; i++) {
    sum += analogRead(BATT_PIN);
  }
  float avgRaw = (float)sum / (float)BATT_ADC_SAMPLES;
  // ESP32 ADC: 0..4095 range, 3.3V reference
  // Voltage divider: R1=10k, R2=4.7k -> scaling factor ~3.127
  float pinVolt = (avgRaw / 4095.0f) * 3.3f;
  float battVolt = pinVolt * 3.127f;
  return battVolt;
}

void battLoop() {
  uint32_t now = millis();
  if (now - lastBattReadMs < BATT_READ_INTERVAL_MS) return;
  lastBattReadMs = now;

  float currentVal = battReadVoltage();
  if (currentVal < 3.0f) return; // Unconnected sensor guard

  battFilterBuf[battFilterIdx] = currentVal;
  battFilterIdx = (battFilterIdx + 1) % BATT_FILTER_SIZE;

  float sum = 0.0f;
  for (int i = 0; i < BATT_FILTER_SIZE; i++) sum += battFilterBuf[i];
  battVoltage = sum / (float)BATT_FILTER_SIZE;
  battPercent = voltToPercent(battVoltage);

  // Fetch thresholds snapshot
  RobotSettings s;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  s = cfg;
  xSemaphoreGive(configMutex);

  // Battery FSM state transitions
  switch (battFSMState) {
    case BATT_NORMAL:
      if (battVoltage < s.battWarnVolt) {
        battFSMState = BATT_WARNING;
        sendEvent("batt_warning", "Voltaj uyarısı: 20% altı");
        Serial.println(F("[BATT] WARNING: Voltage below 20%."));
      }
      break;

    case BATT_WARNING:
      if (battVoltage > s.battWarnVolt + BATT_HYSTERESIS_V) {
        battFSMState = BATT_NORMAL;
        sendEvent("batt_normal", "Voltaj normale döndü");
      } else if (battVoltage < s.battCritVolt) {
        battFSMState = BATT_CRITICAL;
        sendEvent("batt_critical", "Kritik voltaj!");
        Serial.println(F("[BATT] CRITICAL: Voltage critical."));
      }
      break;

    case BATT_CRITICAL:
      if (battVoltage > s.battCritVolt + BATT_HYSTERESIS_V) {
        battFSMState = BATT_WARNING;
      } else if (battVoltage < s.battCutoffVolt) {
        battFSMState = BATT_SHUTDOWN;
        battEmergencyAction();
      }
      break;

    case BATT_SHUTDOWN:
      // Stay in shutdown mode
      break;
  }
}

void battEmergencyAction() {
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  [BATT] CRITICAL VOLTAGE - SHUTDOWN     ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));

  if (hKinTask != nullptr) {
    vTaskSuspend(hKinTask);
    Serial.println(F("[BATT] KinTask suspended."));
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  sitDown();
  delay(500);

  digitalWrite(OE_PIN, HIGH);
  Serial.println(F("[BATT] Servos disabled (OE=HIGH)."));

  sendEvent("batt_emergency", "Critical voltage - servos disabled");
}

void battShutdown() {
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  [BATT] SYSTEM SHUTDOWN INITIATED      ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));

  if (hKinTask != nullptr) {
    vTaskSuspend(hKinTask);
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  sitDown();
  delay(500);

  digitalWrite(OE_PIN, HIGH);
  Serial.println(F("[BATT] Servos powered off. System entering deep sleep."));

  sendEvent("batt_shutdown", "System shutdown complete");
  delay(500);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  esp_deep_sleep_start();
}
