#include "ota_manager.h"
#include "robot_config.h"
#include "servo_drivers.h"
#include "watchdog_safety.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>

#define OTA_HOSTNAME               "Hexapod-ESP32"
#define OTA_PASSWORD               ""
#define OTA_PORT                   3232
#define WIFI_CONNECT_TIMEOUT_MS    15000UL
#define WIFI_RECONNECT_INTERVAL_MS 60000UL
#define OTA_TASK_IDLE_DELAY_MS     500

OTAContext otaCtx = {
  .state                = OTA_STATE_IDLE,
  .lastProgress         = 0,
  .lastErrorCode        = 0,
  .wifiEnabled          = false,
  .lastConnectAttemptMs = 0,
  .totalBytesToFlash    = 0,
  .bytesDoneFlash       = 0,
  .lastErrorMsg         = {}
};

TaskHandle_t hOTATask = nullptr;
extern TaskHandle_t hSensorTask;

static void otaConfigureWiFiCoex() {
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

static bool otaConnectWiFi() {
  otaCtx.state = OTA_STATE_CONNECTING;
  
  xSemaphoreTake(configMutex, portMAX_DELAY);
  String ssid = cfg.wifiSSID;
  String pass = cfg.wifiPass;
  xSemaphoreGive(configMutex);

  if (ssid.length() == 0) {
    Serial.println(F("[OTA] WiFi SSID empty. Cannot start OTA client."));
    otaCtx.state = OTA_STATE_ERROR;
    strcpy(otaCtx.lastErrorMsg, "No WiFi config");
    return false;
  }

  Serial.printf("[OTA] Connecting to WiFi '%s'...\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  
  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startMs > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println(F("[OTA] WiFi connection timed out."));
      WiFi.disconnect(true);
      otaCtx.state = OTA_STATE_ERROR;
      strcpy(otaCtx.lastErrorMsg, "WiFi connection timeout");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }

  otaConfigureWiFiCoex();
  Serial.printf("[OTA] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

static void otaSuspendRobot() {
  digitalWrite(OE_PIN, HIGH);
  Serial.println(F("[OTA] SAFETY: OE=HIGH — Servo torque cut."));

  if (hKinTask != nullptr) {
    vTaskSuspend(hKinTask);
    Serial.println(F("[OTA] SAFETY: KinTask (Core1) suspended."));
  }

  if (hSensorTask != nullptr) {
    vTaskSuspend(hSensorTask);
    Serial.println(F("[OTA] SAFETY: SensorTask (Core0) suspended."));
  }

  vTaskDelay(pdMS_TO_TICKS(50));
  Serial.println(F("[OTA] Robot safely stopped. Ready to flash."));
}

static void otaResumeRobot() {
  if (hSensorTask != nullptr) {
    vTaskResume(hSensorTask);
    Serial.println(F("[OTA] SensorTask (Core0) resumed."));
  }
  vTaskDelay(pdMS_TO_TICKS(20));

  if (hKinTask != nullptr) {
    vTaskResume(hKinTask);
    Serial.println(F("[OTA] KinTask (Core1) resumed."));
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  digitalWrite(OE_PIN, LOW);
  Serial.println(F("[OTA] SAFETY: OE=LOW — Servo torque restored."));
}

static void otaRegisterCallbacks() {
  ArduinoOTA.onStart([]() {
    otaCtx.state        = OTA_STATE_STARTING;
    otaCtx.lastProgress = 0;
    sysState.otaActive  = true;

    const char* typeName = (ArduinoOTA.getCommand() == U_FLASH)
                           ? "Sketch (Flash)"
                           : "Filesystem (SPIFFS/LittleFS)";

    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.printf( "║  [OTA] FLASHING SEQUENCE STARTED       ║\n");
    Serial.printf( "║  Type  : %-30s║\n", typeName);
    Serial.println(F("╚════════════════════════════════════════╝"));

    otaSuspendRobot();
    otaCtx.state = OTA_STATE_FLASHING;
    Serial.println(F("[OTA] Writing flash..."));
  });

  ArduinoOTA.onEnd([]() {
    otaCtx.state        = OTA_STATE_DONE;
    otaCtx.lastProgress = 100;

    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.println(F("║  [OTA] FLASHING SUCCESSFUL!            ║"));
    Serial.println(F("║  System restarting in 3 seconds...     ║"));
    Serial.println(F("╚════════════════════════════════════════╝\n"));

    Serial.flush();
  });

  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    uint8_t pct = (total > 0) ? (uint8_t)((done * 100UL) / total) : 0;

    if (pct >= otaCtx.lastProgress + 5 || pct == 100) {
      otaCtx.lastProgress    = pct;
      otaCtx.bytesDoneFlash  = done;
      otaCtx.totalBytesToFlash = total;

      const int BAR_W = 20;
      int filled = (pct * BAR_W) / 100;
      char bar[BAR_W + 1];
      for (int i = 0; i < BAR_W; i++) bar[i] = (i < filled) ? '#' : '-';
      bar[BAR_W] = '\0';

      Serial.printf("[OTA] Progress: %3d%%  [%s]  %u / %u bytes\n",
                    pct, bar, done, total);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaCtx.state       = OTA_STATE_ERROR;
    otaCtx.lastErrorCode = (uint32_t)error;
    sysState.otaActive = false;

    const char* errStr;
    switch (error) {
      case OTA_AUTH_ERROR:    errStr = "Auth error"; break;
      case OTA_BEGIN_ERROR:   errStr = "Begin error";        break;
      case OTA_CONNECT_ERROR: errStr = "Connect error";                         break;
      case OTA_RECEIVE_ERROR: errStr = "Receive error";                        break;
      case OTA_END_ERROR:     errStr = "End error (CRC mismatch?)";       break;
      default:                errStr = "Unknown error";                          break;
    }

    snprintf(otaCtx.lastErrorMsg, sizeof(otaCtx.lastErrorMsg),
             "Code=%u: %s", (unsigned)error, errStr);

    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.printf( "║  [OTA] FLASHING ERROR                  ║\n");
    Serial.printf( "║  %s\n", otaCtx.lastErrorMsg);
    Serial.println(F("╚════════════════════════════════════════╝"));

    if (otaCtx.state == OTA_STATE_FLASHING ||
        otaCtx.state == OTA_STATE_STARTING) {
      Serial.println(F("[OTA] Initiating recovery..."));
      otaResumeRobot();
    }

    otaCtx.state = OTA_STATE_READY;
  });
}

void setupOTA() {
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }
  otaRegisterCallbacks();
  Serial.printf("[OTA] Configured. Host='%s' Port=%d\n", OTA_HOSTNAME, OTA_PORT);
}

void otaEnable() {
  if (otaCtx.wifiEnabled) return;
  otaCtx.wifiEnabled = true;
  Serial.println(F("[OTA] Enabled. Waiting for WiFi and connection..."));
}

void otaDisable() {
  if (!otaCtx.wifiEnabled) return;
  otaCtx.wifiEnabled = false;
  ArduinoOTA.end();
  WiFi.disconnect(true);
  otaCtx.state = OTA_STATE_IDLE;
  sysState.otaActive = false;
  Serial.println(F("[OTA] Disabled. WiFi disconnected."));
}

void otaPrintStatus() {
  Serial.println(F("┌─── OTA Status ───────────────────────────────┐"));
  const char* states[] = {
    "IDLE", "CONNECTING", "READY", "STARTING", "FLASHING", "DONE", "ERROR"
  };
  Serial.printf("│ Mode      : %-32s│\n", otaCtx.wifiEnabled ? "ACTIVE (WiFi On)" : "INACTIVE (WiFi Off)");
  Serial.printf("│ State     : %-32s│\n", states[(int)otaCtx.state]);
  Serial.printf("│ IP Address: %-32s│\n", WiFi.localIP().toString().c_str());
  Serial.printf("│ Progress  : %-32d│\n", otaCtx.lastProgress);
  if (otaCtx.state == OTA_STATE_ERROR) {
    Serial.printf("│ Error     : %-32s│\n", otaCtx.lastErrorMsg);
  }
  Serial.println(F("└──────────────────────────────────────────────┘"));
}

void taskOTA(void* param) {
  Serial.println(F("[CORE0] taskOTA started."));

  while (true) {
    if (!otaCtx.wifiEnabled) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (otaCtx.state == OTA_STATE_FLASHING ||
        otaCtx.state == OTA_STATE_DONE) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (WiFi.status() != WL_CONNECTED) {
      uint32_t now = millis();
      if ((now - otaCtx.lastConnectAttemptMs) >= WIFI_RECONNECT_INTERVAL_MS
          || otaCtx.lastConnectAttemptMs == 0) {
        otaCtx.lastConnectAttemptMs = now;

        if (otaConnectWiFi()) {
          ArduinoOTA.begin();
          otaCtx.state = OTA_STATE_READY;
          Serial.println(F("[OTA] Listening active. Ready to flash."));
          otaPrintStatus();
        }
      }
      vTaskDelay(pdMS_TO_TICKS(OTA_TASK_IDLE_DELAY_MS));
      continue;
    }

    otaCtx.state = OTA_STATE_READY;
    ArduinoOTA.handle();

    static uint32_t lastRSSIMs = 0;
    if ((millis() - lastRSSIMs) > 300000UL) {
      lastRSSIMs = millis();
      Serial.printf("[OTA] WiFi RSSI=%d dBm IP=%s\n",
                    WiFi.RSSI(),
                    WiFi.localIP().toString().c_str());
    }

    vTaskDelay(pdMS_TO_TICKS(OTA_TASK_IDLE_DELAY_MS));
  }
}
