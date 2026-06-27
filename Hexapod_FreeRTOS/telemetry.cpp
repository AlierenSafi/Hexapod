#include "telemetry.h"
#include "communication.h"
#include "watchdog_safety.h"
#include "battery_monitor.h"
#include "imu_leveling.h"
#include "servo_drivers.h"
#include "kinematics.h"
#include <WiFi.h>

char teleBuffer[1024];

static uint32_t lastFastTeleMs = 0;

void sendFastTelemetry() {
  uint32_t now = millis();
  
  RobotSettings snap;
  IMUData imuSnap;
  LegState legSnap[6];
  
  xSemaphoreTake(configMutex, portMAX_DELAY);
  snap = cfg;
  xSemaphoreGive(configMutex);
  
  xSemaphoreTake(imuMutex, portMAX_DELAY);
  imuSnap = imuData;
  xSemaphoreGive(imuMutex);
  
  for (int i = 0; i < 6; i++) {
    legSnap[i] = legs[i];
  }
  
  int len = snprintf(teleBuffer, sizeof(teleBuffer),
    "{\"t\":\"fast\",\"ts\":%lu,\"imu\":{\"p\":%.2f,\"r\":%.2f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f},"
    "\"gait\":%d,\"moving\":%d,\"legs\":[",
    (unsigned long)now,
    imuSnap.pitch, imuSnap.roll,
    imuSnap.ax, imuSnap.ay, imuSnap.az,
    (int)currentGait, sysState.moving ? 1 : 0
  );

  for (int i = 0; i < 6; i++) {
    len += snprintf(teleBuffer + len, sizeof(teleBuffer) - len,
      "{\"ph\":%.2f,\"sw\":%d,\"fx\":%.1f,\"fy\":%.1f,\"fz\":%.1f,\"cx\":%.1f,\"fm\":%.1f,\"tb\":%.1f}%s",
      legSnap[i].phase,
      legSnap[i].isSwing ? 1 : 0,
      legSnap[i].footPos.x, legSnap[i].footPos.y, legSnap[i].footPos.z,
      legSnap[i].angles.coxa, legSnap[i].angles.femur, legSnap[i].angles.tibia,
      (i < 5) ? "," : ""
    );
  }

  snprintf(teleBuffer + len, sizeof(teleBuffer) - len, "]}");

  broadcastTelemetry(teleBuffer);
}

void sendSlowTelemetry() {
  uint32_t now = millis();
  uint32_t uptimeSec = now / 1000;
  
  sysState.freeHeap = esp_get_free_heap_size();
  sysState.wifiRSSI = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

  snprintf(teleBuffer, sizeof(teleBuffer),
    "{\"t\":\"slow\",\"ts\":%lu,\"batt\":{\"v\":%.2f,\"pct\":%d,\"lvl\":%d},"
    "\"wifi\":{\"rssi\":%d,\"connected\":%d},"
    "\"sys\":{\"up\":%lu,\"heap\":%lu,\"ct\":%d,\"fault\":%d,\"clients\":%lu}}",
    (unsigned long)now,
    battVoltage, (int)battPercent, (int)battFSMState,
    (int)sysState.wifiRSSI, (WiFi.status() == WL_CONNECTED) ? 1 : 0,
    (unsigned long)uptimeSec, (unsigned long)sysState.freeHeap,
    sysState.commTimeout ? 1 : 0, (int)sysState.faultCode,
    (unsigned long)getClientCount()
  );

  broadcastTelemetry(teleBuffer);
}

void sendEvent(const char* eventType, const char* message) {
  snprintf(teleBuffer, sizeof(teleBuffer),
    "{\"t\":\"event\",\"ts\":%lu,\"type\":\"%s\",\"msg\":\"%s\"}",
    (unsigned long)millis(), eventType, message
  );
  broadcastTelemetry(teleBuffer);
}

void sendCommTimeout() {
  sendEvent("comm_timeout", "Communication timeout - motion stopped");
}
