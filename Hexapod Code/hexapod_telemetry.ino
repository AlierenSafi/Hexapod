// ════════════════════════════════════════════════════════════════
//  hexapod_telemetry.ino  ·  Telemetri Sistemi  ·  v3.1.0
//
//  Bu modül ESP32'den Processing GUI'ye anlık veri akışını sağlar.
//  Üç kanal mimarisi:
//    • Fast (10Hz): IMU, gait state, leg positions
//    • Slow (1Hz):  batarya, WiFi, sistem durumu
//    • Event (anlık): uyarılar, hatalar, config ack
//
//  Format: JSON (ArduinoJson kullanmadan, sprintf ile)
//  Hedef: ~500-600 byte/paket, 10Hz'de heap güvenli
// ════════════════════════════════════════════════════════════════

#include <ArduinoJson.h>  // Processing'e uyumlu JSON

// ════════════════════════════════════════════════════════════════
// İLERİ BİLDİRİMLER (hexapod_wifi.ino'da tanımlı)
// ════════════════════════════════════════════════════════════════

extern void broadcastTelemetry(const char* json);

// ════════════════════════════════════════════════════════════════
// SABİTLER ve BUFFER'LAR
// ════════════════════════════════════════════════════════════════

static char teleBuffer[768];           // JSON serialize buffer
static uint32_t lastFastTeleMs = 0;    // Son fast telemetri
static uint32_t lastSlowTeleMs = 0;    // Son slow telemetri
static uint8_t  teleCounter = 0;       // 10 döngüde bir slow gönder

// ════════════════════════════════════════════════════════════════
// FAST TELEMETRI (10Hz)
// ════════════════════════════════════════════════════════════════

void sendFastTelemetry() {
  uint32_t now = millis();
  if (now - lastFastTeleMs < cfg.telemetryRateMs) return;
  lastFastTeleMs = now;
  
  // Thread-safe snapshot al
  RobotSettings snap;
  IMUData imuSnap;
  LegState legSnap[6];
  
  xSemaphoreTake(configMutex, portMAX_DELAY);
  snap = cfg;
  xSemaphoreGive(configMutex);
  
  xSemaphoreTake(imuMutex, portMAX_DELAY);
  imuSnap = imuData;
  xSemaphoreGive(imuMutex);
  
  // Leg state snapshot (mutex gerekmez, sadece Core1 yazıyor, okuma atomic)
  for (int i = 0; i < 6; i++) {
    legSnap[i] = legs[i];
  }
  
  // Manueller yerine snprintf kullanarak JSON formatını doğrudan oluştur
  // ArduinoJson ve stack kopyalama yükünü sıfıra indirir (768 Byte Stack eliminasyonu)
  int len = snprintf(teleBuffer, sizeof(teleBuffer),
    "{\"t\":\"fast\",\"ts\":%lu,\"imu\":{\"p\":%.2f,\"r\":%.2f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f},"
    "\"gait\":%d,\"moving\":%d,\"legs\":[",
    (unsigned long)now,
    imuSnap.pitch, imuSnap.roll,
    imuSnap.ax, imuSnap.ay, imuSnap.az,
    (int)currentGait, isMoving() ? 1 : 0
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

// ════════════════════════════════════════════════════════════════
// SLOW TELEMETRI (1Hz)
// ════════════════════════════════════════════════════════════════

void sendSlowTelemetry() {
  uint32_t now = millis();
  if (now - lastSlowTeleMs < 1000) return;
  lastSlowTeleMs = now;
  
  // Güncelle
  sysState.uptimeMs = now;
  sysState.freeHeap = esp_get_free_heap_size();
  sysState.wifiRSSI = WiFi.RSSI();
  
  StaticJsonDocument<512> doc;
  
  doc["t"] = "slow";
  doc["ts"] = now;
  
  // Batarya
  JsonObject batt = doc.createNestedObject("batt");
  batt["v"] = round2(battState.voltage);
  batt["pct"] = (int)battState.percentage;
  batt["lvl"] = (int)battState.level;
  
  // WiFi
  JsonObject wifi = doc.createNestedObject("wifi");
  wifi["rssi"] = sysState.wifiRSSI;
  wifi["connected"] = (WiFi.status() == WL_CONNECTED);
  
  // Sistem
  JsonObject sys = doc.createNestedObject("sys");
  sys["up"] = now / 1000;  // saniye cinsinden
  sys["heap"] = sysState.freeHeap;
  sys["ct"] = sysState.commTimeout;
  sys["fault"] = sysState.faultCode;
  
  // Task timing (son 1 sn içinde)
  sys["clients"] = getClientCount();
  
  size_t n = serializeJson(doc, teleBuffer, sizeof(teleBuffer));
  if (n < sizeof(teleBuffer) - 1) {
    broadcastTelemetry(teleBuffer);
  }
}

// ════════════════════════════════════════════════════════════════
// EVENT TELEMETRI (Anlık)
// ════════════════════════════════════════════════════════════════

void sendEvent(const char* type, const char* msg) {
  StaticJsonDocument<256> doc;
  
  doc["t"] = "event";
  doc["ts"] = millis();
  doc["type"] = type;
  doc["msg"] = msg;
  
  size_t n = serializeJson(doc, teleBuffer, sizeof(teleBuffer));
  if (n < sizeof(teleBuffer) - 1) {
    broadcastTelemetry(teleBuffer);
  }
}

void sendEventf(const char* type, const char* fmt, ...) {
  char msg[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  
  sendEvent(type, msg);
}

// Belirli event tipleri için kısayollar
void sendLowVoltageWarning(float voltage) {
  sendEventf("batt_warn", "Düşük batarya: %.2fV", voltage);
}

void sendCriticalVoltage(float voltage) {
  sendEventf("batt_crit", "Kritik batarya: %.2fV - SitDown", voltage);
}

void sendCommTimeout() {
  sendEvent("comm_timeout", "Haberleşme timeout - Motion stop");
}

void sendConfigAck(const char* action) {
  sendEventf("config_ack", "Config %s", action);
}

void sendOTAProgress(uint8_t percent) {
  StaticJsonDocument<128> doc;
  doc["t"] = "event";
  doc["type"] = "ota_progress";
  doc["pct"] = percent;
  
  size_t n = serializeJson(doc, teleBuffer, sizeof(teleBuffer));
  if (n < sizeof(teleBuffer) - 1) {
    broadcastTelemetry(teleBuffer);
  }
}

void sendOTAError(const char* msg) {
  sendEvent("ota_error", msg);
}

// ════════════════════════════════════════════════════════════════
// ANA TELEMETRI DÖNGÜSÜ (taskSensorComm'dan çağrılır)
// ════════════════════════════════════════════════════════════════

void telemetryLoop() {
  // Her çağrıda fast telemetri kontrolü
  sendFastTelemetry();
  
  // Her 10 çağrıda bir (100ms × 10 = 1000ms) slow telemetri
  teleCounter++;
  if (teleCounter >= 10) {
    teleCounter = 0;
    sendSlowTelemetry();
  }
}

// ════════════════════════════════════════════════════════════════
// YARDIMCI FONKSİYONLAR
// ════════════════════════════════════════════════════════════════

// Yuvarlama fonksiyonları (JSON boyutunu azaltmak için)
static inline float round1(float v) { return round(v * 10.0f) / 10.0f; }
static inline float round2(float v) { return round(v * 100.0f) / 100.0f; }
static inline float round3(float v) { return round(v * 1000.0f) / 1000.0f; }

// Robot hareket halinde mi?
static bool isMoving() {
  xSemaphoreTake(cmdMutex, portMAX_DELAY);
  bool moving = (ctrlPkt.motion.type == 0x01) &&
                (abs(ctrlPkt.motion.xRate) > 5 ||
                 abs(ctrlPkt.motion.yRate) > 5 ||
                 abs(ctrlPkt.motion.yawRate) > 5);
  xSemaphoreGive(cmdMutex);
  return moving;
}
