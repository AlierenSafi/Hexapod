// ════════════════════════════════════════════════════════════════
//  hexapod_wifi.ino  ·  WiFi + WebSocket Server  ·  v3.1.0
//
//  Bu modül WiFi STA/AP yönetimi ve WebSocket haberleşmesini
//  sağlar. WebSocketsServer (Markus Sattler) kütüphanesi kullanır.
//
//  Endpoint: ws://IP:81
//  Protokol: JSON (komut ve telemetri)
//
//  Bağlantı Durumları:
//    DISCONNECTED → CONNECTING → CONNECTED
//
//  Özellikler:
//    • STA mode (NVS'den credential)
//    • AP mode fallback (credential yoksa)
//    • Auto-reconnect with exponential backoff
//    • Ping/pong heartbeat (30 sn)
//    • Max 5 eşzamanlı client
// ════════════════════════════════════════════════════════════════

#include <WebSocketsServer.h>

// ════════════════════════════════════════════════════════════════
// SABİTLER
// ════════════════════════════════════════════════════════════════

#define WS_PORT               81
#define WS_MAX_CLIENTS        5
#define WS_PING_INTERVAL_MS   30000
#define WIFI_CONNECT_TIMEOUT  15000
#define WIFI_RECONNECT_DELAY  5000

// AP Mode (fallback) ayarları
#define AP_SSID               "Hexapod-Setup"
#define AP_PASS               "12345678"
#define AP_CHANNEL            6
#define AP_MAX_CLIENTS        4

// ════════════════════════════════════════════════════════════════
// GLOBAL DEĞİŞKENLER
// ════════════════════════════════════════════════════════════════

// WiFiState enum'u ve wifiState degiskeni hexapod_esp32_v3.ino'da tanımlı
extern WiFiState wifiState;

static WebSocketsServer webSocket = WebSocketsServer(WS_PORT);
static uint32_t lastReconnectAttempt = 0;
static uint32_t reconnectDelay = WIFI_RECONNECT_DELAY;
static bool apModeActive = false;

// ════════════════════════════════════════════════════════════════
// İLERİ BİLDİRİMLER
// ════════════════════════════════════════════════════════════════

void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void handleJsonCommand(const char* json, size_t len, uint8_t clientNum);
void handleCmdMotion(JsonDocument& doc, uint8_t clientNum);
void handleCmdGait(JsonDocument& doc, uint8_t clientNum);
void handleCmdParamSet(JsonDocument& doc, uint8_t clientNum);
void handleCmdParamBulk(JsonDocument& doc, uint8_t clientNum);
void handleCmdSystem(JsonDocument& doc, uint8_t clientNum);
void handleCmdOTA(JsonDocument& doc, uint8_t clientNum);
void sendConfigToClient(uint8_t clientNum);
void sendError(uint8_t clientNum, const char* msg);
void sendAck(uint8_t clientNum, const char* action);

// ════════════════════════════════════════════════════════════════
// WiFi BAĞLANTI YÖNETİMİ
// ════════════════════════════════════════════════════════════════

void wifiInit() {
  Serial.println(F("\n[WiFi] Modül başlatılıyor..."));
  
  // WiFi credential'ları yükle (eğer Faz 1'de yüklenmediyse)
  if (cfg.wifiSSID[0] == '\0') {
    loadWiFiCredentials();
  }
  
  // STA mode dene (credential varsa)
  if (cfg.wifiSSID[0] != '\0') {
    wifiStartSTA();
  } else {
    Serial.println(F("[WiFi] Credential bulunamadı — AP mode başlatılıyor."));
    wifiStartAP();
  }
  
  // WebSocket server başlat
  webSocket.begin();
  webSocket.onEvent(handleWebSocketEvent);
  
  Serial.printf("[WiFi] WebSocket server başlatıldı: ws://%s:%d\n",
                apModeActive ? WiFi.softAPIP().toString().c_str() : "(connecting)",
                WS_PORT);
}

void wifiStartSTA() {
  if (wifiState == WIFI_CONNECTING || wifiState == WIFI_CONNECTED) {
    return;
  }
  
  wifiState = WIFI_CONNECTING;
  apModeActive = false;
  
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("Hexapod-ESP32");
  
  Serial.printf("[WiFi] STA bağlanıyor: SSID='%s'\n", cfg.wifiSSID);
  
  if (cfg.wifiPass[0] != '\0') {
    WiFi.begin(cfg.wifiSSID, cfg.wifiPass);
  } else {
    WiFi.begin(cfg.wifiSSID);  // Açık ağ
  }
  
  lastReconnectAttempt = millis();
}

void wifiStartAP() {
  wifiState = WIFI_AP_MODE;
  apModeActive = true;
  
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), 
                    IPAddress(192, 168, 4, 1), 
                    IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, false, AP_MAX_CLIENTS);
  
  Serial.println(F("[WiFi] AP Mode aktif:"));
  Serial.printf("  SSID: %s\n", AP_SSID);
  Serial.printf("  Pass: %s\n", AP_PASS);
  Serial.printf("  IP:   %s\n", WiFi.softAPIP().toString().c_str());
}

void wifiLoop() {
  // STA mode bağlantı kontrolü
  if (!apModeActive) {
    if (WiFi.status() == WL_CONNECTED) {
      if (wifiState != WIFI_CONNECTED) {
        wifiState = WIFI_CONNECTED;
        reconnectDelay = WIFI_RECONNECT_DELAY;  // Reset backoff
        Serial.printf("[WiFi] Bağlandı! IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(),
                      WiFi.RSSI());
        sysState.wifiRSSI = WiFi.RSSI();
      }
    } else {
      if (wifiState == WIFI_CONNECTED) {
        Serial.println(F("[WiFi] Bağlantı koptu! Yeniden bağlanılıyor..."));
        wifiState = WIFI_DISCONNECTED;
      }
      
      // Reconnect with backoff
      if (millis() - lastReconnectAttempt > reconnectDelay) {
        lastReconnectAttempt = millis();
        reconnectDelay = min(reconnectDelay * 2, 60000UL);  // Max 60s
        WiFi.reconnect();
      }
    }
  }
  
  // WebSocket loop
  webSocket.loop();
}

// ════════════════════════════════════════════════════════════════
// WEBSOCKET OLAY İŞLEYİCİLERİ
// ════════════════════════════════════════════════════════════════

void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u bağlandı from %s\n", 
                    num, webSocket.remoteIP(num).toString().c_str());
      
      // Hoşgeldin mesajı + config gönder
      {
        StaticJsonDocument<512> doc;
        doc["t"] = "welcome";
        doc["ver"] = "3.1.0";
        doc["uptime"] = millis();
        
        char buf[512];
        serializeJson(doc, buf, sizeof(buf));
        webSocket.sendTXT(num, buf);
        
        sendConfigToClient(num);
      }
      break;
      
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u bağlantı kesti\n", num);
      break;
      
    case WStype_TEXT:
      // JSON komut işle
      handleJsonCommand((char*)payload, length, num);
      break;
      
    case WStype_PING:
      // Pong otomatik gönderilir
      break;
      
    case WStype_ERROR:
      Serial.printf("[WS] Client #%u hata\n", num);
      break;
  }
}

void handleJsonCommand(const char* json, size_t len, uint8_t clientNum) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, json);
  
  if (err) {
    Serial.printf("[WS] JSON parse hatası: %s\n", err.c_str());
    sendError(clientNum, "Invalid JSON");
    return;
  }
  
  const char* cmd = doc["cmd"];
  if (!cmd) {
    sendError(clientNum, "Missing 'cmd' field");
    return;
  }
  
  // Komut kaynağını işaretle
  lastSrc = SRC_WS;
  sysState.lastCmdMs = millis();
  sysState.commTimeout = false;
  
  Serial.printf("[WS] Komut: %s\n", cmd);
  
  // Komut dispatch
  if (strcmp(cmd, "motion") == 0) {
    handleCmdMotion(doc, clientNum);
  }
  else if (strcmp(cmd, "gait") == 0) {
    handleCmdGait(doc, clientNum);
  }
  else if (strcmp(cmd, "param.set") == 0) {
    handleCmdParamSet(doc, clientNum);
  }
  else if (strcmp(cmd, "param.bulk") == 0) {
    handleCmdParamBulk(doc, clientNum);
  }
  else if (strcmp(cmd, "system") == 0) {
    handleCmdSystem(doc, clientNum);
  }
  else if (strcmp(cmd, "get_config") == 0) {
    sendConfigToClient(clientNum);
  }
  else if (strcmp(cmd, "ota") == 0) {
    handleCmdOTA(doc, clientNum);
  }
  else {
    sendError(clientNum, "Unknown command");
  }
}

// ── motion: Hareket komutu ──────────────────────────────────────
void handleCmdMotion(JsonDocument& doc, uint8_t clientNum) {
  float x = doc["x"] | 0;
  float y = doc["y"] | 0;
  float yaw = doc["yaw"] | 0;
  
  // -100..+100 aralığına sınırla
  x = constrain(x, -100.0f, 100.0f);
  y = constrain(y, -100.0f, 100.0f);
  yaw = constrain(yaw, -100.0f, 100.0f);
  
  // RadioPacket oluştur
  RadioPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.motion.type = 0x01;  // Yürü
  pkt.motion.xRate = (int8_t)x;
  pkt.motion.yRate = (int8_t)y;
  pkt.motion.yawRate = (int8_t)yaw;
  
  // Checksum hesapla
  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt.raw[i];
  pkt.raw[9] = chk;
  
  // Mevcut dispatcher'a yönlendir
  dispatchMotionCmd(pkt, SRC_WS);
  
  sendAck(clientNum, "motion");
}

// ── gait: Gait değiştir ─────────────────────────────────────────
void handleCmdGait(JsonDocument& doc, uint8_t clientNum) {
  const char* type = doc["type"];
  if (!type) {
    sendError(clientNum, "Missing 'type' field");
    return;
  }
  
  GaitType newGait = TRIPOD;
  if (strcmp(type, "ripple") == 0) newGait = RIPPLE;
  else if (strcmp(type, "wave") == 0) newGait = WAVE;
  
  // RadioPacket oluştur
  RadioPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.motion.type = 0x03;  // Gait değiştir
  pkt.motion.xRate = (int8_t)newGait;
  
  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt.raw[i];
  pkt.raw[9] = chk;
  
  dispatchMotionCmd(pkt, SRC_WS);
  
  StaticJsonDocument<128> resp;
  resp["ack"] = "gait";
  resp["type"] = type;
  
  char buf[128];
  serializeJson(resp, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

// ── param.set: Tek parametre güncelle ──────────────────────────
void handleCmdParamSet(JsonDocument& doc, uint8_t clientNum) {
  const char* key = doc["key"];
  float value = doc["value"] | 0.0f;
  bool save = doc["save"] | false;
  
  if (!key) {
    sendError(clientNum, "Missing 'key' field");
    return;
  }
  
  bool ok = updateParameter(String(key), value, save);
  
  StaticJsonDocument<128> resp;
  resp["ack"] = "param.set";
  resp["key"] = key;
  resp["value"] = value;
  resp["save"] = save;
  resp["ok"] = ok;
  
  char buf[128];
  serializeJson(resp, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

// ── param.bulk: Toplu parametre güncelle ───────────────────────
void handleCmdParamBulk(JsonDocument& doc, uint8_t clientNum) {
  JsonArray params = doc["params"];
  bool save = doc["save"] | false;
  
  if (!params) {
    sendError(clientNum, "Missing 'params' array");
    return;
  }
  
  int count = 0;
  for (JsonObject param : params) {
    const char* k = param["k"];
    float v = param["v"] | 0.0f;
    if (k) {
      updateParameter(String(k), v, false);  // Önce RAM'e
      count++;
    }
  }
  
  if (save) {
    saveConfiguration();
  }
  
  StaticJsonDocument<128> resp;
  resp["ack"] = "param.bulk";
  resp["count"] = count;
  resp["saved"] = save;
  
  char buf[128];
  serializeJson(resp, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

// ── system: Sistem komutları ────────────────────────────────────
void handleCmdSystem(JsonDocument& doc, uint8_t clientNum) {
  const char* action = doc["action"];
  if (!action) {
    sendError(clientNum, "Missing 'action' field");
    return;
  }
  
  if (strcmp(action, "save_nvs") == 0) {
    saveConfiguration();
    sendAck(clientNum, "save_nvs");
  }
  else if (strcmp(action, "load_nvs") == 0) {
    loadConfiguration();
    configChanged = true;
    sendAck(clientNum, "load_nvs");
  }
  else if (strcmp(action, "reset_defaults") == 0) {
    resetToDefaults();
    sendAck(clientNum, "reset_defaults");
  }
  else if (strcmp(action, "enable_servos") == 0) {
    digitalWrite(OE_PIN, LOW);
    sendAck(clientNum, "enable_servos");
  }
  else if (strcmp(action, "disable_servos") == 0) {
    digitalWrite(OE_PIN, HIGH);
    sendAck(clientNum, "disable_servos");
  }
  else if (strcmp(action, "sitdown") == 0) {
    sitDown();
    sendAck(clientNum, "sitdown");
  }
  else {
    sendError(clientNum, "Unknown system action");
  }
}

// ── ota: OTA komutları ──────────────────────────────────────────
void handleCmdOTA(JsonDocument& doc, uint8_t clientNum) {
  const char* action = doc["action"];
  if (!action) {
    sendError(clientNum, "Missing 'action' field");
    return;
  }
  
  if (strcmp(action, "enable") == 0) {
    otaEnable();
    sendAck(clientNum, "ota_enabled");
  }
  else if (strcmp(action, "disable") == 0) {
    otaDisable();
    sendAck(clientNum, "ota_disabled");
  }
  else if (strcmp(action, "status") == 0) {
    otaPrintStatus();
    sendAck(clientNum, "ota_status");
  }
  else {
    sendError(clientNum, "Unknown OTA action");
  }
}

// ════════════════════════════════════════════════════════════════
// YARDIMCI FONKSİYONLAR
// ════════════════════════════════════════════════════════════════

void sendConfigToClient(uint8_t clientNum) {
  // Thread-safe config snapshot al
  RobotSettings snap;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  snap = cfg;
  xSemaphoreGive(configMutex);
  
  StaticJsonDocument<1024> doc;
  doc["t"] = "config";
  
  JsonObject mech = doc.createNestedObject("mechanics");
  mech["coxa"] = snap.coxaLen;
  mech["femur"] = snap.femurLen;
  mech["tibia"] = snap.tibiaLen;
  mech["stanceR"] = snap.stanceRadius;
  mech["stanceH"] = snap.stanceHeight;
  
  JsonObject step = doc.createNestedObject("step");
  step["height"] = snap.stepHeight;
  step["length"] = snap.stepLength;
  
  JsonObject gait = doc.createNestedObject("gait");
  gait["speed"] = snap.gaitSpeed;
  gait["swingRatio"] = snap.swingRatio;
  gait["type"] = (int)snap.gaitType;
  
  JsonObject pid = doc.createNestedObject("pid");
  pid["kp"] = snap.kp;
  pid["ki"] = snap.ki;
  pid["kd"] = snap.kd;
  pid["limit"] = snap.levelingLimit;
  
  JsonObject imu = doc.createNestedObject("imu");
  imu["alpha"] = snap.compAlpha;
  imu["leveling"] = snap.levelingEnabled;
  
  JsonObject batt = doc.createNestedObject("battery");
  batt["warn"] = snap.battWarnVolt;
  batt["crit"] = snap.battCritVolt;
  batt["cutoff"] = snap.battCutoffVolt;
  
  JsonObject tele = doc.createNestedObject("telemetry");
  tele["rate"] = snap.telemetryRateMs;
  tele["timeout"] = snap.commTimeoutMs;
  
  char buf[1024];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
  
  Serial.printf("[WS] Config gönderildi (%u byte)\n", n);
}

void sendError(uint8_t clientNum, const char* msg) {
  StaticJsonDocument<128> doc;
  doc["t"] = "error";
  doc["msg"] = msg;
  
  char buf[128];
  serializeJson(doc, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

void sendAck(uint8_t clientNum, const char* action) {
  StaticJsonDocument<128> doc;
  doc["t"] = "ack";
  doc["action"] = action;
  doc["ts"] = millis();
  
  char buf[128];
  serializeJson(doc, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

void broadcastTelemetry(const char* json) {
  webSocket.broadcastTXT(json);
}

bool hasAuthenticatedClient() {
  // Şimdilik her zaman true döndürüyoruz
  return true;
}

uint32_t getClientCount() {
  // WebSocketsServer kütüphanesinde client sayısı doğrudan alınamıyor
  // Şimdilik 0 döndürüyoruz
  return 0;
}
