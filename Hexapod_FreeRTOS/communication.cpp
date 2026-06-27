#include "communication.h"
#include "servo_drivers.h" // For OE_PIN
#include "kinematics.h"    // For sitDown
#include "ota_manager.h"   // For otaEnable, otaDisable, otaPrintStatus
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <RF24.h>

#ifdef USE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
static BLECharacteristic* pBLEChar = nullptr;
volatile bool bleConnected = false;
#endif

// System states (externed from main/config/watchdog)
extern struct SystemState {
  uint32_t lastCmdMs;
  uint8_t  faultCode;
  bool     otaActive;
  bool     commTimeout;
} sysState;

// Global communication variables
WiFiState wifiState = WIFI_DISCONNECTED;
bool nrfAvailable = false;
CmdSource lastSrc = SRC_NONE;
const byte NRF_ADDRESS[6] = "Hexa1";

QueueHandle_t xQueueCmd = nullptr;

// NRF24 instance
static RF24 radio(5, 15); // CE=5, CSN=15

// WebSocketsServer instance
static WebSocketsServer webSocket = WebSocketsServer(WS_PORT);
static uint32_t lastReconnectAttempt = 0;
static uint32_t reconnectDelay = WIFI_RECONNECT_DELAY;
static bool apModeActive = false;
volatile uint32_t wsClientCount = 0;

// Internal helpers
static bool verifyChecksum(const RadioPacket& pkt) {
  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt.raw[i];
  return chk == pkt.raw[9];
}

void dispatchMotionCmd(const RadioPacket& pkt, CmdSource src) {
  lastSrc = src;
  sysState.lastCmdMs = millis();
  sysState.commTimeout = false;
  if (xQueueCmd != nullptr) {
    xQueueSend(xQueueCmd, &pkt, 0); // Non-blocking send
  }
}

static void parseBinaryPacket(const RadioPacket& pkt, CmdSource src) {
  if (!verifyChecksum(pkt)) {
    static uint32_t lastErrMs = 0;
    if (millis() - lastErrMs > 500) {
      Serial.printf("[COMM] Checksum error (src=%d)\n", (int)src);
      lastErrMs = millis();
    }
    return;
  }

  if (pkt.config.type == 0xF0) {
    updateParamById(pkt.config.paramId, pkt.config.value, pkt.config.flags);
    return;
  }

  switch (pkt.motion.type) {
    case 0x00: // STOP
    case 0x01: // WALK
    case 0x02: // TURN
    case 0x03: // CHANGE GAIT
      dispatchMotionCmd(pkt, src);
      break;

    case 0x10:
    {
      uint8_t leg  = pkt.motion.cal_leg;
      uint8_t jnt  = pkt.motion.cal_joint;
      int8_t  trim = (int8_t)pkt.motion.cal_trim;
      if (leg < 6 && jnt < 3) {
        xSemaphoreTake(configMutex, portMAX_DELAY);
        cfg.servoTrim[leg][jnt] = constrain(trim, (int8_t)-30, (int8_t)30);
        xSemaphoreGive(configMutex);
        if (hKinTask != nullptr) xTaskNotifyGive(hKinTask);
        syncHardware();
        Serial.printf("[COMM] Trim B%d E%d = %+d°\n", leg, jnt, (int)trim);
      }
      break;
    }
    case 0x11: saveConfiguration();  break;
    case 0x12: resetToDefaults();    break;
    case 0x20: otaEnable();      break;
    case 0x21: otaDisable();     break;
    case 0x22: otaPrintStatus(); break;
    default:
      Serial.printf("[COMM] Unknown type: 0x%02X\n", pkt.motion.type);
      break;
  }
}

static void parseLegacyPacket(const uint8_t* data, size_t len, CmdSource src) {
  if (len < 4) return;

  RadioPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.motion.type    = data[0];
  pkt.motion.xRate   = (int8_t)data[1];
  pkt.motion.yRate   = (int8_t)data[2];
  pkt.motion.yawRate = (int8_t)data[3];

  if (data[0] == 0x10 && len >= 4) {
    if (len >= 6) {
      pkt.motion.cal_leg   = data[1];
      pkt.motion.cal_joint = data[2];
      pkt.motion.cal_trim  = data[3];
    }
  }

  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt.raw[i];
  pkt.raw[9] = chk;

  parseBinaryPacket(pkt, src);
}

static void parseJsonPacket(const String& json) {
  auto extractStr = [&](const char* field) -> String {
    String pat = String("\"") + field + "\":\"";
    int i = json.indexOf(pat);
    if (i < 0) return String();
    i += pat.length();
    int j = json.indexOf('"', i);
    return (j > i) ? json.substring(i, j) : String();
  };
  auto extractFloat = [&](const char* field) -> float {
    String pat = String("\"") + field + "\":";
    int i = json.indexOf(pat);
    if (i < 0) return 0.0f;
    i += pat.length();
    int j = i;
    while (j < (int)json.length() && json[j] != ',' && json[j] != '}' && json[j] != ' ') j++;
    return json.substring(i, j).toFloat();
  };
  auto extractInt = [&](const char* field) -> int {
    return (int)extractFloat(field);
  };

  String key = extractStr("k");
  if (key.length() == 0) {
    Serial.println(F("[JSON] Error: 'k' field not found."));
    return;
  }

  if (key == "save")  { saveConfiguration();  return; }
  if (key == "load")  { loadConfiguration();
                        if (hKinTask != nullptr) xTaskNotifyGive(hKinTask); return; }
  if (key == "reset") { resetToDefaults();    return; }
  if (key == "print") { printConfiguration(); return; }

  float value   = extractFloat("v");
  bool  saveNVS = (extractInt("s") == 1);

  updateParameter(key, value, saveNVS);
}

// WebSocket support functions
static void sendError(uint8_t clientNum, const char* msg) {
  StaticJsonDocument<128> doc;
  doc["t"] = "error";
  doc["msg"] = msg;
  char buf[128];
  serializeJson(doc, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

static void sendAck(uint8_t clientNum, const char* action) {
  StaticJsonDocument<128> doc;
  doc["ack"] = action;
  doc["ok"] = true;
  char buf[128];
  serializeJson(doc, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

static void sendConfigToClient(uint8_t clientNum) {
  StaticJsonDocument<1024> doc;
  doc["t"] = "config";

  xSemaphoreTake(configMutex, portMAX_DELAY);
  doc["coxa"] = cfg.coxaLen;
  doc["femur"] = cfg.femurLen;
  doc["tibia"] = cfg.tibiaLen;
  doc["st_radius"] = cfg.stanceRadius;
  doc["st_height"] = cfg.stanceHeight;
  doc["step_h"] = cfg.stepHeight;
  doc["step_len"] = cfg.stepLength;
  doc["gait_spd"] = cfg.gaitSpeed;
  doc["swing_r"] = cfg.swingRatio;
  doc["gait_type"] = (int)cfg.gaitType;
  doc["kp"] = cfg.kp;
  doc["ki"] = cfg.ki;
  doc["kd"] = cfg.kd;
  doc["lev_lim"] = cfg.levelingLimit;
  doc["pid_ilim"] = cfg.pidIntLimit;
  doc["comp_a"] = cfg.compAlpha;
  doc["s_min"] = cfg.servoMin;
  doc["s_max"] = cfg.servoMax;
  doc["lev_en"] = cfg.levelingEnabled;
  doc["pr_en"] = cfg.phaseResetEnabled;
  doc["bat_warn"] = cfg.battWarnVolt;
  doc["bat_crit"] = cfg.battCritVolt;
  doc["bat_cut"] = cfg.battCutoffVolt;
  doc["tele_ms"] = cfg.telemetryRateMs;
  doc["comm_to"] = cfg.commTimeoutMs;
  doc["ssid"] = cfg.wifiSSID;

  JsonArray trims = doc.createNestedArray("trim");
  for (int l = 0; l < 6; l++) {
    JsonArray t = trims.createNestedArray();
    t.add(cfg.servoTrim[l][0]);
    t.add(cfg.servoTrim[l][1]);
    t.add(cfg.servoTrim[l][2]);
  }
  xSemaphoreGive(configMutex);

  char buf[1024];
  serializeJson(doc, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

static void handleCmdMotion(JsonDocument& doc, uint8_t clientNum) {
  float x = doc["x"] | 0.0f;
  float y = doc["y"] | 0.0f;
  float yaw = doc["yaw"] | 0.0f;

  x = constrain(x, -100.0f, 100.0f);
  y = constrain(y, -100.0f, 100.0f);
  yaw = constrain(yaw, -100.0f, 100.0f);

  RadioPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.motion.type = 0x01;
  pkt.motion.xRate = (int8_t)x;
  pkt.motion.yRate = (int8_t)y;
  pkt.motion.yawRate = (int8_t)yaw;

  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt.raw[i];
  pkt.raw[9] = chk;

  dispatchMotionCmd(pkt, SRC_WS);
  sendAck(clientNum, "motion");
}

static void handleCmdGait(JsonDocument& doc, uint8_t clientNum) {
  const char* type = doc["type"];
  if (!type) {
    sendError(clientNum, "Missing 'type' field");
    return;
  }

  GaitType newGait = TRIPOD;
  if (strcmp(type, "ripple") == 0) newGait = RIPPLE;
  else if (strcmp(type, "wave") == 0) newGait = WAVE;

  RadioPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.motion.type = 0x03;
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

static void handleCmdParamSet(JsonDocument& doc, uint8_t clientNum) {
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

static void handleCmdParamBulk(JsonDocument& doc, uint8_t clientNum) {
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
      updateParameter(String(k), v, false);
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

static void handleCmdSystem(JsonDocument& doc, uint8_t clientNum) {
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
    if (hKinTask != nullptr) xTaskNotifyGive(hKinTask);
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

static void handleCmdOTA(JsonDocument& doc, uint8_t clientNum) {
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

static void handleJsonCommand(const char* json, size_t len, uint8_t clientNum) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, json, len);

  if (err) {
    Serial.printf("[WS] JSON parse error: %s\n", err.c_str());
    sendError(clientNum, "Invalid JSON");
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) {
    sendError(clientNum, "Missing 'cmd' field");
    return;
  }

  lastSrc = SRC_WS;
  sysState.lastCmdMs = millis();
  sysState.commTimeout = false;

  Serial.printf("[WS] Cmd: %s\n", cmd);

  if (strcmp(cmd, "motion") == 0)             handleCmdMotion(doc, clientNum);
  else if (strcmp(cmd, "gait") == 0)          handleCmdGait(doc, clientNum);
  else if (strcmp(cmd, "param.set") == 0)     handleCmdParamSet(doc, clientNum);
  else if (strcmp(cmd, "param.bulk") == 0)    handleCmdParamBulk(doc, clientNum);
  else if (strcmp(cmd, "system") == 0)        handleCmdSystem(doc, clientNum);
  else if (strcmp(cmd, "get_config") == 0)    sendConfigToClient(clientNum);
  else if (strcmp(cmd, "ota") == 0)           handleCmdOTA(doc, clientNum);
  else                                        sendError(clientNum, "Unknown command");
}

static void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsClientCount++;
      Serial.printf("[WS] Client #%u connected from %s (Total: %u)\n",
                    num, webSocket.remoteIP(num).toString().c_str(), (unsigned int)wsClientCount);
      {
        StaticJsonDocument<512> doc;
        doc["t"] = "welcome";
        doc["ver"] = "3.2.0-freertos";
        doc["uptime"] = millis();
        char buf[512];
        serializeJson(doc, buf, sizeof(buf));
        webSocket.sendTXT(num, buf);
        sendConfigToClient(num);
      }
      break;

    case WStype_DISCONNECTED:
      if (wsClientCount > 0) wsClientCount--;
      Serial.printf("[WS] Client #%u disconnected (Total: %u)\n", num, (unsigned int)wsClientCount);
      break;

    case WStype_TEXT:
      handleJsonCommand((char*)payload, length, num);
      break;

    case WStype_PING:
      break;
    case WStype_PONG:
      break;
    case WStype_BIN:
      break;
    case WStype_ERROR:
      Serial.printf("[WS] Client #%u error\n", num);
      break;
  }
}

// Start Wi-Fi & WebSockets
void wifiInit() {
  WiFi.disconnect(true);
  delay(100);

  xSemaphoreTake(configMutex, portMAX_DELAY);
  String ssid = cfg.wifiSSID;
  String pass = cfg.wifiPass;
  xSemaphoreGive(configMutex);

  if (ssid.length() > 0) {
    Serial.printf("[WIFI] Connecting to STA: SSID='%s'\n", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    wifiState = WIFI_CONNECTING;
  } else {
    Serial.println(F("[WIFI] SSID is empty -> Starting AP Mode."));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
    apModeActive = true;
    wifiState = WIFI_AP_MODE;
    Serial.printf("[WIFI] AP Started. SSID='%s' IP='%s'\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }

  webSocket.begin();
  webSocket.onEvent(handleWebSocketEvent);
  Serial.println(F("[WIFI] WebSockets started on port 81"));
}

void wifiLoop() {
  webSocket.loop();

  if (apModeActive) return;

  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiState != WIFI_CONNECTED) {
      wifiState = WIFI_CONNECTED;
      Serial.printf("[WIFI] Connected! STA IP: '%s'\n", WiFi.localIP().toString().c_str());
    }
  } else {
    if (wifiState == WIFI_CONNECTED) {
      wifiState = WIFI_DISCONNECTED;
      Serial.println(F("[WIFI] Connection lost! STA disconnected."));
    }
    if (now - lastReconnectAttempt > reconnectDelay) {
      lastReconnectAttempt = now;
      Serial.println(F("[WIFI] Reconnecting..."));
      WiFi.begin();
    }
  }
}

void broadcastTelemetry(const char* json) {
  webSocket.broadcastTXT(json);
}

bool hasAuthenticatedClient() {
  return true;
}

uint32_t getClientCount() {
  return wsClientCount;
}

#ifdef USE_BLE
class HexapodBLECallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String raw = pChar->getValue();
    size_t len  = raw.length();
    if (len == 0) return;

    if (raw[0] == '{') {
      parseJsonPacket(raw);
      return;
    }
    if (len >= 10) {
      RadioPacket pkt;
      memset(&pkt, 0, sizeof(pkt));
      memcpy(pkt.raw, raw.c_str(), sizeof(pkt.raw));
      parseBinaryPacket(pkt, SRC_BLE);
      return;
    }
    parseLegacyPacket((const uint8_t*)raw.c_str(), len, SRC_BLE);
  }
};

class HexapodServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer* pSvr) override {
    bleConnected = true;
    Serial.println(F("[BLE] Connected."));
  }
  void onDisconnect(BLEServer* pSvr) override {
    bleConnected = false;
    RadioPacket stopPkt;
    memset(&stopPkt, 0, sizeof(stopPkt)); // type=0 = DUR
    dispatchMotionCmd(stopPkt, SRC_NONE);
    pSvr->startAdvertising();
    Serial.println(F("[BLE] Disconnected. Re-advertising..."));
  }
};

void bleInit() {
  BLEDevice::init("Hexapod-ESP32");
  BLEServer* pSvr = BLEDevice::createServer();
  pSvr->setCallbacks(new HexapodServerCallback());

  BLEService* pSvc = pSvr->createService(BLE_SERVICE_UUID);
  pBLEChar = pSvc->createCharacteristic(
    BLE_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pBLEChar->setCallbacks(new HexapodBLECallback());
  pBLEChar->addDescriptor(new BLE2902());

  pSvc->start();
  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  pAdv->start();

  Serial.println(F("[BLE] Started. name: 'Hexapod-ESP32'"));
}
#endif

bool nrfInit() {
  if (!radio.begin()) {
    Serial.println(F("[NRF24] Module not found!"));
    return false;
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPayloadSize(sizeof(RadioPacket));
  radio.setRetries(3, 5);
  radio.openReadingPipe(1, NRF_ADDRESS);
  radio.startListening();
  Serial.printf("[NRF24] Ready. Channel=108 Address=%s Payload=%u bytes\n",
                (const char*)NRF_ADDRESS, (unsigned)sizeof(RadioPacket));
  return true;
}

void nrfPoll() {
  if (!nrfAvailable) return;
  while (radio.available()) {
    RadioPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    radio.read(&pkt, sizeof(pkt));
    parseBinaryPacket(pkt, SRC_NRF);
  }
}
