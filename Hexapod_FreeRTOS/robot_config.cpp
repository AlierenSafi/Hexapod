#include "robot_config.h"
#include <stddef.h>

// Global settings instances
RobotSettings cfg;
SemaphoreHandle_t configMutex = nullptr;
uint32_t savedConfigCRC = 0;

// PID internal states (declared extern here, defined in kinematics.cpp or main)
extern float pidPitchInt, pidPitchPrev;
extern float pidRollInt, pidRollPrev;

// CRC32 implementation
uint32_t calculateCRC32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

// Private helper to set factory defaults
static void setDefaults(RobotSettings* s) {
  s->coxaLen         = 25.0f;
  s->femurLen        = 50.0f;
  s->tibiaLen        = 70.0f;
  s->stanceRadius    = 100.0f;
  s->stanceHeight    = -80.0f;   // Negative = downwards

  s->stepHeight      = 30.0f;
  s->stepLength      = 35.0f;

  s->gaitSpeed       = 0.025f;   // ~800ms/loop cycle @50Hz
  s->swingRatio      = 0.33f;    // 33% of cycle in swing
  s->gaitType        = TRIPOD;

  s->kp              = 1.5f;
  s->ki              = 0.05f;
  s->kd              = 0.8f;
  s->levelingLimit   = 25.0f;    // ±25mm max Z offset
  s->pidIntLimit     = 20.0f;    // Integral windup limit

  s->compAlpha       = 0.96f;    // Gyro weight

  s->servoMin        = 150;      // ≈ 0.73ms → -90°
  s->servoMax        = 600;      // ≈ 2.93ms → +90°

  s->levelingEnabled   = true;
  s->phaseResetEnabled = true;

  memset(s->servoTrim, 0, sizeof(s->servoTrim));

  s->battWarnVolt    = 7.0f;     // Warning: ~20%
  s->battCritVolt    = 6.4f;     // Critical
  s->battCutoffVolt  = 6.0f;     // Cutoff

  s->telemetryRateMs = 100;      // 10Hz fast telemetry
  s->commTimeoutMs   = 5000;     // 5 seconds timeout

  memset(s->wifiSSID, 0, sizeof(s->wifiSSID));
  memset(s->wifiPass, 0, sizeof(s->wifiPass));
}

void loadConfiguration() {
  Preferences prefs;
  setDefaults(&cfg);

  if (!prefs.begin("hexapod", true)) {
    Serial.println(F("[CFG] NVS namespace not found — defaults used."));
    printConfiguration();
    return;
  }

  cfg.coxaLen          = prefs.getFloat("coxa",     cfg.coxaLen);
  cfg.femurLen         = prefs.getFloat("femur",    cfg.femurLen);
  cfg.tibiaLen         = prefs.getFloat("tibia",    cfg.tibiaLen);
  cfg.stanceRadius     = prefs.getFloat("st_radius",cfg.stanceRadius);
  cfg.stanceHeight     = prefs.getFloat("st_height",cfg.stanceHeight);
  cfg.stepHeight       = prefs.getFloat("step_h",   cfg.stepHeight);
  cfg.stepLength       = prefs.getFloat("step_len", cfg.stepLength);
  cfg.gaitSpeed        = prefs.getFloat("gait_spd", cfg.gaitSpeed);
  cfg.swingRatio       = prefs.getFloat("swing_r",  cfg.swingRatio);
  cfg.gaitType         = (GaitType)prefs.getUChar("gait_type", (uint8_t)cfg.gaitType);
  cfg.kp               = prefs.getFloat("kp",       cfg.kp);
  cfg.ki               = prefs.getFloat("ki",       cfg.ki);
  cfg.kd               = prefs.getFloat("kd",       cfg.kd);
  cfg.levelingLimit    = prefs.getFloat("lev_lim",  cfg.levelingLimit);
  cfg.pidIntLimit      = prefs.getFloat("pid_ilim", cfg.pidIntLimit);
  cfg.compAlpha        = prefs.getFloat("comp_a",   cfg.compAlpha);
  cfg.servoMin         = prefs.getUShort("s_min",   cfg.servoMin);
  cfg.servoMax         = prefs.getUShort("s_max",   cfg.servoMax);
  cfg.levelingEnabled  = prefs.getBool("lev_en",    cfg.levelingEnabled);
  cfg.phaseResetEnabled= prefs.getBool("pr_en",     cfg.phaseResetEnabled);

  for (int l = 0; l < 6; l++) {
    for (int j = 0; j < 3; j++) {
      char key[5];
      snprintf(key, sizeof(key), "t%d%d", l, j);
      cfg.servoTrim[l][j] = prefs.getChar(key, cfg.servoTrim[l][j]);
    }
  }

  cfg.battWarnVolt     = prefs.getFloat("bat_warn",  cfg.battWarnVolt);
  cfg.battCritVolt     = prefs.getFloat("bat_crit",  cfg.battCritVolt);
  cfg.battCutoffVolt   = prefs.getFloat("bat_cut",   cfg.battCutoffVolt);
  cfg.telemetryRateMs  = prefs.getUShort("tele_ms",  cfg.telemetryRateMs);
  cfg.commTimeoutMs    = prefs.getUShort("comm_to",  cfg.commTimeoutMs);

  loadWiFiCredentials();
  prefs.end();

  validateConfig(&cfg);
  savedConfigCRC = calculateCRC32((const uint8_t*)&cfg, offsetof(RobotSettings, wifiSSID));

  Serial.println(F("[CFG] Loaded from NVS."));
  printConfiguration();
}

void saveConfiguration() {
  Preferences prefs;
  RobotSettings snap;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  snap = cfg;
  xSemaphoreGive(configMutex);

  uint32_t currentCRC = calculateCRC32((const uint8_t*)&snap, offsetof(RobotSettings, wifiSSID));
  if (currentCRC == savedConfigCRC) {
    Serial.println(F("[CFG] Configuration unchanged. NVS write skipped."));
    return;
  }

  if (!prefs.begin("hexapod", false)) {
    Serial.println(F("[CFG] ERROR: NVS write failed!"));
    return;
  }

  prefs.putFloat("coxa",     snap.coxaLen);
  prefs.putFloat("femur",    snap.femurLen);
  prefs.putFloat("tibia",    snap.tibiaLen);
  prefs.putFloat("st_radius",snap.stanceRadius);
  prefs.putFloat("st_height",snap.stanceHeight);
  prefs.putFloat("step_h",   snap.stepHeight);
  prefs.putFloat("step_len", snap.stepLength);
  prefs.putFloat("gait_spd", snap.gaitSpeed);
  prefs.putFloat("swing_r",  snap.swingRatio);
  prefs.putUChar("gait_type",(uint8_t)snap.gaitType);
  prefs.putFloat("kp",       snap.kp);
  prefs.putFloat("ki",       snap.ki);
  prefs.putFloat("kd",       snap.kd);
  prefs.putFloat("lev_lim",  snap.levelingLimit);
  prefs.putFloat("pid_ilim", snap.pidIntLimit);
  prefs.putFloat("comp_a",   snap.compAlpha);
  prefs.putUShort("s_min",   snap.servoMin);
  prefs.putUShort("s_max",   snap.servoMax);
  prefs.putBool("lev_en",    snap.levelingEnabled);
  prefs.putBool("pr_en",     snap.phaseResetEnabled);

  for (int l = 0; l < 6; l++) {
    for (int j = 0; j < 3; j++) {
      char key[5];
      snprintf(key, sizeof(key), "t%d%d", l, j);
      prefs.putChar(key, snap.servoTrim[l][j]);
    }
  }

  prefs.putFloat("bat_warn",  snap.battWarnVolt);
  prefs.putFloat("bat_crit",  snap.battCritVolt);
  prefs.putFloat("bat_cut",   snap.battCutoffVolt);
  prefs.putUShort("tele_ms",  snap.telemetryRateMs);
  prefs.putUShort("comm_to",  snap.commTimeoutMs);

  prefs.end();
  savedConfigCRC = currentCRC;
  Serial.println(F("[CFG] Saved to NVS."));
}

void resetToDefaults() {
  Preferences prefs;
  prefs.begin("hexapod", false);
  prefs.clear();
  prefs.end();

  xSemaphoreTake(configMutex, portMAX_DELAY);
  setDefaults(&cfg);
  xSemaphoreGive(configMutex);

  // Notify KinTask of config change using Task Notification
  if (hKinTask != nullptr) {
    xTaskNotifyGive(hKinTask);
  }

  Serial.println(F("[CFG] Reset to defaults. NVS cleared."));
  printConfiguration();
}

void validateConfig(RobotSettings* s) {
  bool dirty = false;

  #define CLAMP_CFG(field, lo, hi) \
    do { float _v = constrain(s->field, (float)(lo), (float)(hi)); \
         if (_v != s->field) { s->field = _v; dirty = true; } } while(0)

  #define NAN_GUARD(field, def) \
    do { if (!isfinite(s->field)) { s->field = (def); dirty = true; \
         Serial.printf("[CFG] WARNING: " #field " NaN/Inf -> %.2f\n",(float)(def)); } } while(0)

  NAN_GUARD(coxaLen,       25.0f);
  NAN_GUARD(femurLen,      50.0f);
  NAN_GUARD(tibiaLen,      70.0f);
  NAN_GUARD(stanceRadius, 100.0f);
  NAN_GUARD(stanceHeight, -80.0f);
  NAN_GUARD(stepHeight,    30.0f);
  NAN_GUARD(stepLength,    35.0f);
  NAN_GUARD(gaitSpeed,    0.025f);
  NAN_GUARD(swingRatio,   0.33f);
  NAN_GUARD(kp,            1.5f);
  NAN_GUARD(ki,           0.05f);
  NAN_GUARD(kd,            0.8f);
  NAN_GUARD(compAlpha,    0.96f);

  CLAMP_CFG(coxaLen,       5.0f,  100.0f);
  CLAMP_CFG(femurLen,      10.0f, 150.0f);
  CLAMP_CFG(tibiaLen,      10.0f, 150.0f);
  CLAMP_CFG(stanceRadius,  50.0f, 250.0f);
  CLAMP_CFG(stanceHeight, -200.0f, -20.0f);
  CLAMP_CFG(stepHeight,    5.0f,   80.0f);
  CLAMP_CFG(stepLength,    5.0f,  100.0f);
  CLAMP_CFG(gaitSpeed,     0.005f, 0.08f);
  CLAMP_CFG(swingRatio,    0.15f,  0.60f);
  CLAMP_CFG(kp,            0.0f,   20.0f);
  CLAMP_CFG(ki,            0.0f,    2.0f);
  CLAMP_CFG(kd,            0.0f,   10.0f);
  CLAMP_CFG(levelingLimit, 5.0f,   50.0f);
  CLAMP_CFG(pidIntLimit,   1.0f,   50.0f);
  CLAMP_CFG(compAlpha,     0.80f,   0.99f);

  if (s->servoMin < 100)  { s->servoMin = 100;  dirty = true; }
  if (s->servoMax > 700)  { s->servoMax = 700;  dirty = true; }
  if (s->servoMin >= s->servoMax) {
    s->servoMin = 150; s->servoMax = 600; dirty = true;
    Serial.println(F("[CFG] WARNING: servoMin >= servoMax — reset."));
  }

  if ((uint8_t)s->gaitType > 2) {
    s->gaitType = TRIPOD; dirty = true;
  }

  for (int l = 0; l < 6; l++) {
    for (int j = 0; j < 3; j++) {
      int8_t t = s->servoTrim[l][j];
      if (t < -30 || t > 30) {
        s->servoTrim[l][j] = constrain(t, (int8_t)-30, (int8_t)30);
        dirty = true;
      }
    }
  }

  float maxReach = s->femurLen + s->tibiaLen - 1.0f;
  float effRadius = s->stanceRadius - s->coxaLen;
  float effH      = fabsf(s->stanceHeight);
  float D = sqrtf(effRadius * effRadius + effH * effH);
  if (D > maxReach) {
    float newH = sqrtf(maxReach * maxReach - effRadius * effRadius);
    if (isfinite(newH)) {
      s->stanceHeight = -(newH);
      Serial.printf("[CFG] WARNING: Stance reach limit -> stanceHeight=%.1f\n", s->stanceHeight);
      dirty = true;
    }
  }

  NAN_GUARD(battWarnVolt,   7.0f);
  NAN_GUARD(battCritVolt,   6.4f);
  NAN_GUARD(battCutoffVolt, 6.0f);
  CLAMP_CFG(battWarnVolt,   6.0f, 8.4f);
  CLAMP_CFG(battCritVolt,   5.5f, 8.0f);
  CLAMP_CFG(battCutoffVolt, 5.0f, 7.5f);

  if (s->battCritVolt >= s->battWarnVolt) {
    s->battCritVolt = s->battWarnVolt - 0.4f;
    dirty = true;
    Serial.println(F("[CFG] WARNING: battCritVolt >= battWarnVolt — fixed."));
  }
  if (s->battCutoffVolt >= s->battCritVolt) {
    s->battCutoffVolt = s->battCritVolt - 0.4f;
    dirty = true;
    Serial.println(F("[CFG] WARNING: battCutoffVolt >= battCritVolt — fixed."));
  }

  if (s->telemetryRateMs < 50)   { s->telemetryRateMs = 50;   dirty = true; }
  if (s->telemetryRateMs > 2000) { s->telemetryRateMs = 2000; dirty = true; }
  if (s->commTimeoutMs < 1000)   { s->commTimeoutMs = 1000;   dirty = true; }
  if (s->commTimeoutMs > 30000)  { s->commTimeoutMs = 30000;  dirty = true; }

  #undef CLAMP_CFG
  #undef NAN_GUARD

  if (dirty) {
    Serial.println(F("[CFG] Config validation resolved out-of-bound settings."));
  }
}

bool updateParameter(const String& key, float value, bool saveNVS) {
  bool found = false;

  xSemaphoreTake(configMutex, portMAX_DELAY);

  if      (key == "coxa")      { cfg.coxaLen       = value; found = true; }
  else if (key == "femur")     { cfg.femurLen       = value; found = true; }
  else if (key == "tibia")     { cfg.tibiaLen       = value; found = true; }
  else if (key == "st_radius") { cfg.stanceRadius   = value; found = true; }
  else if (key == "st_height") { cfg.stanceHeight   = value; found = true; }
  else if (key == "step_h")    { cfg.stepHeight     = value; found = true; }
  else if (key == "step_len")  { cfg.stepLength     = value; found = true; }
  else if (key == "gait_spd")  { cfg.gaitSpeed      = value; found = true; }
  else if (key == "swing_r")   { cfg.swingRatio     = value; found = true; }
  else if (key == "gait_type") { cfg.gaitType       = (GaitType)((int)value % 3); found = true; }
  else if (key == "kp")        { cfg.kp             = value; found = true; }
  else if (key == "ki")        { cfg.ki             = value; found = true; }
  else if (key == "kd")        { cfg.kd             = value; found = true; }
  else if (key == "lev_lim")   { cfg.levelingLimit  = value; found = true; }
  else if (key == "pid_ilim")  { cfg.pidIntLimit    = value; found = true; }
  else if (key == "comp_a")    { cfg.compAlpha      = value; found = true; }
  else if (key == "s_min")     { cfg.servoMin       = (uint16_t)value; found = true; }
  else if (key == "s_max")     { cfg.servoMax       = (uint16_t)value; found = true; }
  else if (key == "lev_en")    { cfg.levelingEnabled    = (value > 0.5f); found = true; }
  else if (key == "pr_en")     { cfg.phaseResetEnabled  = (value > 0.5f); found = true; }
  else if (key == "bat_warn")  { cfg.battWarnVolt       = value; found = true; }
  else if (key == "bat_crit")  { cfg.battCritVolt       = value; found = true; }
  else if (key == "bat_cut")   { cfg.battCutoffVolt     = value; found = true; }
  else if (key == "tele_ms")   { cfg.telemetryRateMs    = (uint16_t)value; found = true; }
  else if (key == "comm_to")   { cfg.commTimeoutMs      = (uint16_t)value; found = true; }
  else if (key.startsWith("t") && key.length() == 3) {
    int l = key[1] - '0';
    int j = key[2] - '0';
    if (l >= 0 && l <= 5 && j >= 0 && j <= 2) {
      cfg.servoTrim[l][j] = (int8_t)constrain((int)value, -30, 30);
      found = true;
    }
  }

  if (found) {
    validateConfig(&cfg);
  }

  xSemaphoreGive(configMutex);

  if (!found) {
    Serial.printf("[CFG] Unknown param key: '%s'\n", key.c_str());
    return false;
  }

  Serial.printf("[CFG] %s = %.4f%s\n", key.c_str(), value, saveNVS ? " [NVS]" : "");

  // Notify KinTask of config change using Task Notification
  if (hKinTask != nullptr) {
    xTaskNotifyGive(hKinTask);
  }

  syncHardware();

  if (saveNVS) {
    saveConfiguration();
  }

  return true;
}

bool updateParamById(uint8_t paramId, float value, uint8_t flags) {
  bool saveNVS = (flags & PKT_FLAG_SAVE_NVS) != 0;

  if (paramId == PID_SAVE_NVS)  { saveConfiguration();  return true; }
  if (paramId == PID_LOAD_NVS)  { loadConfiguration();
                                   if (hKinTask != nullptr) xTaskNotifyGive(hKinTask);
                                   return true; }
  if (paramId == PID_RESET_DEF) { resetToDefaults();    return true; }

  if (paramId >= PID_TRIM_BASE && paramId <= 0x57) {
    uint8_t idx  = paramId - PID_TRIM_BASE;
    uint8_t leg  = idx / 3;
    uint8_t jnt  = idx % 3;
    if (leg < 6 && jnt < 3) {
      xSemaphoreTake(configMutex, portMAX_DELAY);
      cfg.servoTrim[leg][jnt] = (int8_t)constrain((int)value, -30, 30);
      xSemaphoreGive(configMutex);
      Serial.printf("[CFG] Trim B%d E%d = %+d°\n", leg, jnt, (int)value);
      if (hKinTask != nullptr) xTaskNotifyGive(hKinTask);
      syncHardware();
      if (saveNVS) saveConfiguration();
      return true;
    }
    return false;
  }

  const char* keyStr = nullptr;
  switch ((ParamID)paramId) {
    case PID_COXA_LEN:      keyStr = "coxa";      break;
    case PID_FEMUR_LEN:     keyStr = "femur";     break;
    case PID_TIBIA_LEN:     keyStr = "tibia";     break;
    case PID_STANCE_RADIUS: keyStr = "st_radius"; break;
    case PID_STANCE_HEIGHT: keyStr = "st_height"; break;
    case PID_STEP_HEIGHT:   keyStr = "step_h";    break;
    case PID_STEP_LENGTH:   keyStr = "step_len";  break;
    case PID_GAIT_SPEED:    keyStr = "gait_spd";  break;
    case PID_SWING_RATIO:   keyStr = "swing_r";   break;
    case PID_GAIT_TYPE:     keyStr = "gait_type"; break;
    case PID_KP:            keyStr = "kp";        break;
    case PID_KI:            keyStr = "ki";        break;
    case PID_KD:            keyStr = "kd";        break;
    case PID_LEV_LIMIT:     keyStr = "lev_lim";   break;
    case PID_PID_INT_LIM:   keyStr = "pid_ilim";  break;
    case PID_COMP_ALPHA:    keyStr = "comp_a";    break;
    case PID_SERVO_MIN:     keyStr = "s_min";     break;
    case PID_SERVO_MAX:     keyStr = "s_max";     break;
    case PID_LEVELING_EN:   keyStr = "lev_en";    break;
    case PID_PHASE_RST_EN:  keyStr = "pr_en";     break;
    case PID_BATT_WARN:     keyStr = "bat_warn";  break;
    case PID_BATT_CRIT:     keyStr = "bat_crit";  break;
    case PID_BATT_CUTOFF:   keyStr = "bat_cut";   break;
    case PID_TELE_RATE:     keyStr = "tele_ms";   break;
    case PID_COMM_TIMEOUT:  keyStr = "comm_to";   break;
    default:
      Serial.printf("[CFG] Unknown ParamID: 0x%02X\n", paramId);
      return false;
  }

  return updateParameter(String(keyStr), value, saveNVS);
}

void syncHardware() {
  pidPitchInt  = 0.0f; pidPitchPrev = 0.0f;
  pidRollInt   = 0.0f; pidRollPrev  = 0.0f;
}

void printConfiguration() {
  xSemaphoreTake(configMutex, portMAX_DELAY);
  RobotSettings s = cfg;
  xSemaphoreGive(configMutex);

  Serial.println(F("┌─── RobotSettings ──────────────────────────────────────┐"));
  Serial.printf( "│ Mechanics : Coxa=%.0f  Femur=%.0f  Tibia=%.0f mm\n", s.coxaLen, s.femurLen, s.tibiaLen);
  Serial.printf( "│ Stance    : R=%.0f  H=%.0f mm\n", s.stanceRadius, s.stanceHeight);
  Serial.printf( "│ Step      : H=%.0f  L=%.0f mm\n", s.stepHeight, s.stepLength);
  const char* gt[] = {"Tripod","Ripple","Wave"};
  Serial.printf( "│ Gait      : %s  Spd=%.3f  Swing=%.2f\n", gt[(int)s.gaitType], s.gaitSpeed, s.swingRatio);
  Serial.printf( "│ PID       : Kp=%.2f  Ki=%.3f  Kd=%.2f\n", s.kp, s.ki, s.kd);
  Serial.printf( "│ IMU       : Alpha=%.2f  Leveling=%s\n", s.compAlpha, s.levelingEnabled ? "ON" : "OFF");
  Serial.printf( "│ Servo PWM : Min=%d  Max=%d\n", s.servoMin, s.servoMax);
  Serial.printf( "│ PhaseRst  : %s\n", s.phaseResetEnabled ? "ON" : "OFF");
  Serial.printf( "│ Battery   : Warn=%.2fV  Crit=%.2fV  Cutoff=%.2fV\n", s.battWarnVolt, s.battCritVolt, s.battCutoffVolt);
  Serial.printf( "│ Telemetry : %dms  Timeout: %dms\n", s.telemetryRateMs, s.commTimeoutMs);
  Serial.printf( "│ WiFi SSID : %s\n", s.wifiSSID[0] ? s.wifiSSID : "(AP mode fallback)");
  Serial.print(  "│ Trim      : ");
  for (int l = 0; l < 6; l++) {
    Serial.printf("B%d[%+d,%+d,%+d] ", l, s.servoTrim[l][0], s.servoTrim[l][1], s.servoTrim[l][2]);
  }
  Serial.println();
  Serial.println(F("└────────────────────────────────────────────────────────┘"));
}

static const char* WIFI_NS = "wifi";

void loadWiFiCredentials() {
  Preferences wifiPrefs;
  if (!wifiPrefs.begin(WIFI_NS, true)) {
    Serial.println(F("[WIFI] WiFi credentials not found in NVS — AP mode fallback."));
    return;
  }
  wifiPrefs.getString("ssid", cfg.wifiSSID, sizeof(cfg.wifiSSID));
  wifiPrefs.getString("pass", cfg.wifiPass, sizeof(cfg.wifiPass));
  wifiPrefs.end();
  Serial.printf("[WIFI] Loaded SSID='%s'\n", cfg.wifiSSID[0] ? cfg.wifiSSID : "(empty)");
}

void saveWiFiCredentials() {
  Preferences wifiPrefs;
  if (!wifiPrefs.begin(WIFI_NS, false)) {
    Serial.println(F("[WIFI] ERROR: NVS open failed!"));
    return;
  }
  wifiPrefs.putString("ssid", cfg.wifiSSID);
  wifiPrefs.putString("pass", cfg.wifiPass);
  wifiPrefs.end();
  Serial.println(F("[WIFI] Saved credentials."));
}

void clearWiFiCredentials() {
  Preferences wifiPrefs;
  if (wifiPrefs.begin(WIFI_NS, false)) {
    wifiPrefs.clear();
    wifiPrefs.end();
  }
  memset(cfg.wifiSSID, 0, sizeof(cfg.wifiSSID));
  memset(cfg.wifiPass, 0, sizeof(cfg.wifiPass));
  Serial.println(F("[WIFI] Cleared credentials."));
}
