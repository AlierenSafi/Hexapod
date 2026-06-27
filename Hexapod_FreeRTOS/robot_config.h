#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Parameter ID Definitions (binary packets)
enum ParamID : uint8_t {
  PID_COXA_LEN      = 0x01,
  PID_FEMUR_LEN     = 0x02,
  PID_TIBIA_LEN     = 0x03,
  PID_STANCE_RADIUS = 0x04,
  PID_STANCE_HEIGHT = 0x05,
  PID_STEP_HEIGHT   = 0x06,
  PID_STEP_LENGTH   = 0x07,
  PID_GAIT_SPEED    = 0x08,
  PID_SWING_RATIO   = 0x09,
  PID_GAIT_TYPE     = 0x0A,
  PID_KP            = 0x10,
  PID_KI            = 0x11,
  PID_KD            = 0x12,
  PID_COMP_ALPHA    = 0x13,
  PID_LEV_LIMIT     = 0x14,
  PID_PID_INT_LIM   = 0x15,
  PID_SERVO_MIN     = 0x20,
  PID_SERVO_MAX     = 0x21,
  PID_LEVELING_EN   = 0x30,
  PID_PHASE_RST_EN  = 0x31,
  PID_BATT_WARN     = 0x32,
  PID_BATT_CRIT     = 0x33,
  PID_BATT_CUTOFF   = 0x34,
  PID_TELE_RATE     = 0x35,
  PID_COMM_TIMEOUT  = 0x36,
  PID_TRIM_BASE     = 0x40,
  PID_SAVE_NVS      = 0xF0,
  PID_LOAD_NVS      = 0xF1,
  PID_RESET_DEF     = 0xF2,
  PID_UNKNOWN       = 0xFF,
};

#define PKT_FLAG_SAVE_NVS  0x01
#define PKT_FLAG_SILENT    0x02

struct Vec2 { float x, y; };
struct Vec3 { float x, y, z; };

enum GaitType : uint8_t { TRIPOD = 0, RIPPLE = 1, WAVE = 2 };

struct __attribute__((aligned(4))) RobotSettings {
  float coxaLen;
  float femurLen;
  float tibiaLen;
  float stanceRadius;
  float stanceHeight;
  float stepHeight;
  float stepLength;
  float    gaitSpeed;
  float    swingRatio;
  GaitType gaitType;
  uint8_t  _pad0[3];
  float kp;
  float ki;
  float kd;
  float levelingLimit;
  float pidIntLimit;
  float compAlpha;
  uint16_t servoMin;
  uint16_t servoMax;
  bool levelingEnabled;
  bool phaseResetEnabled;
  uint8_t _pad1[2];
  int8_t servoTrim[6][3];
  uint8_t _pad2[2];
  float battWarnVolt;
  float battCritVolt;
  float battCutoffVolt;
  uint16_t telemetryRateMs;
  uint16_t commTimeoutMs;
  char wifiSSID[33];
  char wifiPass[65];
};

extern RobotSettings cfg;
extern SemaphoreHandle_t configMutex;
extern TaskHandle_t hKinTask; // Declared to send notifications

void loadConfiguration();
void saveConfiguration();
void resetToDefaults();
void validateConfig(RobotSettings* s);
bool updateParameter(const String& key, float value, bool saveNVS);
bool updateParamById(uint8_t paramId, float value, uint8_t flags);
void syncHardware();
void printConfiguration();
void loadWiFiCredentials();
void saveWiFiCredentials();
void clearWiFiCredentials();

uint32_t calculateCRC32(const uint8_t* data, size_t length);
extern uint32_t savedConfigCRC;

#endif
