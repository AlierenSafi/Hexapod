#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "robot_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define WS_PORT               81
#define WS_MAX_CLIENTS        5
#define WS_PING_INTERVAL_MS   30000
#define WIFI_CONNECT_TIMEOUT  15000
#define WIFI_RECONNECT_DELAY  5000

#define AP_SSID               "Hexapod-Setup"
#define AP_PASS               "12345678"
#define AP_CHANNEL            6
#define AP_MAX_CLIENTS        4

#define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_UUID     "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Command Source definitions
enum CmdSource : uint8_t { SRC_NONE = 0, SRC_BLE = 1, SRC_NRF = 2, SRC_WS = 3 };

union __attribute__((packed)) RadioPacket {
  struct {
    uint8_t type;
    int8_t  xRate;
    int8_t  yRate;
    int8_t  yawRate;
    uint8_t cal_leg;
    uint8_t cal_joint;
    uint8_t cal_trim;
    uint8_t _r[2];
    uint8_t checksum;
  } motion;

  struct {
    uint8_t type;
    uint8_t paramId;
    float   value;
    uint8_t flags;
    uint8_t _r[2];
    uint8_t checksum;
  } config;

  uint8_t raw[10];
};

struct __attribute__((packed)) LegacyCmd {
  uint8_t type;
  int8_t  xRate, yRate, yawRate;
  uint8_t reserved, checksum;
};

// WiFi State Enum
enum WiFiState : uint8_t {
  WIFI_DISCONNECTED = 0,
  WIFI_CONNECTING   = 1,
  WIFI_CONNECTED    = 2,
  WIFI_AP_MODE      = 3
};

extern WiFiState wifiState;
extern bool nrfAvailable;
extern CmdSource lastSrc;
extern const byte NRF_ADDRESS[6];

// FreeRTOS Queue for motion commands
extern QueueHandle_t xQueueCmd;

void wifiInit();
void wifiLoop();
void broadcastTelemetry(const char* json);
bool hasAuthenticatedClient();
uint32_t getClientCount();

#ifdef USE_BLE
  void bleInit();
  extern volatile bool bleConnected;
#endif

bool nrfInit();
void nrfPoll();

void dispatchMotionCmd(const RadioPacket& pkt, CmdSource src);

#endif
