// ════════════════════════════════════════════════════════════════
//  hexapod_esp32.ino  ·  Ana Dosya  ·  v3.0.0
//  ESP32 18-DOF Heksapod — Dinamik Konfigürasyon Sistemi
//
//  Dosya Yapısı:
//    hexapod_esp32.ino   ← Bu dosya (struct'lar, globaller, setup/loop)
//    hexapod_config.ino  ← Konfigürasyon sistemi (NVS, updateParameter)
//    hexapod_drivers.ino ← PCA9685 servo sürücüsü
//    hexapod_ik.ino      ← Ters kinematik + sikloid yörünge
//    hexapod_gait.ino    ← Gait planlayıcı + güvenli pozu
//    hexapod_imu.ino     ← MPU6050 + Complementary Filter + PID
//    hexapod_comm.ino    ← BLE + NRF24 + Paket parser
//    hexapod_tasks.ino   ← FreeRTOS Core0/Core1 görevleri
// ════════════════════════════════════════════════════════════════

// ── Kütüphaneler ─────────────────────────────────────────────────
#include <Wire.h>
#include <math.h>
#include <SPI.h>
#include <RF24.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
// BLE — isteğe bağlı (WiFi WebSocket birincil haberleşme)
// Etkinleştirmek için: build flag veya burada #define USE_BLE
// #define USE_BLE
#ifdef USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>

// ════════════════════════════════════════════════════════════════
// BÖLÜM 1 — SABİT DONANIM PIN TANIMLARI
// (Donanıma bağlı — runtime'da değiştirilemez)
// ════════════════════════════════════════════════════════════════
#define I2C_SDA          21
#define I2C_SCL          22
#define OE_PIN            4   // PCA9685 Output Enable (Active LOW)
#define BATT_ADC_PIN     36   // Batarya voltaj ADC (VP, ADC1_CH0)
                              // 20kΩ+10kΩ bölücü + 100nF → Vout=Vin/3
#define NRF_CE_PIN        5   // NRF24L01+ CE
#define NRF_CSN_PIN      15   // NRF24L01+ CSN
// NRF SPI: SCK=18, MOSI=23, MISO=19 (ESP32 varsayılan)

// Bacak ucu switch GPIO pinleri (INPUT_PULLUP)
const uint8_t SWITCH_PINS[6] = {13, 14, 25, 26, 32, 33};

// PCA9685 I2C adresleri
#define PCA9685_RIGHT    0x40
#define PCA9685_LEFT     0x41

// PCA9685 register adresleri
#define PCA9685_MODE1    0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L        0x06
#define SERVO_FREQ       50   // Hz (sabit, donanım frekansı)

// MPU6050
#define MPU6050_ADDR     0x68
#define MPU_PWR_MGMT     0x6B
#define MPU_ACCEL_XOUT   0x3B
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG_REG   0x1A

// BLE UUID'leri (Nordic UART benzeri)
#ifdef USE_BLE
  #define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
  #define BLE_CHAR_UUID     "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#endif

// NRF24 ağ adresi (kumanda ile eşleşmeli)
const byte NRF_ADDRESS[6] = "HXPD1";

// ════════════════════════════════════════════════════════════════
// BÖLÜM 2 — VERİ TİPLERİ VE ENUM'LAR
// ════════════════════════════════════════════════════════════════

// 2D / 3D vektör
struct Vec2 { float x, y; };
struct Vec3 { float x, y, z; };

// Eklem açıları
struct JointAngles {
  float coxa;    // Yatay dönüş (°)
  float femur;   // Dikey kaldırma (°)
  float tibia;   // Diz bükümü (°)
};

// Yürüyüş tipi
enum GaitType  : uint8_t { TRIPOD = 0, RIPPLE = 1, WAVE = 2 };

// Komut kaynağı
enum CmdSource : uint8_t { SRC_NONE = 0, SRC_BLE = 1, SRC_NRF = 2, SRC_WS = 3 };

// ── Parametre Kimlik Kodu (Binary paket için) ─────────────────────
// Config paketi: [0xF0][paramId][float(4B)][flags][reserved][checksum]
enum ParamID : uint8_t {
  // Mekanik
  PID_COXA_LEN      = 0x01,
  PID_FEMUR_LEN     = 0x02,
  PID_TIBIA_LEN     = 0x03,
  PID_STANCE_RADIUS = 0x04,
  PID_STANCE_HEIGHT = 0x05,
  PID_STEP_HEIGHT   = 0x06,
  PID_STEP_LENGTH   = 0x07,
  // Gait
  PID_GAIT_SPEED    = 0x08,
  PID_SWING_RATIO   = 0x09,
  PID_GAIT_TYPE     = 0x0A,
  // IMU / PID
  PID_KP            = 0x10,
  PID_KI            = 0x11,
  PID_KD            = 0x12,
  PID_COMP_ALPHA    = 0x13,
  PID_LEV_LIMIT     = 0x14,
  PID_PID_INT_LIM   = 0x15,
  // Servo limitleri
  PID_SERVO_MIN     = 0x20,
  PID_SERVO_MAX     = 0x21,
  // Özellik bayrakları
  PID_LEVELING_EN   = 0x30,
  PID_PHASE_RST_EN  = 0x31,
  // Batarya eşikleri
  PID_BATT_WARN     = 0x32,
  PID_BATT_CRIT     = 0x33,
  PID_BATT_CUTOFF   = 0x34,
  // Telemetri ve güvenlik
  PID_TELE_RATE     = 0x35,
  PID_COMM_TIMEOUT  = 0x36,
  // Servo trim — 0x40 + leg*3 + joint (0x40..0x57)
  PID_TRIM_BASE     = 0x40,
  // Sistem komutları
  PID_SAVE_NVS      = 0xF0,  // Tüm yapıyı Flash'a kaydet
  PID_LOAD_NVS      = 0xF1,  // Flash'tan yeniden yükle
  PID_RESET_DEF     = 0xF2,  // Fabrika varsayılanlarına dön
  PID_UNKNOWN       = 0xFF,
};

// ── Paket Bayrakları ──────────────────────────────────────────────
#define PKT_FLAG_SAVE_NVS  0x01  // Bu değeri NVS'e de kaydet
#define PKT_FLAG_SILENT    0x02  // Serial log basma

// ════════════════════════════════════════════════════════════════
// BÖLÜM 3 — MERKEZİ KONFİGÜRASYON YAPISI (RobotSettings)
//
//  Tasarım ilkesi:
//   · Tüm runtime ayarlanabilir parametreler buradadır.
//   · Donanım pin sabitleri (yukarıda) buraya dahil değildir.
//   · configMutex koruması altında okunup yazılır.
//   · NVS anahtarları: max 15 karakter (Preferences limiti).
// ════════════════════════════════════════════════════════════════
struct __attribute__((aligned(4))) RobotSettings {

  // ── Mekanik: Bacak segment uzunlukları (mm) ───────────────────
  float coxaLen;         // NVS: "coxa"
  float femurLen;        // NVS: "femur"
  float tibiaLen;        // NVS: "tibia"

  // ── Duruş geometrisi (mm) ─────────────────────────────────────
  float stanceRadius;    // NVS: "st_radius"  — bacak kökü–ayak yatay mesafesi
  float stanceHeight;    // NVS: "st_height"  — çalışma yüksekliği (negatif)

  // ── Adım parametreleri (mm) ───────────────────────────────────
  float stepHeight;      // NVS: "step_h"     — maksimum kaldırma yüksekliği
  float stepLength;      // NVS: "step_len"   — tek adım yatay mesafesi

  // ── Yürüyüş dinamikleri ───────────────────────────────────────
  float    gaitSpeed;    // NVS: "gait_spd"   — faz artışı / döngü (0.01–0.05)
  float    swingRatio;   // NVS: "swing_r"    — swing oranı (0.2–0.5)
  GaitType gaitType;     // NVS: "gait_type"  — 0=Tripod 1=Ripple 2=Wave
  uint8_t  _pad0[3];     // Hizalama dolgusu

  // ── Auto-leveling PID kazançları ──────────────────────────────
  float kp;              // NVS: "kp"
  float ki;              // NVS: "ki"
  float kd;              // NVS: "kd"
  float levelingLimit;   // NVS: "lev_lim"    — maks Z offset (mm)
  float pidIntLimit;     // NVS: "pid_ilim"   — integral windup limiti

  // ── Complementary Filter ──────────────────────────────────────
  float compAlpha;       // NVS: "comp_a"     — 0.90–0.99

  // ── Servo PWM limitleri (4096 adım @ 50Hz) ────────────────────
  uint16_t servoMin;     // NVS: "s_min"      — ~150 (0.73ms)
  uint16_t servoMax;     // NVS: "s_max"      — ~600 (2.93ms)

  // ── Özellik bayrakları ────────────────────────────────────────
  bool levelingEnabled;  // NVS: "lev_en"     — otonom denge aktif mi?
  bool phaseResetEnabled;// NVS: "pr_en"      — switch faz sıfırlama aktif mi?
  uint8_t _pad1[2];      // Hizalama dolgusu

  // ── Servo kalibrasyon trim değerleri (°, –30..+30) ───────────
  // [bacak 0-5][eklem: 0=coxa, 1=femur, 2=tibia]
  // NVS: "t00".."t52"
  int8_t servoTrim[6][3];
  uint8_t _pad2[2];      // Hizalama dolgusu

  // ── Batarya izleme eşikleri (V) ─────────────────────────────
  float battWarnVolt;     // NVS: "bat_warn"  — düşük voltaj uyarısı (varsayılan 7.0V)
  float battCritVolt;     // NVS: "bat_crit"  — kritik: sitDown + OE=HIGH (varsayılan 6.4V)
  float battCutoffVolt;   // NVS: "bat_cut"   — kesim: deep sleep (varsayılan 6.0V)

  // ── Telemetri ve güvenlik ───────────────────────────────────
  uint16_t telemetryRateMs; // NVS: "tele_ms"  — fast telemetri aralığı (varsayılan 100ms)
  uint16_t commTimeoutMs;   // NVS: "comm_to"  — komut timeout süresi (varsayılan 5000ms)

  // ── WiFi kimlik bilgileri ───────────────────────────────────
  char wifiSSID[33];      // NVS: "wifi_ssid" — maks 32 karakter + null
  char wifiPass[65];      // NVS: "wifi_pass" — maks 64 karakter + null

};  // sizeof ≈ 188 byte

// ════════════════════════════════════════════════════════════════
// BÖLÜM 4 — KONTROL PAKETİ (BLE ve NRF24 Ortak Format)
//
//  10 byte sabit paket — iki alternatif görünüm (union):
//
//  [ HAREKET PAKETİ ] type=0x00–0x12
//    [type][xRate][yRate][yawRate][0][0][0][0][0][checksum]
//
//  [ KONFİG PAKETİ ] type=0xF0
//    [0xF0][paramId][val0][val1][val2][val3][flags][0][0][checksum]
//    val0-val3: IEEE-754 float, little-endian
//    flags: PKT_FLAG_SAVE_NVS | PKT_FLAG_SILENT
//
//  checksum = XOR(byte[0]..byte[8])
//
//  BLE Ek Destek: JSON string paketi ('{' ile başlıyorsa):
//    {"k":"step_h","v":30.0,"s":1}
//    k = parametre adı (string)
//    v = değer (float)
//    s = 1 ise NVS'e kaydet (isteğe bağlı)
// ════════════════════════════════════════════════════════════════
union __attribute__((packed)) RadioPacket {
  // Hareket komutu görünümü
  struct {
    uint8_t type;      // 0x00=dur, 0x01=yürü, 0x02=dön, 0x03=gait
    int8_t  xRate;     // –100..+100
    int8_t  yRate;     // –100..+100
    int8_t  yawRate;   // –100..+100
    uint8_t cal_leg;   // Kalibrasyon: bacak indeksi (0x10–0x12)
    uint8_t cal_joint; // Kalibrasyon: eklem indeksi
    uint8_t cal_trim;  // Kalibrasyon: trim değeri
    uint8_t _r[2];     // Rezerv
    uint8_t checksum;
  } motion;

  // Konfigürasyon paketi görünümü
  struct {
    uint8_t type;      // 0xF0
    uint8_t paramId;   // ParamID enum
    float   value;     // IEEE-754 little-endian
    uint8_t flags;     // PKT_FLAG_*
    uint8_t _r[2];     // Rezerv
    uint8_t checksum;
  } config;

  uint8_t raw[10];     // Ham bayt erişimi (checksum için)
};
static_assert(sizeof(RadioPacket) <= 32, "RadioPacket çok büyük");

// ── Eski 6-byte motion struct (eski komutlarla uyumluluk) ─────────
struct __attribute__((packed)) LegacyCmd {
  uint8_t type;
  int8_t  xRate, yRate, yawRate;
  uint8_t reserved, checksum;
};

// ════════════════════════════════════════════════════════════════
// BÖLÜM 5 — BACAK DURUM YAPISI
// ════════════════════════════════════════════════════════════════
struct LegState {
  Vec3        footPos;        // Anlık ayak pozisyonu (bacak köküne göre, mm)
  Vec3        stancePos;      // Swing başlangıcındaki son yere temas noktası
  float       phase;          // Gait fazı [0.0, 1.0)
  bool        isSwing;        // true = havada
  bool        switchContact;  // Önceki switch durumu (kenar tespiti)
  bool        _pad[2];
  JointAngles angles;         // Hesaplanan eklem açıları
};

// IMU işlenmiş veri
struct IMUData {
  float pitch, roll;     // Complementary Filter çıktısı (°)
  float ax, ay, az;      // Ham ivme (g)
  float gx, gy, gz;      // Ham açısal hız (°/s)
};

// ── Batarya durumu (hexapod_battery.ino tarafından güncellenir) ──
enum BattLevel : uint8_t {
  BATT_SHUTDOWN = 0,   // ≤ battCutoffVolt → deep sleep
  BATT_CRITICAL = 1,   // ≤ battCritVolt  → sitDown + OE=HIGH
  BATT_LOW_WARN = 2,   // ≤ battWarnVolt  → GUI alarm
  BATT_NORMAL   = 3,   // Nominal çalışma
  BATT_FULL     = 4    // ≥ 8.2V
};

struct BatteryState {
  float    voltage;      // Filtrelenmiş voltaj (V)
  float    percentage;   // Yüzde (%)
  BattLevel level;       // Kademeli seviye
  uint32_t lastReadMs;   // Son başarılı okuma zamanı
};

// ── Sistem durumu (runtime izleme) ──────────────────────────────
struct SystemState {
  uint32_t uptimeMs;       // Çalışma süresi
  uint32_t lastCmdMs;      // Son hareket komutu zamanı
  bool     commTimeout;    // true = commTimeoutMs aşıldı → motion stop
  bool     battLow;        // true = batarya uyarı seviyesinde
  uint8_t  faultCode;      // 0=OK, bit0=I2C hata, bit1=IMU, bit2=WiFi
  int16_t  wifiRSSI;       // WiFi sinyal gücü (dBm)
  uint32_t freeHeap;       // Serbest heap bellek (byte)
};

// ════════════════════════════════════════════════════════════════
// BÖLÜM 6 — SABİT TABLOLAR (Donanım bağımlı, değişmez)
// ════════════════════════════════════════════════════════════════

// Gövde merkezine göre bacak kök pozisyonları (mm)
const Vec2 LEG_ORIGIN[6] = {
  { 60.0f,  80.0f},  // 0: Sağ Ön
  { 80.0f,   0.0f},  // 1: Sağ Orta
  { 60.0f, -80.0f},  // 2: Sağ Arka
  {-60.0f,  80.0f},  // 3: Sol Ön
  {-80.0f,   0.0f},  // 4: Sol Orta
  {-60.0f, -80.0f},  // 5: Sol Arka
};

// Sol bacak coxa servo yönü ters (mekanik simetri)
const int8_t LEG_DIR[6] = {1, 1, 1, -1, -1, -1};

// Gait faz ofseti dizileri
const float TRIPOD_PHASE[6] = {0.00f, 0.50f, 0.00f, 0.50f, 0.00f, 0.50f};
const float RIPPLE_PHASE[6] = {0.00f, 0.33f, 0.66f, 0.50f, 0.83f, 0.16f};
const float WAVE_PHASE[6]   = {0.00f, 0.17f, 0.33f, 0.50f, 0.67f, 0.83f};

// PCA9685 → bacak kanal haritası
struct ServoMap { uint8_t addr, coxa, femur, tibia; };
const ServoMap LEG_SERVO[6] = {
  {PCA9685_RIGHT,  0,  1,  2},  // Bacak 0: Sağ Ön
  {PCA9685_RIGHT,  3,  4,  5},  // Bacak 1: Sağ Orta
  {PCA9685_RIGHT,  6,  7,  8},  // Bacak 2: Sağ Arka
  {PCA9685_LEFT,   0,  1,  2},  // Bacak 3: Sol Ön
  {PCA9685_LEFT,   3,  4,  5},  // Bacak 4: Sol Orta
  {PCA9685_LEFT,   6,  7,  8},  // Bacak 5: Sol Arka
};

// ════════════════════════════════════════════════════════════════
// BÖLÜM 7 — GLOBAL DEĞİŞKENLER
// ════════════════════════════════════════════════════════════════

// ── WiFi State (hexapod_wifi.ino ve hexapod_battery.ino tarafından kullanılır)
enum WiFiState : uint8_t {
  WIFI_DISCONNECTED = 0,
  WIFI_CONNECTING   = 1,
  WIFI_CONNECTED    = 2,
  WIFI_AP_MODE      = 3
};
WiFiState wifiState = WIFI_DISCONNECTED;

// ── Input Mode (hexapod_future.ino tarafından kullanılır)
typedef enum {
  MODE_MANUAL_WS = 0,      // WebSocket GUI (şu anki)
  MODE_MANUAL_NRF24 = 1,   // NRF24 kumanda (gelecek)
  MODE_AUTO_FOLLOW = 2,    // Rota izleme (gelecek)
  MODE_AUTO_AVOID = 3,     // Engelden kaçınma (gelecek)
  MODE_AUTO_PATROL = 4     // Devriye (gelecek)
} InputMode;

// ── Auto State (hexapod_future.ino tarafından kullanılır)
typedef enum {
  AUTO_IDLE = 0,
  AUTO_PLANNING = 1,
  AUTO_MOVING = 2,
  AUTO_OBSTACLE_DETECTED = 3,
  AUTO_AVOIDING = 4,
  AUTO_REACHED = 5,
  AUTO_ERROR = 6
} AutoState;

// ── Waypoint (hexapod_future.ino tarafından kullanılır)
struct Waypoint {
  float x;        // Gövde göreli X (mm)
  float y;        // Gövde göreli Y (mm)
  float heading;  // Hedef yön (derece)
  float speed;    // Hız (0-100)
  uint8_t flags;  // Özel davranış flag'leri
};

#define WP_FLAG_PAUSE    0x01  // Bu noktada bekle
#define WP_FLAG_LOOKAT   0x02  // Yönel ama hareket etme
#define WP_FLAG_FINAL    0x04  // Son nokta

// ── WebSocket (hexapod_wifi.ino'da tanımlı)
// WebSocketsServer webSocket(WS_PORT);

// ── Merkezi konfigürasyon (configMutex koruması altında okunur/yazılır)
RobotSettings cfg;

// ── Hareket durumu ────────────────────────────────────────────────
LegState  legs[6];
IMUData   imuData = {};

// ── Batarya ve sistem durumu ──────────────────────────────────────
BatteryState battState = {};
SystemState  sysState  = {};

// ── Kontrol komutu havuzu (cmdMutex koruması) ────────────────────
RadioPacket ctrlPkt;     // Son gelen hareket paketi
CmdSource   lastSrc  = SRC_NONE;
GaitType    currentGait = TRIPOD;

// ── Çekirdekler arası lock-free sinyaller ────────────────────────
// Core0 yazar → Core1 okur → Core1 false yapar
volatile bool phaseResetReq[6]  = {};
// Core0 yazar → Core1 okur → konfigürasyonu yeniden yükler
volatile bool configChanged     = false;

// ── Donanım varlık bayrakları (setup'ta belirlenir) ───────────────
bool mpuAvailable  = false;
bool nrfAvailable  = false;

// ── FreeRTOS senkronizasyon nesneleri ────────────────────────────
SemaphoreHandle_t configMutex = nullptr;  // cfg struct koruması
SemaphoreHandle_t cmdMutex    = nullptr;  // ctrlPkt koruması
SemaphoreHandle_t imuMutex    = nullptr;  // imuData koruması
SemaphoreHandle_t wireMutex   = nullptr;  // I2C bus paylaşımı

// ── FreeRTOS görev handle'ları ────────────────────────────────────
TaskHandle_t hSensorTask = nullptr;
TaskHandle_t hKinTask    = nullptr;

// ── Donanım nesneleri ─────────────────────────────────────────────
RF24        radio(NRF_CE_PIN, NRF_CSN_PIN);
Preferences prefs;

// ── BLE ──────────────────────────────────────────────────────────
#ifdef USE_BLE
  BLECharacteristic* pBLEChar     = nullptr;
  volatile bool      bleConnected = false;
#endif

// ── PID iç durumu (sadece Core1'de erişilir — mutex gerekmez) ────
float pidPitchInt  = 0.0f, pidPitchPrev = 0.0f;
float pidRollInt   = 0.0f, pidRollPrev  = 0.0f;

// ── Forward declarations (ileri bildirimler) ────────────────────
void wifiInit();
void wifiLoop();
void battInit();
void battLoop();
void wdtInit();
void wdtFeed();
void wdtLoop();
void wdtSubscribeTask(TaskHandle_t taskHandle, const char* taskName);
void telemetryLoop();
void sendEvent(const char* type, const char* msg);
void sendLowVoltageWarning(float voltage);
void sendCriticalVoltage(float voltage);
void sendCommTimeout();
void markSensorTaskLoop();
void markKinTaskLoop();

// ── FreeRTOS Task'leri ───────────────────────────────────────────
void taskSensorComm(void* param);
void taskKinematics(void* param);

// ════════════════════════════════════════════════════════════════
// setup()
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F(
    "\n╔══════════════════════════════════════════════════╗\n"
      "║  ESP32 Hexapod  v3.0.0  — Dinamik Konfig Sistemi ║\n"
      "║  BLE + NRF24  |  18-DOF  |  Dual-Core FreeRTOS   ║\n"
      "╚══════════════════════════════════════════════════╝\n"
  ));

  // ── 1. Güvenlik önlemi: Servolar pasif ──────────────────────────
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH);
  Serial.println(F("[SAFETY] OE=HIGH — Servolar devre dışı."));

  // ── 2. Switch pinleri ────────────────────────────────────────────
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
  }

  // ── 3. Mutex'ler (TASK'LARDAN ÖNCE oluşturulmalı) ───────────────
  configMutex = xSemaphoreCreateMutex();
  cmdMutex    = xSemaphoreCreateMutex();
  imuMutex    = xSemaphoreCreateMutex();
  wireMutex   = xSemaphoreCreateMutex();

  if (!configMutex || !cmdMutex || !imuMutex || !wireMutex) {
    Serial.println(F("[FATAL] Mutex oluşturma başarısız! Yeniden başlatılıyor."));
    esp_restart();
  }
  Serial.println(F("[RTOS] 4× Mutex oluşturuldu."));

  // ── 4. Konfigürasyonu yükle (varsayılan → NVS üstüne yaz) ───────
  loadConfiguration();   // hexapod_config.ino

  // ── 5. I2C ──────────────────────────────────────────────────────
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.printf("[I2C] SDA=%d  SCL=%d  400kHz\n", I2C_SDA, I2C_SCL);

  // ── 6. PCA9685 servo sürücüleri ─────────────────────────────────
  pca9685Init(PCA9685_RIGHT);
  pca9685Init(PCA9685_LEFT);

  // ── 7. BLE (opsiyonel — WiFi WebSocket birincil) ──────────────────
  #ifdef USE_BLE
    bleInit();   // hexapod_comm.ino
  #else
    Serial.println(F("[BLE] Devre dışı (USE_BLE tanımsız). WiFi WebSocket aktif."));
  #endif

  // ── 8. NRF24L01+ ────────────────────────────────────────────────
  nrfAvailable = nrfInit();   // hexapod_comm.ino

  // ── 9. MPU6050 ──────────────────────────────────────────────────
  mpuAvailable = mpuInit();   // hexapod_imu.ino
  if (!mpuAvailable) {
    Serial.println(F("[UYARI] MPU6050 bulunamadı — auto-leveling devre dışı."));
  }

  // ── 10. Servolar etkin + başlangıç pozu ─────────────────────────
  delay(500);
  digitalWrite(OE_PIN, LOW);
  Serial.println(F("[SAFETY] OE=LOW — Servolar etkinleştirildi."));
  sitDown();   // hexapod_gait.ino
  delay(1500);

  // ── 11. FreeRTOS görevleri ───────────────────────────────────────
  // Core 0 — Sensör + Haberleşme + Parser (100Hz, 10KB stack)
  xTaskCreatePinnedToCore(
    taskSensorComm, "SensorTask",
    10240, nullptr, 2, &hSensorTask, 0
  );

  // Core 1 — Kinematik + Servo (50Hz kesin, 20KB stack)
  xTaskCreatePinnedToCore(
    taskKinematics, "KinTask",
    20480, nullptr, 2, &hKinTask, 1
  );

  Serial.println(F("[HAZIR] Core0: SensorTask | Core1: KinTask"));
  Serial.println(F("[HAZIR] BLE: 'Hexapod-ESP32' | NRF: Kanal 108"));

  // ── 12. OTA Manager (bağımsız modül — hexapod_ota.ino) ──────────
  //  Bu tek satır OTA altyapısının tamamını başlatır:
  //  WiFi bağlantısı, ArduinoOTA yapılandırması ve OTATask oluşturma.
  //  hKinTask ve hSensorTask bu noktada geçerli olmalıdır.
  setupOTA();
}

// ════════════════════════════════════════════════════════════════
// loop() — Boş. Tüm mantık FreeRTOS task'larında.
// ════════════════════════════════════════════════════════════════
void loop() {
  vTaskDelay(pdMS_TO_TICKS(5000));
}
