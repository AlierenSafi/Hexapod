// ════════════════════════════════════════════════════════════════
//  hexapod_future.ino  ·  Gelecek Özellikler Stub'ı  ·  v3.1.0
//
//  BU DOSYA ÇALIŞMAZ - SADECE ARAYÜZ TANIMLARI İÇERİR
//
//  Bu dosya aşağıdaki gelecek özellikler için genişleme noktaları
//  ve arayüz tanımları içerir:
//
//    • NRF24 Kumanda Entegrasyonu
//    • Otonom Davranışlar (Rota İzleme)
//    • Input Source Abstraction
//
//  Bu stub'lar derlenir ancak aktif olarak kullanılmazlar.
//  Gerçek implementasyon sonraki versiyonlarda eklenecektir.
// ════════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════════
// BÖLÜM 1 — INPUT SOURCE ABSTRACTION
// ════════════════════════════════════════════════════════════════
//
// Amaç: WebSocket GUI, NRF24 kumanda ve otonom davranışlar aynı
// komut modeline yazabilmeli.
//
// Mevcut: SRC_NONE, SRC_BLE, SRC_NRF, SRC_WS
// Gelecek: SRC_AUTO (otonom davranış)

// InputMode enum'u hexapod_esp32_v3.ino'da tanımlı:
// typedef enum {
//   MODE_MANUAL_WS = 0,      // WebSocket GUI (şu anki)
//   MODE_MANUAL_NRF24 = 1,   // NRF24 kumanda (gelecek)
//   MODE_AUTO_FOLLOW = 2,    // Rota izleme (gelecek)
//   MODE_AUTO_AVOID = 3,     // Engelden kaçınma (gelecek)
//   MODE_AUTO_PATROL = 4     // Devriye (gelecek)
// } InputMode;

// Input source selector (gelecekte kullanılacak)
struct InputSource {
  InputMode mode;
  CmdSource src;
  bool active;
  uint32_t lastActivityMs;
  
  // Mode değişim callback'i
  void (*onModeChange)(InputMode oldMode, InputMode newMode);
};

// Global input source (stub)
static InputSource g_inputSource = {
  MODE_MANUAL_WS,
  SRC_WS,
  true,
  0,
  nullptr
};

// Mode değiştirme fonksiyonu (stub)
bool setInputMode(InputMode newMode) {
  if (g_inputSource.mode == newMode) return true;
  
  Serial.printf("[INPUT] Mode değişimi: %d → %d\n", 
                (int)g_inputSource.mode, (int)newMode);
  
  // Callback çağır
  if (g_inputSource.onModeChange) {
    g_inputSource.onModeChange(g_inputSource.mode, newMode);
  }
  
  g_inputSource.mode = newMode;
  
  // Mode'a göre source ata
  switch (newMode) {
    case MODE_MANUAL_WS:
      g_inputSource.src = SRC_WS;
      break;
    case MODE_MANUAL_NRF24:
      g_inputSource.src = SRC_NRF;
      break;
    case MODE_AUTO_FOLLOW:
    case MODE_AUTO_AVOID:
    case MODE_AUTO_PATROL:
      g_inputSource.src = SRC_NONE;  // Otonom = dahili
      break;
  }
  
  return true;
}

// ════════════════════════════════════════════════════════════════
// BÖLÜM 2 — NRF24 KUMANDA ADAPTER (Stub)
// ════════════════════════════════════════════════════════════════
//
// Amaç: Fiziksel NRF24 kumanda (joystick'li) ile robotu kontrol etmek
//
// Gereksinimler:
//   • NRF24L01+ modül (mevcut)
//   • Arduino/ESP32 tabanlı kumanda cihazı
//   • 2x analog joystick (X/Y)
//   • Birkaç buton (gait değiştir, servo enable/disable)

// Kumanda paket formatı (gelecek)
struct NrfControllerPacket {
  uint8_t type;        // 0x01 = motion, 0x02 = button
  int8_t  joyLx;       // Sol joystick X (-100..+100)
  int8_t  joyLy;       // Sol joystick Y (-100..+100)
  int8_t  joyRx;       // Sağ joystick X (-100..+100)
  int8_t  joyRy;       // Sağ joystick Y (-100..+100)
  uint8_t buttons;     // Buton bitmask
  uint8_t checksum;
};

// Buton bitmask
#define NRF_BTN_GAIT     0x01
#define NRF_BTN_SERVO    0x02
#define NRF_BTN_SITDOWN  0x04
#define NRF_BTN_RESERVED 0x08

// NRF24 kumanda durumu
struct NrfControllerState {
  bool connected;
  uint32_t lastPacketMs;
  uint32_t packetCount;
  uint32_t timeoutCount;
  
  // Son bilinen değerler
  int8_t joyLx, joyLy, joyRx, joyRy;
  uint8_t buttons;
  
  // Timeout kontrolü
  static const uint32_t TIMEOUT_MS = 500;  // 500ms paket yoksa timeout
};

static NrfControllerState g_nrfController = {};

// NRF24 kumanda init (stub)
void nrfControllerInit() {
  Serial.println(F("[NRF-CTRL] Kumanda adapter stub yüklendi."));
  Serial.println(F("[NRF-CTRL] NOT: Gerçek implementasyon sonraki versiyonda."));
  
  g_nrfController.connected = false;
  g_nrfController.lastPacketMs = 0;
}

// NRF24 kumanda loop (stub - hexapod_comm.ino'nun nrfPoll()'üne entegre edilecek)
void nrfControllerLoop() {
  // Mevcut nrfPoll() buraya yönlendirilecek
  // Gelecekte: Kumanda paketi gelirse parse et ve komuta çevir
  
  // Timeout kontrolü
  if (g_nrfController.connected) {
    if (millis() - g_nrfController.lastPacketMs > NrfControllerState::TIMEOUT_MS) {
      g_nrfController.connected = false;
      g_nrfController.timeoutCount++;
      
      Serial.println(F("[NRF-CTRL] Kumanda timeout!"));
      
      // Güvenlik: motion stop
      if (g_inputSource.mode == MODE_MANUAL_NRF24) {
        RadioPacket stopPkt;
        memset(&stopPkt, 0, sizeof(stopPkt));
        dispatchMotionCmd(stopPkt, SRC_NRF);
      }
    }
  }
}

// Kumanda paketi parse (stub)
void parseNrfControllerPacket(const uint8_t* data, size_t len) {
  if (len != sizeof(NrfControllerPacket)) return;
  
  const NrfControllerPacket* pkt = (const NrfControllerPacket*)data;
  
  // Checksum kontrolü (gelecekte eklenecek)
  
  g_nrfController.connected = true;
  g_nrfController.lastPacketMs = millis();
  g_nrfController.packetCount++;
  
  // Değerleri kaydet
  g_nrfController.joyLx = pkt->joyLx;
  g_nrfController.joyLy = pkt->joyLy;
  g_nrfController.joyRx = pkt->joyRx;
  g_nrfController.joyRy = pkt->joyRy;
  g_nrfController.buttons = pkt->buttons;
  
  // Mode kontrolü
  if (g_inputSource.mode != MODE_MANUAL_NRF24) {
    // Otomatik olarak NRF24 mode'a geç
    setInputMode(MODE_MANUAL_NRF24);
  }
  
  // Komuta çevir (gelecekte eklenecek)
  // nrfControllerToMotionCommand();
}

// ════════════════════════════════════════════════════════════════
// BÖLÜM 3 — OTONOM DAVRANIŞLAR (Stub)
// ════════════════════════════════════════════════════════════════
//
// Amaç: Robotun özerk olarak rota izlemesi, engelden kaçınması,
// veya devriye atması.

// AutoState enum'u hexapod_esp32_v3.ino'da tanımlı:
// typedef enum {
//   AUTO_IDLE = 0,
//   AUTO_PLANNING = 1,
//   AUTO_MOVING = 2,
//   AUTO_OBSTACLE_DETECTED = 3,
//   AUTO_AVOIDING = 4,
//   AUTO_REACHED = 5,
//   AUTO_ERROR = 6
// } AutoState;

// Waypoint struct'ı hexapod_esp32_v3.ino'da tanımlı:
// struct Waypoint {
//   float x;        // Gövde göreli X (mm)
//   float y;        // Gövde göreli Y (mm)
//   float heading;  // Hedef yön (derece)
//   float speed;    // Hız (0-100)
//   uint8_t flags;  // Özel davranış flag'leri
// };
// #define WP_FLAG_PAUSE    0x01
// #define WP_FLAG_LOOKAT   0x02
// #define WP_FLAG_FINAL    0x04

// Otonom navigasyon context
struct AutoNavigation {
  bool active;
  AutoState state;
  
  // Rota
  Waypoint* waypoints;
  uint8_t wpCount;
  uint8_t currentWp;
  
  // Sensör verileri (gelecekte eklenecek)
  float frontDistance;    // Ön mesafe sensörü (cm)
  float leftDistance;     // Sol mesafe sensörü (cm)
  float rightDistance;    // Sağ mesafe sensörü (cm)
  
  // PID kontrol (yörünge takibi)
  float pathKp, pathKi, pathKd;
  float pathErrorIntegral;
  float pathErrorPrev;
  
  // İstatistikler
  uint32_t startTimeMs;
  uint32_t obstacleCount;
  float totalDistance;
};

static AutoNavigation g_autoNav = {};

// Otonom init (stub)
void autoNavigationInit() {
  Serial.println(F("[AUTO-NAV] Otonom navigasyon stub yüklendi."));
  Serial.println(F("[AUTO-NAV] NOT: Gerçek implementasyon sonraki versiyonda."));
  
  g_autoNav.active = false;
  g_autoNav.state = AUTO_IDLE;
  g_autoNav.waypoints = nullptr;
  g_autoNav.wpCount = 0;
  g_autoNav.currentWp = 0;
}

// Rota yükle (stub)
bool loadRoute(Waypoint* points, uint8_t count) {
  if (points == nullptr || count == 0) return false;
  
  g_autoNav.waypoints = points;
  g_autoNav.wpCount = count;
  g_autoNav.currentWp = 0;
  
  Serial.printf("[AUTO-NAV] Rota yüklendi: %d waypoint\n", count);
  return true;
}

// Otonom başlat/durdur (stub)
bool startAutoNavigation() {
  if (g_autoNav.waypoints == nullptr || g_autoNav.wpCount == 0) {
    Serial.println(F("[AUTO-NAV] HATA: Rota yüklenmemiş!"));
    return false;
  }
  
  g_autoNav.active = true;
  g_autoNav.state = AUTO_PLANNING;
  g_autoNav.startTimeMs = millis();
  g_autoNav.obstacleCount = 0;
  g_autoNav.totalDistance = 0;
  
  setInputMode(MODE_AUTO_FOLLOW);
  
  Serial.println(F("[AUTO-NAV] Otonom navigasyon başlatıldı."));
  return true;
}

void stopAutoNavigation() {
  g_autoNav.active = false;
  g_autoNav.state = AUTO_IDLE;
  
  // Motion stop
  RadioPacket stopPkt;
  memset(&stopPkt, 0, sizeof(stopPkt));
  dispatchMotionCmd(stopPkt, SRC_NONE);
  
  // Mode'u manuel'e çevir
  setInputMode(MODE_MANUAL_WS);
  
  Serial.println(F("[AUTO-NAV] Otonom navigasyon durduruldu."));
}

// Ana otonom loop (stub - taskSensorComm'dan çağrılacak)
void autoNavigationLoop() {
  if (!g_autoNav.active) return;
  
  switch (g_autoNav.state) {
    case AUTO_IDLE:
      // Bekle
      break;
      
    case AUTO_PLANNING:
      // Bir sonraki waypoint'e yol planla
      // Gelecekte: A* veya basit lineer interpolasyon
      g_autoNav.state = AUTO_MOVING;
      break;
      
    case AUTO_MOVING:
      // Hareket komutu üret
      // Gelecekte: PID kontrol ile yörünge takibi
      
      // Engel kontrolü
      if (g_autoNav.frontDistance < 30.0f) {  // 30cm
        g_autoNav.state = AUTO_OBSTACLE_DETECTED;
        g_autoNav.obstacleCount++;
      }
      break;
      
    case AUTO_OBSTACLE_DETECTED:
      // Engelden kaçınma stratejisi seç
      // Gelecekte: Sağa/sola dön veya geri git
      g_autoNav.state = AUTO_AVOIDING;
      break;
      
    case AUTO_AVOIDING:
      // Kaçınma manevrası yap
      // Gelecekte: Sabit bir pattern veya dinamik
      
      // Engel kalktı mı?
      if (g_autoNav.frontDistance > 50.0f) {
        g_autoNav.state = AUTO_PLANNING;  // Yeniden planla
      }
      break;
      
    case AUTO_REACHED:
      // Hedefe ulaşıldı
      Serial.println(F("[AUTO-NAV] Hedefe ulaşıldı!"));
      stopAutoNavigation();
      break;
      
    case AUTO_ERROR:
      // Hata durumu
      Serial.println(F("[AUTO-NAV] HATA! Durduruluyor..."));
      stopAutoNavigation();
      break;
  }
}

// ════════════════════════════════════════════════════════════════
// BÖLÜM 4 — YARDIMCI FONKSİYONLAR
// ════════════════════════════════════════════════════════════════

const char* inputModeToString(InputMode mode) {
  switch (mode) {
    case MODE_MANUAL_WS:     return "MANUAL_WS";
    case MODE_MANUAL_NRF24:  return "MANUAL_NRF24";
    case MODE_AUTO_FOLLOW:   return "AUTO_FOLLOW";
    case MODE_AUTO_AVOID:    return "AUTO_AVOID";
    case MODE_AUTO_PATROL:   return "AUTO_PATROL";
    default:                 return "UNKNOWN";
  }
}

const char* autoStateToString(AutoState state) {
  switch (state) {
    case AUTO_IDLE:               return "IDLE";
    case AUTO_PLANNING:           return "PLANNING";
    case AUTO_MOVING:             return "MOVING";
    case AUTO_OBSTACLE_DETECTED:  return "OBSTACLE_DETECTED";
    case AUTO_AVOIDING:           return "AVOIDING";
    case AUTO_REACHED:            return "REACHED";
    case AUTO_ERROR:              return "ERROR";
    default:                      return "UNKNOWN";
  }
}

void printFutureStubStatus() {
  Serial.println(F("\n╔══════════════════════════════════════════════════╗"));
  Serial.println(F("║  GELECEK ÖZELLİKLER STUB DURUMU                  ║"));
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.printf( "║  Input Mode:  %s\n", inputModeToString(g_inputSource.mode));
  Serial.printf( "║  NRF24 Conn:  %s\n", g_nrfController.connected ? "EVET" : "hayır");
  Serial.printf( "║  Auto State:  %s\n", autoStateToString(g_autoNav.state));
  Serial.printf( "║  Auto Active: %s\n", g_autoNav.active ? "EVET" : "hayır");
  Serial.println(F("╚══════════════════════════════════════════════════╝\n"));
}
