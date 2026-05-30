// ════════════════════════════════════════════════════════════════
//  hexapod_watchdog.ino  ·  Watchdog ve Hata Kurtarma  ·  v3.1.0
//
//  Bu modül ESP32 Task Watchdog Timer (TWDT) ve haberleşme timeout
//  mekanizmalarını yönetir.
//
//  Özellikler:
//    • Task Watchdog: Her task 5 sn'de bir feed vermeli
//    • Comm Timeout: 5 sn komut gelmezse motion stop
//    • Hata sınıflandırması: Recoverable / Degradable / Fatal
//    • Kurtarma politikası: Otomatik retry → Degraded mode → Safe stop
// ════════════════════════════════════════════════════════════════

#include <esp_task_wdt.h>

// ════════════════════════════════════════════════════════════════
// SABİTLER
// ════════════════════════════════════════════════════════════════

#define WDT_TIMEOUT_SECONDS   5
#define WDT_PANIC_ON_TIMEOUT  true  // Timeout olursa panic/reset

// Comm timeout (hareket komutu gelmezse dur)
#define COMM_TIMEOUT_CHECK_MS 100   // Her 100ms kontrol

// Fault kodları (bit maskesi)
#define FAULT_NONE      0x00
#define FAULT_I2C       0x01
#define FAULT_IMU       0x02
#define FAULT_WIFI      0x04
#define FAULT_NRF24     0x08
#define FAULT_BATTERY   0x10
#define FAULT_SERVO     0x20

// ════════════════════════════════════════════════════════════════
// GLOBAL DEĞİŞKENLER
// ════════════════════════════════════════════════════════════════

static bool wdtInitialized = false;
static uint32_t lastCommCheckMs = 0;
static uint32_t commTimeoutCount = 0;
static uint32_t wdtResetCount = 0;

// Task timing izleme
static uint32_t sensorTaskLoopCount = 0;
static uint32_t kinTaskLoopCount = 0;
static uint32_t lastSensorTaskMs = 0;
static uint32_t lastKinTaskMs = 0;

// ════════════════════════════════════════════════════════════════
// BAŞLATMA
// ════════════════════════════════════════════════════════════════

void wdtInit() {
  Serial.println(F("\n[WDT] Watchdog başlatılıyor..."));
  
  // Task Watchdog Timer yapılandır
  // ESP-IDF 5.x+ için config struct kullan
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = (1 << 0) | (1 << 1),  // Her iki core da idle izle
    .trigger_panic = WDT_PANIC_ON_TIMEOUT
  };
  
  esp_err_t err = esp_task_wdt_init(&twdt_config);
  
  if (err != ESP_OK) {
    Serial.printf("[WDT] HATA: TWDT init başarısız (err=%d)\n", err);
    return;
  }
  
  // Görevlere abone ol (her görev kendi init'inde abone olacak)
  // taskSensorComm ve taskKinematics'te esp_task_wdt_add() çağrılacak
  
  wdtInitialized = true;
  Serial.printf("[WDT] TWDT aktif: %d sn timeout, panic=%s\n",
                WDT_TIMEOUT_SECONDS, WDT_PANIC_ON_TIMEOUT ? "evet" : "hayır");
}

void wdtSubscribeTask(TaskHandle_t taskHandle, const char* taskName) {
  if (!wdtInitialized) return;
  
  esp_err_t err = esp_task_wdt_add(taskHandle);
  if (err == ESP_OK) {
    Serial.printf("[WDT] '%s' TWDT'ye abone oldu.\n", taskName);
  } else {
    Serial.printf("[WDT] HATA: '%s' abone olamadı (err=%d)\n", taskName, err);
  }
}

// ════════════════════════════════════════════════════════════════
// WDT FEED (Her task'in döngüsünde çağrılmalı)
// ════════════════════════════════════════════════════════════════

void wdtFeed() {
  if (!wdtInitialized) return;
  esp_task_wdt_reset();
}

// ════════════════════════════════════════════════════════════════
// HABERLEŞME TIMEOUT KONTROLÜ
// ════════════════════════════════════════════════════════════════

void commTimeoutCheck() {
  uint32_t now = millis();
  if (now - lastCommCheckMs < COMM_TIMEOUT_CHECK_MS) return;
  lastCommCheckMs = now;
  
  // Son komut ne zaman geldi?
  uint32_t elapsed = now - sysState.lastCmdMs;
  
  if (elapsed > cfg.commTimeoutMs) {
    if (!sysState.commTimeout) {
      // İlk kez timeout
      sysState.commTimeout = true;
      commTimeoutCount++;
      
      Serial.printf("[COMM] Timeout! Son komuttan beri %lu ms. Motion stop.\n", elapsed);
      
      // Motion stop
      RadioPacket stopPkt;
      memset(&stopPkt, 0, sizeof(stopPkt));
      stopPkt.motion.type = 0x00;  // DUR
      dispatchMotionCmd(stopPkt, SRC_NONE);
      
      // Event gönder
      sendCommTimeout();
    }
  } else {
    if (sysState.commTimeout) {
      // Timeout temizlendi
      sysState.commTimeout = false;
      Serial.println(F("[COMM] Timeout temizlendi."));
    }
  }
}

// ════════════════════════════════════════════════════════════════
// HATA SINIFLANDIRMA ve KURTARMA
// ════════════════════════════════════════════════════════════════

void setFault(uint8_t faultBit, bool active) {
  if (active) {
    sysState.faultCode |= faultBit;
  } else {
    sysState.faultCode &= ~faultBit;
  }
}

bool hasFault(uint8_t faultBit) {
  return (sysState.faultCode & faultBit) != 0;
}

void handleFault(uint8_t faultBit) {
  switch (faultBit) {
    case FAULT_I2C:
      Serial.println(F("[FAULT] I2C hatası - IMU ve servo sürücüler etkilenebilir."));
      // I2C bus'ı resetle
      Wire.end();
      delay(10);
      Wire.begin(I2C_SDA, I2C_SCL);
      Wire.setClock(400000);
      break;
      
    case FAULT_IMU:
      Serial.println(F("[FAULT] IMU hatası - Auto-leveling devre dışı."));
      // Leveling'i devre dışı bırak ama harekete devam et
      sysState.faultCode |= FAULT_IMU;
      break;
      
    case FAULT_WIFI:
      Serial.println(F("[FAULT] WiFi hatası - WebSocket yeniden bağlanıyor."));
      // WiFi reconnect hexapod_wifi.ino içinde yönetiliyor
      break;
      
    case FAULT_BATTERY:
      Serial.println(F("[FAULT] Batarya hatası - hexapod_battery.ino yönetiyor."));
      break;
      
    case FAULT_SERVO:
      Serial.println(F("[FAULT] Servo hatası - PCA9685 reset deneniyor."));
      digitalWrite(OE_PIN, HIGH);
      delay(100);
      pca9685Init(PCA9685_RIGHT);
      pca9685Init(PCA9685_LEFT);
      digitalWrite(OE_PIN, LOW);
      break;
  }
}

// ════════════════════════════════════════════════════════════════
// GÜVENLİ DURDURMA (Safe Stop)
// ════════════════════════════════════════════════════════════════

void safeStop() {
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  [SAFETY] GÜVENLİ DURDURMA AKTİF       ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));
  
  // 1. Motion stop
  RadioPacket stopPkt;
  memset(&stopPkt, 0, sizeof(stopPkt));
  stopPkt.motion.type = 0x00;
  dispatchMotionCmd(stopPkt, SRC_NONE);
  
  // 2. Kısa bekle (gait döngüsü tamamlansın)
  delay(100);
  
  // 3. Sit down
  sitDown();
  delay(500);
  
  // 4. Servo tork kes
  digitalWrite(OE_PIN, HIGH);
  
  // 5. Event
  sendEvent("safe_stop", "Güvenli durdurma tamamlandı");
  
  Serial.println(F("[SAFETY] Servolar devre dışı. Sistem güvenli durumda."));
}

void safeResume() {
  Serial.println(F("[SAFETY] Sistem devam ettiriyor..."));
  
  // Servo tork geri ver
  digitalWrite(OE_PIN, LOW);
  
  // Kısa bekle
  delay(200);
  
  sendEvent("safe_resume", "Sistem devam etti");
}

// ════════════════════════════════════════════════════════════════
// ANA DÖNGÜ (taskSensorComm'dan çağrılır)
// ════════════════════════════════════════════════════════════════

void wdtLoop() {
  // WDT feed
  wdtFeed();
  
  // Comm timeout kontrolü
  commTimeoutCheck();
  
  // Aktif fault'ları kontrol et
  if (sysState.faultCode != FAULT_NONE) {
    // Her fault için ayrı handling
    for (int i = 0; i < 8; i++) {
      uint8_t bit = (1 << i);
      if (hasFault(bit)) {
        handleFault(bit);
      }
    }
  }
}

// ════════════════════════════════════════════════════════════════
// TASK TIMING İZLEME (Debug için)
// ════════════════════════════════════════════════════════════════

void markSensorTaskLoop() {
  sensorTaskLoopCount++;
  lastSensorTaskMs = millis();
}

void markKinTaskLoop() {
  kinTaskLoopCount++;
  lastKinTaskMs = millis();
}

void printTaskTiming() {
  uint32_t now = millis();
  
  Serial.println(F("┌─── Task Timing ────────────────────────────┐"));
  Serial.printf( "│ SensorTask: %lu loops, last=%lu ms ago\n",
                 sensorTaskLoopCount,
                 now - lastSensorTaskMs);
  Serial.printf( "│ KinTask:    %lu loops, last=%lu ms ago\n",
                 kinTaskLoopCount,
                 now - lastKinTaskMs);
  Serial.printf( "│ WDT Resets: %lu\n", wdtResetCount);
  Serial.printf( "│ Comm TOs:   %lu\n", commTimeoutCount);
  Serial.println(F("└────────────────────────────────────────────┘"));
}

// ════════════════════════════════════════════════════════════════
// YARDIMCI FONKSİYONLAR
// ════════════════════════════════════════════════════════════════

const char* faultCodeToString(uint8_t code) {
  switch (code) {
    case FAULT_NONE:    return "NONE";
    case FAULT_I2C:     return "I2C";
    case FAULT_IMU:     return "IMU";
    case FAULT_WIFI:    return "WIFI";
    case FAULT_NRF24:   return "NRF24";
    case FAULT_BATTERY: return "BATTERY";
    case FAULT_SERVO:   return "SERVO";
    default:            return "MULTIPLE";
  }
}

void wdtPrintStatus() {
  Serial.println(F("┌─── Watchdog Durumu ────────────────────────┐"));
  Serial.printf( "│ TWDT:       %s\n", wdtInitialized ? "Aktif" : "Devre dışı");
  Serial.printf( "│ Timeout:    %d sn\n", WDT_TIMEOUT_SECONDS);
  Serial.printf( "│ Comm TO:    %s (%lu ms)\n",
                 sysState.commTimeout ? "EVET" : "hayır",
                 cfg.commTimeoutMs);
  Serial.printf( "│ Fault:      0x%02X (%s)\n",
                 sysState.faultCode,
                 faultCodeToString(sysState.faultCode));
  Serial.println(F("└────────────────────────────────────────────┘"));
}
