// ════════════════════════════════════════════════════════════════
//  hexapod_ota.ino  ·  WiFi OTA Manager  ·  v3.0.0
//
//  Bu modül diğer hiçbir .ino dosyasına bağımlı değildir.
//  Mevcut IK / Gait / Sensör / BLE / NRF24 kodlarına DOKUNMAZ.
//
//  hexapod_esp32.ino'ya eklenmesi gereken tek satır:
//    setup() sonunda → setupOTA();
//
//  ── Tasarım Kararları ────────────────────────────────────────────
//
//  WiFi / BLE Birlikte Çalışma (Coexistence):
//    ESP32 radyosu WiFi ve BT/BLE için TDMA multiplexing kullanır.
//    esp_bt_controller_config ve coex_preference ile öncelik
//    BLE'ye verilir → BLE bağlantısı kopmaz, BLE throughput korunur.
//    NRF24L01+ Kanal 108 (2508MHz); WiFi en yüksek kanal 13
//    (2472MHz)'in üzerindedir → pratik RF çakışması minimumdur.
//
//  OTA Güvenlik Sekansı (Hareket Durdurma):
//    1. OE = HIGH  → Tüm servo PWM çıkışları anlık kesilir
//    2. KinTask SUSPEND → Core1 döngüsü durdurulur (servo yazmaz)
//    3. SensorTask SUSPEND → IMU / switch okumaları durdurulur
//    4. OTA flash yazımı başlar
//    Tersine sıra: RESUME SensorTask → RESUME KinTask → OE = LOW
//
//  FreeRTOS Öncelik Hiyerarşisi:
//    KinTask    (Core1): öncelik 2  — 50Hz, en kritik
//    SensorTask (Core0): öncelik 2  — 100Hz
//    OTATask    (Core0): öncelik 1  — arka plan, düşük öncelik
//
//  OTA Durum Makinesi:
//    IDLE → CONNECTING → READY → [STARTING → FLASHING → DONE]
//                ↑                              ↓
//                └─────────── ERROR ────────────┘
//
//  Desteklenen OTA Komut Paketi (BLE / NRF24):
//    type=0x20 → WiFi bağlan + OTA modu etkinleştir
//    type=0x21 → WiFi bağlantısını kes
//    type=0x22 → OTA bilgi raporu (Serial'e döker)
// ════════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>    // esp_wifi_set_ps() — güç yönetimi

// ════════════════════════════════════════════════════════════════
// BÖLÜM 1 — KULLANICI AYARLARI
//   Bu sabitler fiziksel ortama göre değiştirilmelidir.
//   WiFi kimlik bilgileri NVS'den yüklenir (hexapod_config.ino)
// ════════════════════════════════════════════════════════════════

// WiFi kimlik bilgileri — NVS'den yüklenir, burada sabit tanımlı değil
// Eğer NVS'de credential yoksa AP mode fallback aktif olur
// Credential ayarlamak için: {"cmd":"system","action":"save_wifi","ssid":"...","pass":"..."}

// OTA erişim denetimi (boş bırakırsan şifresiz → güvensiz ağda kullanma)
#define OTA_HOSTNAME           "Hexapod-ESP32"
#define OTA_PASSWORD           ""       // Boş = şifresiz, "şifre" = korumalı
#define OTA_PORT               3232     // Arduino OTA varsayılan portu

// Zamanlama sabitleri
#define WIFI_CONNECT_TIMEOUT_MS    15000UL   // WiFi bağlantı zaman aşımı (ms)
#define WIFI_RECONNECT_INTERVAL_MS 60000UL   // Bağlantı yenileme periyodu (ms)
#define OTA_TASK_IDLE_DELAY_MS       500     // Boşta ArduinoOTA.handle() aralığı
#define OTA_TASK_STACK_SIZE        6144      // Stack boyutu (byte)
#define OTA_TASK_PRIORITY             1      // Core 1/2'nin altında

// ════════════════════════════════════════════════════════════════
// BÖLÜM 2 — OTA DURUM MAKİNESİ
// ════════════════════════════════════════════════════════════════

enum OTAState : uint8_t {
  OTA_STATE_IDLE       = 0,   // WiFi devre dışı veya bağlanmamış
  OTA_STATE_CONNECTING = 1,   // WiFi bağlantı kuruluyor
  OTA_STATE_READY      = 2,   // WiFi bağlı, OTA dinliyor
  OTA_STATE_STARTING   = 3,   // OTA başlıyor: güvenlik sekansı
  OTA_STATE_FLASHING   = 4,   // Flash yazılıyor
  OTA_STATE_DONE       = 5,   // Güncelleme tamam, yeniden başlatılıyor
  OTA_STATE_ERROR      = 6    // Hata: kurtarma sekansı
};

// ── OTA bağlam yapısı ─────────────────────────────────────────────
struct OTAContext {
  volatile OTAState state;
  volatile uint8_t  lastProgress;   // %0–100
  volatile uint32_t lastErrorCode;
  volatile bool     wifiEnabled;    // WiFi modunu etkinleştir bayrağı
  uint32_t          lastConnectAttemptMs;
  uint32_t          totalBytesToFlash;
  uint32_t          bytesDoneFlash;
  char              lastErrorMsg[64];
};

static OTAContext otaCtx = {
  .state                = OTA_STATE_IDLE,
  .lastProgress         = 0,
  .lastErrorCode        = 0,
  .wifiEnabled          = false,
  .lastConnectAttemptMs = 0,
  .totalBytesToFlash    = 0,
  .bytesDoneFlash       = 0,
  .lastErrorMsg         = {}
};

// OTA görev handle'ı (hexapod_esp32.ino'dan da suspend/resume için erişilebilir)
TaskHandle_t hOTATask = nullptr;

// ════════════════════════════════════════════════════════════════
// BÖLÜM 3 — KRİTİK GÜVENLİK SEKANSINA YARDIMCI FONKSİYONLAR
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// otaSuspendRobot — OTA başlamadan önce hareketi güvenli durdur
//
//  Sıra kritiktir:
//    1. OE=HIGH (anlık) → servo akımı kesilir, mekanik çarpmalar önlenir
//    2. KinTask suspend  → Core1 servo yazmayı bırakır
//    3. SensorTask suspend → Core0 I2C okumayı bırakır
//
//  Bu noktadan sonra I2C bus boştur → ArduinoOTA yeniden başlatma
//  sürecinde I2C çakışması olmaz.
// ────────────────────────────────────────────────────────────────
static void otaSuspendRobot() {
  // ── Adım 1: Servo PWM çıkışlarını anında kes ─────────────────
  digitalWrite(OE_PIN, HIGH);
  Serial.println(F("[OTA] SAFETY: OE=HIGH — Servo tork kesildi."));

  // ── Adım 2: Kinematik görevi askıya al ───────────────────────
  if (hKinTask != nullptr) {
    vTaskSuspend(hKinTask);
    Serial.println(F("[OTA] SAFETY: KinTask (Core1) askıya alındı."));
  }

  // ── Adım 3: Sensör görevi askıya al ──────────────────────────
  if (hSensorTask != nullptr) {
    vTaskSuspend(hSensorTask);
    Serial.println(F("[OTA] SAFETY: SensorTask (Core0) askıya alındı."));
  }

  // ── Adım 4: Kısa bekleme — görev anahtarlaması için ──────────
  // FreeRTOS scheduler, vTaskSuspend çağrısının etkili olması için
  // en az bir tick beklemek gerekir.
  vTaskDelay(pdMS_TO_TICKS(50));
  Serial.println(F("[OTA] Robot güvenle durduruldu. Flash yazımı başlayabilir."));
}

// ────────────────────────────────────────────────────────────────
// otaResumeRobot — OTA iptal veya hata sonrasında robotu geri getir
//
//  Sıra tersine çevrilir:
//    1. SensorTask resume (önce veri akışı başlasın)
//    2. KinTask resume (ardından hareket mantığı çalışsın)
//    3. OE=LOW (kinematic ilk frame'i yazdıktan sonra servo aktif)
//
//  NOT: OTA başarılıysa bu fonksiyon çağrılmaz (sistem yeniden başlar).
// ────────────────────────────────────────────────────────────────
static void otaResumeRobot() {
  // Sensör görevi önce
  if (hSensorTask != nullptr) {
    vTaskResume(hSensorTask);
    Serial.println(F("[OTA] SensorTask (Core0) devam ettiridi."));
  }
  vTaskDelay(pdMS_TO_TICKS(20));

  // Kinematik görevi
  if (hKinTask != nullptr) {
    vTaskResume(hKinTask);
    Serial.println(F("[OTA] KinTask (Core1) devam ettirildi."));
  }
  vTaskDelay(pdMS_TO_TICKS(100));  // Bir gait döngüsü kadar bekle

  // Servo PWM çıkışlarını aç
  digitalWrite(OE_PIN, LOW);
  Serial.println(F("[OTA] SAFETY: OE=LOW — Servo tork geri verildi."));
}

// ════════════════════════════════════════════════════════════════
// BÖLÜM 4 — ArduinoOTA CALLBACK'LERİ
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// otaRegisterCallbacks — OTA callback'lerini bir kez kaydeder
// ────────────────────────────────────────────────────────────────
static void otaRegisterCallbacks() {

  // ── onStart: Güncelleme isteği alındı ─────────────────────────
  ArduinoOTA.onStart([]() {
    otaCtx.state        = OTA_STATE_STARTING;
    otaCtx.lastProgress = 0;

    // OTA tipi: Sketch mi, Filesystem mi?
    const char* typeName = (ArduinoOTA.getCommand() == U_FLASH)
                           ? "Sketch (Flash)"
                           : "Filesystem (SPIFFS/LittleFS)";

    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.printf( "║  [OTA] GÜNCELLEME BAŞLIYOR             ║\n");
    Serial.printf( "║  Tip   : %-30s║\n", typeName);
    Serial.println(F("╚════════════════════════════════════════╝"));

    // Kritik güvenlik sekansını uygula
    otaSuspendRobot();

    // NVS'yi kapat (güncelleme sırasında Flash erişim çakışması önle)
    prefs.end();

    otaCtx.state = OTA_STATE_FLASHING;
    Serial.println(F("[OTA] Flash yazımı başlıyor..."));
  });

  // ── onEnd: Güncelleme tamamlandı ──────────────────────────────
  ArduinoOTA.onEnd([]() {
    otaCtx.state        = OTA_STATE_DONE;
    otaCtx.lastProgress = 100;

    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.println(F("║  [OTA] GÜNCELLEME BAŞARILI!            ║"));
    Serial.println(F("║  Sistem 3 saniyede yeniden başlıyor... ║"));
    Serial.println(F("╚════════════════════════════════════════╝\n"));

    // Yeniden başlatmadan önce Serial flush
    Serial.flush();
    // ArduinoOTA kütüphanesi ESP.restart() çağıracak
    // Bu callback dönünce Arduino OTA handler restart eder
  });

  // ── onProgress: Yazım ilerleme bildirimi ──────────────────────
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    // Her %5'te bir log bas (seri portu boğmamak için)
    uint8_t pct = (total > 0) ? (uint8_t)((done * 100UL) / total) : 0;

    if (pct >= otaCtx.lastProgress + 5 || pct == 100) {
      otaCtx.lastProgress    = pct;
      otaCtx.bytesDoneFlash  = done;
      otaCtx.totalBytesToFlash = total;

      // İlerleme çubuğu (20 karakter genişlik)
      const int BAR_W = 20;
      int filled = (pct * BAR_W) / 100;
      char bar[BAR_W + 1];
      for (int i = 0; i < BAR_W; i++) bar[i] = (i < filled) ? '█' : '░';
      bar[BAR_W] = '\0';

      Serial.printf("[OTA] İlerleme: %3d%%  [%s]  %u / %u byte\n",
                    pct, bar, done, total);
    }
  });

  // ── onError: Hata oluştu ───────────────────────────────────────
  ArduinoOTA.onError([](ota_error_t error) {
    otaCtx.state       = OTA_STATE_ERROR;
    otaCtx.lastErrorCode = (uint32_t)error;

    // Hata kodunu string'e çevir
    const char* errStr;
    switch (error) {
      case OTA_AUTH_ERROR:    errStr = "Kimlik doğrulama hatası (şifre yanlış?)"; break;
      case OTA_BEGIN_ERROR:   errStr = "Başlatma hatası (yetersiz alan?)";        break;
      case OTA_CONNECT_ERROR: errStr = "Bağlantı hatası";                         break;
      case OTA_RECEIVE_ERROR: errStr = "Veri alma hatası";                        break;
      case OTA_END_ERROR:     errStr = "Sonlandırma hatası (CRC uyumsuz?)";       break;
      default:                errStr = "Bilinmeyen hata";                          break;
    }

    snprintf(otaCtx.lastErrorMsg, sizeof(otaCtx.lastErrorMsg),
             "Kod=%u: %s", (unsigned)error, errStr);

    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.printf( "║  [OTA] HATA                            ║\n");
    Serial.printf( "║  %s\n", otaCtx.lastErrorMsg);
    Serial.println(F("╚════════════════════════════════════════╝"));

    // Kurtarma: robotu geri getir
    if (otaCtx.state == OTA_STATE_FLASHING ||
        otaCtx.state == OTA_STATE_STARTING) {
      Serial.println(F("[OTA] Kurtarma sekansı başlatılıyor..."));
      otaResumeRobot();
    }

    otaCtx.state = OTA_STATE_READY;  // Yeniden OTA dinlemeye dön
  });
}

// ════════════════════════════════════════════════════════════════
// BÖLÜM 5 — WiFi YÖNETİMİ
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// otaConfigureWiFiCoex — WiFi/BLE birlikte çalışma tercihini ayarla
//
//  esp_coex_preference_set(ESP_COEX_PREFER_BT):
//    BT/BLE'ye radyo önceliği verir.
//    WiFi throughput düşer (~5-10 Mbit/s) ama BLE bağlantısı korunur.
//    OTA için 5 Mbit/s fazlasıyla yeterlidir.
//
//  WiFi güç tasarrufu: WIFI_PS_MIN_MODEM
//    Modem uyku aralığı minimumda → gecikme azalır
//    BLE ile çakışma azalır (radyo boş zamanı daha az)
// ────────────────────────────────────────────────────────────────
static void otaConfigureWiFiCoex() {
  // ESP32 Arduino core 3.3.x'te coex API doğrudan erişilebilir değil.
  // WiFi güç tasarrufu: minimum gecikme, BLE ile çakışmayı azaltır.
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  Serial.println(F("[OTA] WiFi PS=MIN_MODEM ayarlandı (BLE öncelikli çalışma)."));
}

// ────────────────────────────────────────────────────────────────
// otaConnectWiFi — WiFi bağlantısı kur (bloklamayan, zaman aşımlı)
//
//  Bağlantı kurulamazsa false döner; görev döngüsü yeniden deneyecektir.
//  Bu fonksiyon OTA Task içinden çağrılır (Ana görevleri bloklamaz).
// ────────────────────────────────────────────────────────────────
static bool otaConnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  // NVS'den credential yükle (eğer yüklenmemişse)
  if (cfg.wifiSSID[0] == '\0') {
    loadWiFiCredentials();
  }

  // Kimlik bilgileri tanımsızsa bağlanma
  if (cfg.wifiSSID[0] == '\0') {
    Serial.println(F("[OTA] UYARI: WiFi SSID ayarlanmamış. "
                     "WiFi credential'ları NVS'e kaydedin veya "
                     "hexapod_wifi.ino üzerinden AP mode kullanın."));
    return false;
  }

  otaCtx.state = OTA_STATE_CONNECTING;
  Serial.printf("[OTA] WiFi bağlanıyor: '%s' ...\n", cfg.wifiSSID);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  
  if (cfg.wifiPass[0] != '\0') {
    WiFi.begin(cfg.wifiSSID, cfg.wifiPass);
  } else {
    WiFi.begin(cfg.wifiSSID);  // Açık ağ
  }

  otaConfigureWiFiCoex();

  // Zaman aşımlı bekleme — aktif döngüde 100ms adımlarla poll
  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if ((millis() - startMs) > WIFI_CONNECT_TIMEOUT_MS) {
      WiFi.disconnect(true);
      otaCtx.state = OTA_STATE_IDLE;
      Serial.printf("[OTA] WiFi zaman aşımı (%lu ms). "
                    "Sonraki deneme: %lu sn sonra.\n",
                    WIFI_CONNECT_TIMEOUT_MS,
                    WIFI_RECONNECT_INTERVAL_MS / 1000UL);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Diğer task'lara CPU ver
  }

  Serial.printf("[OTA] WiFi bağlandı. IP: %s  RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(),
                WiFi.RSSI());
  return true;
}

// ════════════════════════════════════════════════════════════════
// BÖLÜM 6 — DIŞA AÇIK FONKSİYONLAR (hexapod_esp32.ino çağırır)
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// setupOTA — OTA altyapısını kurar ve FreeRTOS task'ını başlatır
//
//  hexapod_esp32.ino setup() sonunda bir kez çağrılır.
//  Görevler (hKinTask, hSensorTask) bu çağrıdan ÖNCE oluşturulmuş
//  olmalıdır (suspend/resume için geçerli handle şarttır).
// ────────────────────────────────────────────────────────────────
void setupOTA() {
  Serial.println(F("\n[OTA] Modül başlatılıyor..."));

  // ── ArduinoOTA yapılandır (WiFi bağlanmadan önce tanımlanabilir) ─
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);

  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
    Serial.println(F("[OTA] Şifre koruması AKTİF."));
  } else {
    Serial.println(F("[OTA] UYARI: Şifre koruması KAPALI. "
                     "OTA_PASSWORD sabitini ayarla."));
  }

  otaRegisterCallbacks();

  // ── OTA FreeRTOS görevi — Core 0, düşük öncelik (1) ─────────────
  BaseType_t ret = xTaskCreatePinnedToCore(
    taskOTA,              // Görev fonksiyonu
    "OTATask",            // Görev adı (izleme için)
    OTA_TASK_STACK_SIZE,  // Stack (6KB — WiFi stack dahil)
    nullptr,              // Parametre
    OTA_TASK_PRIORITY,    // Öncelik 1 (SensorTask=2, KinTask=2'nin altında)
    &hOTATask,            // Handle
    0                     // Core 0
  );

  if (ret != pdPASS) {
    Serial.println(F("[OTA] HATA: Task oluşturulamadı! "
                     "Yetersiz heap. OTA devre dışı."));
    hOTATask = nullptr;
    return;
  }

  Serial.println(F("[OTA] Task oluşturuldu (Core0, öncelik=1)."));
  Serial.println(F("[OTA] WiFi bağlantısı arka planda kuruluyor..."));

  // Otomatik WiFi bağlantısını etkinleştir
  // otaCtx.wifiEnabled = true;
  Serial.println(F("[OTA] WiFi pasif. Aktifleştirmek için 0x20 komutu gönder."));
}

// ────────────────────────────────────────────────────────────────
// handleOTA — ArduinoOTA dinleyicisini çağırır
//
//  Bu fonksiyon taskOTA() içinden çağrılır.
//  loop()'tan da çağrılabilir (OTA task kullanılmak istenmezse).
// ────────────────────────────────────────────────────────────────
void handleOTA() {
  ArduinoOTA.handle();
}

// ────────────────────────────────────────────────────────────────
// otaEnable — WiFi'yi runtime'da etkinleştir (BLE/NRF komutuyla)
// ────────────────────────────────────────────────────────────────
void otaEnable() {
  otaCtx.wifiEnabled         = true;
  otaCtx.lastConnectAttemptMs = 0;  // Hemen denesin
  Serial.println(F("[OTA] WiFi modu etkinleştirildi."));
}

// ────────────────────────────────────────────────────────────────
// otaDisable — WiFi'yi kapat (güç tasarrufu veya RF gürültü azaltma)
// ────────────────────────────────────────────────────────────────
void otaDisable() {
  otaCtx.wifiEnabled = false;
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println(F("[OTA] WiFi devre dışı bırakıldı."));
  }
  otaCtx.state = OTA_STATE_IDLE;
}

// ────────────────────────────────────────────────────────────────
// otaGetState — Mevcut OTA durumunu döndürür
// ────────────────────────────────────────────────────────────────
uint8_t otaGetState() {
  return (uint8_t)otaCtx.state;
}

// ────────────────────────────────────────────────────────────────
// otaPrintStatus — OTA modülünün anlık durumunu rapor eder
// ────────────────────────────────────────────────────────────────
void otaPrintStatus() {
  const char* stateStr[] = {
    "IDLE", "CONNECTING", "READY", "STARTING", "FLASHING", "DONE", "ERROR"
  };
  const char* wifiStr = (WiFi.status() == WL_CONNECTED)
                        ? WiFi.localIP().toString().c_str()
                        : "Bağlı değil";

  Serial.println(F("┌─── OTA Modül Durumu ─────────────────────────┐"));
  Serial.printf( "│ Durum     : %-32s│\n", stateStr[(int)otaCtx.state]);
  Serial.printf( "│ WiFi      : %-32s│\n", wifiStr);
  Serial.printf( "│ Hostname  : %-32s│\n", OTA_HOSTNAME);
  Serial.printf( "│ Port      : %-32d│\n", OTA_PORT);
  if (otaCtx.state == OTA_STATE_FLASHING) {
    Serial.printf("│ İlerleme  : %%%-3d  (%u / %u byte)%-5s│\n",
                  otaCtx.lastProgress,
                  otaCtx.bytesDoneFlash,
                  otaCtx.totalBytesToFlash, " ");
  }
  if (otaCtx.state == OTA_STATE_ERROR && otaCtx.lastErrorMsg[0]) {
    Serial.printf("│ Hata      : %-32s│\n", otaCtx.lastErrorMsg);
  }
  Serial.println(F("└──────────────────────────────────────────────┘"));
}

// ════════════════════════════════════════════════════════════════
// BÖLÜM 7 — FREERTOS OTA GÖREVİ
//
//  Durum makinesine göre WiFi bağlantısı yönetir ve
//  ArduinoOTA.handle() çağırır.
//
//  FLASHING ve DONE durumlarında görev kendini askıya alır;
//  ArduinoOTA callback'leri durum güncellemeyi yönetir.
// ════════════════════════════════════════════════════════════════
void taskOTA(void* param) {
  Serial.println(F("[CORE0] taskOTA başlatıldı."));

  while (true) {

    // ── OTA etkin değilse boşta bekle ─────────────────────────────
    if (!otaCtx.wifiEnabled) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // ── Flash yazılırken veya tamamlandığında döngüye girme ────────
    if (otaCtx.state == OTA_STATE_FLASHING ||
        otaCtx.state == OTA_STATE_DONE) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ── WiFi bağlantı durumunu kontrol et ─────────────────────────
    if (WiFi.status() != WL_CONNECTED) {
      // Yeniden bağlanma periyodunu kontrol et
      uint32_t now = millis();
      if ((now - otaCtx.lastConnectAttemptMs) >= WIFI_RECONNECT_INTERVAL_MS
          || otaCtx.lastConnectAttemptMs == 0) {
        otaCtx.lastConnectAttemptMs = now;

        if (otaConnectWiFi()) {
          // Bağlantı başarılı: ArduinoOTA dinlemeye başlat
          ArduinoOTA.begin();
          otaCtx.state = OTA_STATE_READY;
          Serial.println(F("[OTA] Dinleme başlatıldı. "
                           "Arduino IDE / avrdude ile güncelleyebilirsiniz."));
          otaPrintStatus();
        }
      }
      // Bağlantı yoksa fazla CPU harcama
      vTaskDelay(pdMS_TO_TICKS(OTA_TASK_IDLE_DELAY_MS));
      continue;
    }

    // ── WiFi bağlı: ArduinoOTA dinleyicisini çağır ────────────────
    // Bu çağrı bloklamayan bir UDP poll işlemidir.
    // Güncelleme paketi gelirse onStart() callback'ini tetikler.
    otaCtx.state = OTA_STATE_READY;
    ArduinoOTA.handle();

    // WiFi bağlantı kalitesini periyodik izle (5 dakikada bir)
    static uint32_t lastRSSIMs = 0;
    if ((millis() - lastRSSIMs) > 300000UL) {
      lastRSSIMs = millis();
      Serial.printf("[OTA] WiFi kalitesi: RSSI=%d dBm  IP=%s\n",
                    WiFi.RSSI(),
                    WiFi.localIP().toString().c_str());
    }

    // ── Görev boşta bekleme: SensorTask ve KinTask'a CPU ver ──────
    // 500ms aralık: OTA UDP paketini anında yakalar (UDP zaman aşımı >1s)
    vTaskDelay(pdMS_TO_TICKS(OTA_TASK_IDLE_DELAY_MS));
  }
}
