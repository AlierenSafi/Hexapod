// ════════════════════════════════════════════════════════════════
//  hexapod_battery.ino  ·  Batarya İzleme ve Koruma  ·  v3.1.0
//
//  Bu modül 2S LiPo bataryanın voltajını izler ve düşük voltaj
//  durumlarında robotu güvenli şekilde durdurur.
//
//  Donanım:
//    • ADC pin: GPIO 36 (VP, ADC1_CH0)
//    • Voltaj bölücü: 20kΩ + 10kΩ (Vout = Vin/3)
//    • Filtre: 100nF paralel kapasitör
//
//  FSM (Finite State Machine):
//    NORMAL → LOW_WARN → CRITICAL → SHUTDOWN
//
//  Kalibrasyon:
//    • esp_adc_cal kullanarak factory calibration
//    • Aksi halde ±0.3V hata olabilir
// ════════════════════════════════════════════════════════════════

#include <esp_adc_cal.h>
#include <driver/adc.h>  // ESP-IDF ADC driver

// ════════════════════════════════════════════════════════════════
// İLERİ BİLDİRİMLER (hexapod_esp32_v3.ino'da tanımlı)
// ════════════════════════════════════════════════════════════════

// WiFiState enum'u hexapod_esp32_v3.ino'da tanımlı
extern WiFiState wifiState;

// ════════════════════════════════════════════════════════════════
// SABİTLER
// ════════════════════════════════════════════════════════════════

#define BATT_ADC_SAMPLES      32      // Oversampling sayısı
#define BATT_FILTER_SIZE      8       // Moving average pencere boyutu
#define BATT_READ_INTERVAL_MS 1000    // Okuma aralığı (1Hz)

// Histerezis değerleri (bouncing önleme)
#define BATT_HYSTERESIS_V     0.15f

// 2S LiPo discharge curve (voltaj → yüzde)
// Lineer interpolasyon kullanılır
static const float BATT_CURVE_V[] = {6.0f, 6.4f, 7.0f, 7.4f, 8.2f, 8.4f};
static const float BATT_CURVE_P[] = {0.0f, 5.0f, 20.0f, 50.0f, 90.0f, 100.0f};

// ════════════════════════════════════════════════════════════════
// GLOBAL DEĞİŞKENLER
// ════════════════════════════════════════════════════════════════

static esp_adc_cal_characteristics_t adcChars;
static bool adcCalibrated = false;
static float voltageFilter[BATT_FILTER_SIZE];
static uint8_t filterIndex = 0;
static uint32_t lastReadMs = 0;
static BattLevel lastLevel = BATT_NORMAL;

// ════════════════════════════════════════════════════════════════
// BAŞLATMA
// ════════════════════════════════════════════════════════════════

void battInit() {
  Serial.println(F("\n[BATT] Batarya izleme başlatılıyor..."));
  
  // ADC1 kanalını yapılandır
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);  // 0-3.3V
  
  // Factory calibration karakteristikleri
  esp_adc_cal_value_t calType = esp_adc_cal_characterize(
    ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
    1100,  // Default Vref (mV), factory varsa kullanılmaz
    &adcChars
  );
  
  if (calType == ESP_ADC_CAL_VAL_EFUSE_VREF ||
      calType == ESP_ADC_CAL_VAL_EFUSE_TP) {
    adcCalibrated = true;
    Serial.println(F("[BATT] ADC factory calibration aktif."));
  } else {
    adcCalibrated = false;
    Serial.println(F("[BATT] UYARI: ADC factory calibration yok! ±0.3V hata olabilir."));
  }
  
  // Filtre buffer'ını sıfırla
  for (int i = 0; i < BATT_FILTER_SIZE; i++) {
    voltageFilter[i] = 7.4f;  // Nominal voltaj
  }
  
  // İlk okuma
  battReadVoltage();
  
  Serial.printf("[BATT] Eşikler: Warn=%.2fV  Crit=%.2fV  Cutoff=%.2fV\n",
                cfg.battWarnVolt, cfg.battCritVolt, cfg.battCutoffVolt);
  Serial.printf("[BATT] Anlık: %.2fV  (%%%.0f)\n",
                battState.voltage, battState.percentage);
}

// ════════════════════════════════════════════════════════════════
// VOLTaj OKUMA ve FİLTRELEME
// ════════════════════════════════════════════════════════════════

float battReadVoltage() {
  uint32_t now = millis();
  if (now - lastReadMs < 100) {
    return battState.voltage;  // Çok sık okuma yapma
  }
  lastReadMs = now;
  
  // Oversampling
  uint32_t adcSum = 0;
  for (int i = 0; i < BATT_ADC_SAMPLES; i++) {
    adcSum += adc1_get_raw(ADC1_CHANNEL_0);
    delayMicroseconds(100);  // Kısa bekleme
  }
  uint32_t adcAvg = adcSum / BATT_ADC_SAMPLES;
  
  // Voltaja çevir (mV → V)
  uint32_t voltage_mV;
  if (adcCalibrated) {
    voltage_mV = esp_adc_cal_raw_to_voltage(adcAvg, &adcChars);
  } else {
    voltage_mV = (adcAvg * 3300) / 4095;  // Basit lineer
  }
  
  // Bölücü telafisi (Vout = Vin/3 → Vin = Vout × 3)
  float voltage = (voltage_mV / 1000.0f) * 3.0f;
  
  // Moving average filtresi
  voltageFilter[filterIndex] = voltage;
  filterIndex = (filterIndex + 1) % BATT_FILTER_SIZE;
  
  float avgVoltage = 0;
  for (int i = 0; i < BATT_FILTER_SIZE; i++) {
    avgVoltage += voltageFilter[i];
  }
  avgVoltage /= BATT_FILTER_SIZE;
  
  // State güncelle
  battState.voltage = avgVoltage;
  battState.percentage = voltageToPercentage(avgVoltage);
  battState.lastReadMs = now;
  
  return avgVoltage;
}

// ════════════════════════════════════════════════════════════════
// VOLTaj → YÜZDE DÖNÜŞÜMÜ
// ════════════════════════════════════════════════════════════════

float voltageToPercentage(float voltage) {
  // Eşik kontrolü
  if (voltage >= BATT_CURVE_V[5]) return 100.0f;
  if (voltage <= BATT_CURVE_V[0]) return 0.0f;
  
  // Lineer interpolasyon
  for (int i = 0; i < 5; i++) {
    if (voltage >= BATT_CURVE_V[i] && voltage < BATT_CURVE_V[i+1]) {
      float t = (voltage - BATT_CURVE_V[i]) / (BATT_CURVE_V[i+1] - BATT_CURVE_V[i]);
      return BATT_CURVE_P[i] + t * (BATT_CURVE_P[i+1] - BATT_CURVE_P[i]);
    }
  }
  
  return 0.0f;
}

// ════════════════════════════════════════════════════════════════
// FSM (Finite State Machine)
// ════════════════════════════════════════════════════════════════

void battCheckLevels() {
  float v = battState.voltage;
  BattLevel newLevel = lastLevel;
  
  switch (lastLevel) {
    case BATT_FULL:
    case BATT_NORMAL:
      if (v < cfg.battWarnVolt) {
        newLevel = BATT_LOW_WARN;
        sendLowVoltageWarning(v);
        sysState.battLow = true;
      }
      break;
      
    case BATT_LOW_WARN:
      // Histerezis: eşiğin üstünde +0.15V olmalı temizlenmek için
      if (v >= cfg.battWarnVolt + BATT_HYSTERESIS_V) {
        newLevel = BATT_NORMAL;
        sysState.battLow = false;
      }
      else if (v < cfg.battCritVolt) {
        newLevel = BATT_CRITICAL;
        sendCriticalVoltage(v);
        battEmergencyAction();
      }
      break;
      
    case BATT_CRITICAL:
      if (v >= cfg.battCritVolt + BATT_HYSTERESIS_V) {
        newLevel = BATT_LOW_WARN;
      }
      else if (v < cfg.battCutoffVolt) {
        newLevel = BATT_SHUTDOWN;
        battShutdown();
      }
      break;
      
    case BATT_SHUTDOWN:
      // Geri dönüş yok - deep sleep'ten reset gerekir
      break;
  }
  
  if (newLevel != lastLevel) {
    Serial.printf("[BATT] Seviye değişti: %d → %d  (%.2fV)\n",
                  lastLevel, newLevel, v);
    lastLevel = newLevel;
  }
  
  battState.level = lastLevel;
}

// ════════════════════════════════════════════════════════════════
// ANA DÖNGÜ (taskSensorComm'dan çağrılır)
// ════════════════════════════════════════════════════════════════

void battLoop() {
  uint32_t now = millis();
  if (now - lastReadMs < BATT_READ_INTERVAL_MS) return;
  
  battReadVoltage();
  battCheckLevels();
}

// ════════════════════════════════════════════════════════════════
// ACİL DURUM EYLEMLERİ
// ════════════════════════════════════════════════════════════════

void battEmergencyAction() {
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  [BATT] KRİTİK VOLTaj - ACİL DURUM    ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));
  
  // 1. Robotu güvenli pozisyona getir
  sitDown();
  delay(500);
  
  // 2. Servo torkunu kes
  digitalWrite(OE_PIN, HIGH);
  Serial.println(F("[BATT] Servolar devre dışı bırakıldı (OE=HIGH)."));
  
  // 3. Event gönder
  sendEvent("batt_emergency", "Kritik voltaj - Servolar devre dışı");
}

void battShutdown() {
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  [BATT] BATARYA TÜKENDİ - UYKU MODU   ║"));
  Serial.println(F("╚════════════════════════════════════════╝\n"));
  
  // 1. Son event
  sendEvent("batt_shutdown", "Deep sleep'e geçiliyor");
  delay(100);
  
  // 2. WiFi ve diğer sistemleri kapat
  // WiFi kütüphanesi zaten hexapod_wifi.ino'da include edildi
  // Burada sadece durum değişkenini ayarla
  wifiState = WIFI_DISCONNECTED;
  
  // 3. Deep sleep (butonla uyanma veya timer)
  // GPIO 0 (BOOT butonu) ile uyanma
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  
  Serial.println(F("[BATT] Deep sleep başlatılıyor..."));
  Serial.flush();
  
  esp_deep_sleep_start();
}

// ════════════════════════════════════════════════════════════════
// YARDIMCI FONKSİYONLAR
// ════════════════════════════════════════════════════════════════

const char* battLevelToString(BattLevel lvl) {
  switch (lvl) {
    case BATT_SHUTDOWN: return "SHUTDOWN";
    case BATT_CRITICAL: return "CRITICAL";
    case BATT_LOW_WARN: return "LOW_WARN";
    case BATT_NORMAL:   return "NORMAL";
    case BATT_FULL:     return "FULL";
    default:            return "UNKNOWN";
  }
}

void battPrintStatus() {
  Serial.println(F("┌─── Batarya Durumu ─────────────────────────┐"));
  Serial.printf( "│ Voltaj:  %.2fV\n", battState.voltage);
  Serial.printf( "│ Yüzde:   %.0f%%\n", battState.percentage);
  Serial.printf( "│ Seviye:  %s\n", battLevelToString(battState.level));
  Serial.printf( "│ ADC:     %s\n", adcCalibrated ? "Kalibre" : "Kalibesiz!");
  Serial.printf( "│ Eşikler: W=%.1fV C=%.1fV K=%.1fV\n",
                 cfg.battWarnVolt, cfg.battCritVolt, cfg.battCutoffVolt);
  Serial.println(F("└────────────────────────────────────────────┘"));
}
