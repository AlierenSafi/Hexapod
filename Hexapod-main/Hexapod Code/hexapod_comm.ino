// ════════════════════════════════════════════════════════════════
//  hexapod_comm.ino  ·  Haberleşme ve Paket Parser  ·  v3.1.0
//
//  Parser Katmanı:
//    parseBinaryPacket()  — 10-byte RadioPacket (BLE + NRF24)
//    parseJsonPacket()    — JSON string (BLE veya WebSocket)
//    parseLegacyPacket()  — 6-byte eski format (geriye dönük uyum)
//    dispatchMotionCmd()  — Hareket komutunu ortak havuza yazar
//
//  BLE paket boyutuna göre otomatik format tespiti:
//    4 byte   → Legacy motion (eski format)
//    10 byte  → Binary RadioPacket
//    '{' ile başlıyorsa → JSON config
//
//  NRF24: Yalnızca 10-byte RadioPacket (sabit payload)
//  WebSocket: JSON format (WiFi birincil haberleşme)
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// İÇ YARDIMCI — Paket checksum doğrulama
//
//  RadioPacket için: checksum = XOR(raw[0]..raw[8])
// ────────────────────────────────────────────────────────────────
static bool verifyChecksum(const RadioPacket& pkt) {
  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt.raw[i];
  return chk == pkt.raw[9];   // raw[9] = checksum alanı
}

// ────────────────────────────────────────────────────────────────
// dispatchMotionCmd — Hareket paketini cmdMutex korumasında yazar
//
//  Gelen komutu ctrlPkt (ortak havuz) ve lastSrc'ye yazar.
//  BLE bağlantısı kesilince çağrılan temizleyici de bunu kullanır.
// ────────────────────────────────────────────────────────────────
static void dispatchMotionCmd(const RadioPacket& pkt, CmdSource src) {
  xSemaphoreTake(cmdMutex, portMAX_DELAY);
  ctrlPkt = pkt;
  lastSrc = src;
  xSemaphoreGive(cmdMutex);
}

// ────────────────────────────────────────────────────────────────
// parseBinaryPacket — 10-byte RadioPacket'ı işler
//
//  type 0x00–0x12 → Hareket / Kalibrasyon komutları
//  type 0xF0      → Konfigürasyon paketi (updateParamById)
// ────────────────────────────────────────────────────────────────
static void parseBinaryPacket(const RadioPacket& pkt, CmdSource src) {
  // Checksum doğrula
  if (!verifyChecksum(pkt)) {
    static uint32_t lastErrMs = 0;
    if (millis() - lastErrMs > 500) {
      Serial.printf("[COMM] Checksum hatası (src=%d)\n", (int)src);
      lastErrMs = millis();
    }
    return;
  }

  // ── Konfigürasyon paketi ────────────────────────────────────────
  if (pkt.config.type == 0xF0) {
    updateParamById(pkt.config.paramId, pkt.config.value, pkt.config.flags);
    return;
  }

  // ── Hareket / sistem komutları ─────────────────────────────────
  switch (pkt.motion.type) {
    case 0x00:  // DUR
    case 0x01:  // YÜRÜ
    case 0x02:  // DÖNDÜR
    case 0x03:  // GAİT DEĞİŞTİR
      dispatchMotionCmd(pkt, src);
      break;

    case 0x10:  // Tek trim yaz (inline kalibrasyon)
    {
      uint8_t leg  = pkt.motion.cal_leg;
      uint8_t jnt  = pkt.motion.cal_joint;
      int8_t  trim = (int8_t)pkt.motion.cal_trim;
      if (leg < 6 && jnt < 3) {
        xSemaphoreTake(configMutex, portMAX_DELAY);
        cfg.servoTrim[leg][jnt] = constrain(trim, (int8_t)-30, (int8_t)30);
        xSemaphoreGive(configMutex);
        configChanged = true;
        syncHardware();
        Serial.printf("[COMM] Trim B%d E%d = %+d°\n", leg, jnt, (int)trim);
      }
      break;
    }
    case 0x11: saveConfiguration();  break;  // Flash kaydet
    case 0x12: resetToDefaults();    break;  // Fabrika sıfırlama

    // ── OTA Komutları (hexapod_ota.ino) ───────────────────────────
    case 0x20: otaEnable();      break;  // WiFi aç + OTA dinlemeye başla
    case 0x21: otaDisable();     break;  // WiFi kapat (güç tasarrufu)
    case 0x22: otaPrintStatus(); break;  // OTA durum raporu (Serial log)

    default:
      Serial.printf("[COMM] Tanınmayan type: 0x%02X\n", pkt.motion.type);
      break;
  }
}

// ────────────────────────────────────────────────────────────────
// parseLegacyPacket — 4–6 byte eski format (geriye dönük uyum)
//
//  Eski BLE uygulamaları 4 byte gönderir: [type][xRate][yRate][yaw]
//  Bu paketleri 10-byte RadioPacket'a dönüştürüp işler.
// ────────────────────────────────────────────────────────────────
static void parseLegacyPacket(const uint8_t* data, size_t len, CmdSource src) {
  if (len < 4) return;

  RadioPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.motion.type    = data[0];
  pkt.motion.xRate   = (int8_t)data[1];
  pkt.motion.yRate   = (int8_t)data[2];
  pkt.motion.yawRate = (int8_t)data[3];

  // Eski kalibrasyon komutları (4-byte format)
  if (data[0] == 0x10 && len >= 4) {
    if (len >= 6) {
      pkt.motion.cal_leg   = data[1];
      pkt.motion.cal_joint = data[2];
      pkt.motion.cal_trim  = data[3];
    }
  }

  // Eski format checksumi yok → checksum field'ı XOR ile doldur
  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt.raw[i];
  pkt.raw[9] = chk;

  parseBinaryPacket(pkt, src);
}

// ────────────────────────────────────────────────────────────────
// parseJsonPacket — JSON string config komutu (yalnızca BLE)
//
//  Format: {"k":"step_h","v":30.0,"s":1}
//    k = parametre anahtarı (NVS key string)
//    v = değer (float)
//    s = 1 ise NVS'e kaydet (isteğe bağlı, varsayılan 0)
//
//  Tam JSON parser yerine minimal string çekme kullanılır
//  (cJSON veya ArduinoJson bağımlılığı olmadan).
//
//  Desteklenen özel komutlar (v=0 ile):
//    {"k":"save"}    → saveConfiguration()
//    {"k":"load"}    → loadConfiguration()
//    {"k":"reset"}   → resetToDefaults()
//    {"k":"print"}   → printConfiguration()
// ────────────────────────────────────────────────────────────────
static void parseJsonPacket(const String& json) {
  // ── Minimal JSON alan çekicisi ─────────────────────────────────
  // String::indexOf ve substring kullanır — heap tahsisi yok
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
    // Virgül, '}', boşluğa kadar oku
    int j = i;
    while (j < (int)json.length() &&
           json[j] != ',' && json[j] != '}' && json[j] != ' ') j++;
    return json.substring(i, j).toFloat();
  };
  auto extractInt = [&](const char* field) -> int {
    return (int)extractFloat(field);
  };

  String key = extractStr("k");

  if (key.length() == 0) {
    Serial.println(F("[JSON] Hata: 'k' alanı bulunamadı."));
    return;
  }

  // Özel sistem komutları (v gerektirmez)
  if (key == "save")  { saveConfiguration();  return; }
  if (key == "load")  { loadConfiguration();
                        configChanged = true; return; }
  if (key == "reset") { resetToDefaults();    return; }
  if (key == "print") { printConfiguration(); return; }

  float value   = extractFloat("v");
  bool  saveNVS = (extractInt("s") == 1);

  updateParameter(key, value, saveNVS);
}

// ════════════════════════════════════════════════════════════════
// BLE CALLBACK SINIFLARI (Sadece USE_BLE tanımlıysa)
// ════════════════════════════════════════════════════════════════

#ifdef USE_BLE

class HexapodBLECallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String raw = pChar->getValue();
    size_t len  = raw.length();
    if (len == 0) return;

    // JSON format tespiti: '{' ile başlıyorsa
    if (raw[0] == '{') {
      parseJsonPacket(raw);
      return;
    }

    // Binary: 10 byte → tam RadioPacket
    if (len >= 10) {
      RadioPacket pkt;
      memset(&pkt, 0, sizeof(pkt));
      memcpy(pkt.raw, raw.c_str(), sizeof(pkt.raw));
      parseBinaryPacket(pkt, SRC_BLE);
      return;
    }

    // Legacy: 4–9 byte
    parseLegacyPacket((const uint8_t*)raw.c_str(), len, SRC_BLE);
  }
};

class HexapodServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer* pSvr) override {
    bleConnected = true;
    Serial.println(F("[BLE] Bağlandı."));
  }
  void onDisconnect(BLEServer* pSvr) override {
    bleConnected = false;
    // Güvenlik: bağlantı kopunca robotu durdur
    xSemaphoreTake(cmdMutex, portMAX_DELAY);
    memset(&ctrlPkt, 0, sizeof(ctrlPkt));  // type=0 = DUR
    lastSrc = SRC_NONE;
    xSemaphoreGive(cmdMutex);
    pSvr->startAdvertising();
    Serial.println(F("[BLE] Bağlantı kesildi. Reklam yeniden başlatıldı."));
  }
};

// ────────────────────────────────────────────────────────────────
// bleInit — BLE sunucusunu başlatır
// ────────────────────────────────────────────────────────────────
void bleInit() {
  BLEDevice::init("Hexapod-ESP32");
  BLEServer*  pSvr  = BLEDevice::createServer();
  pSvr->setCallbacks(new HexapodServerCallback());

  BLEService* pSvc  = pSvr->createService(BLE_SERVICE_UUID);
  pBLEChar = pSvc->createCharacteristic(
    BLE_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pBLEChar->setCallbacks(new HexapodBLECallback());
  pBLEChar->addDescriptor(new BLE2902());

  pSvc->start();
  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);  // iPhone uyumluluğu
  pAdv->start();

  Serial.println(F("[BLE] Başlatıldı. 'Hexapod-ESP32'"));
}

#endif // USE_BLE

// ════════════════════════════════════════════════════════════════
// NRF24L01+ BAŞLATMA VE ALIM
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// nrfInit — NRF24L01+ alıcı modunda başlatır
//
//  Kanal 108 (2508MHz) — WiFi/BT 2.4GHz band dışı
//  250kbps — maksimum menzil için
//  Payload: sizeof(RadioPacket) = 10 byte (sabit)
// ────────────────────────────────────────────────────────────────
bool nrfInit() {
  if (!radio.begin()) {
    Serial.println(F("[NRF24] Modül bulunamadı!"));
    Serial.println(F("[NRF24] Bağlantı: CE=GPIO5 CSN=GPIO15 SCK=18 MOSI=23 MISO=19"));
    return false;
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPayloadSize(sizeof(RadioPacket));   // 10 byte
  radio.setRetries(3, 5);
  radio.openReadingPipe(1, NRF_ADDRESS);
  radio.startListening();

  Serial.printf("[NRF24] Hazır. Kanal=108  Adres=%s  Payload=%u byte\n",
                (const char*)NRF_ADDRESS, (unsigned)sizeof(RadioPacket));
  return true;
}

// ────────────────────────────────────────────────────────────────
// nrfPoll — NRF24 alım tamponunu boşaltır (Core0 döngüsünden)
// ────────────────────────────────────────────────────────────────
void nrfPoll() {
  if (!nrfAvailable) return;

  while (radio.available()) {
    RadioPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    radio.read(&pkt, sizeof(pkt));
    parseBinaryPacket(pkt, SRC_NRF);
  }
}
