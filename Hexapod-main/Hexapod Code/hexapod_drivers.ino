// ════════════════════════════════════════════════════════════════
//  hexapod_drivers.ino  ·  PCA9685 Servo Sürücüsü  ·  v3.0.0
//
//  Tüm I2C işlemleri wireMutex korumasındadır.
//  writeAngles(), cfg'yi doğrudan okumak yerine çağıran tarafından
//  geçirilen RobotSettings pointer'ını kullanır → Core1 snapshot güvenli.
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// pca9685Init — Modülü 50Hz servo modunda başlatır
// ────────────────────────────────────────────────────────────────
void pca9685Init(uint8_t addr) {
  // Normal mod (sleep temizle)
  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(addr);
  Wire.write(PCA9685_MODE1);
  Wire.write(0x00);
  uint8_t err = Wire.endTransmission();
  xSemaphoreGive(wireMutex);

  if (err != 0) {
    Serial.printf("[DRV] HATA: PCA9685 0x%02X yanıt vermedi (err=%d)\n", addr, err);
    return;
  }
  delay(5);

  // Prescaler: round(25MHz / (4096 × freq)) – 1
  uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * (float)SERVO_FREQ) - 0.5f);

  xSemaphoreTake(wireMutex, portMAX_DELAY);
  // Sleep → Prescaler yaz → Restart → Auto-increment
  Wire.beginTransmission(addr); Wire.write(PCA9685_MODE1);   Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(addr); Wire.write(PCA9685_PRESCALE);Wire.write(prescale); Wire.endTransmission();
  Wire.beginTransmission(addr); Wire.write(PCA9685_MODE1);   Wire.write(0x80); Wire.endTransmission();
  xSemaphoreGive(wireMutex);
  delay(5);

  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(addr); Wire.write(PCA9685_MODE1); Wire.write(0x20); Wire.endTransmission();
  xSemaphoreGive(wireMutex);

  Serial.printf("[DRV] PCA9685 0x%02X başlatıldı. Prescale=%d (%dHz)\n",
                addr, prescale, SERVO_FREQ);
}

// ────────────────────────────────────────────────────────────────
// pca9685SetPWM — Kanala PWM değeri yazar
//
//  Değer s->servoMin..s->servoMax arasında kısıtlanır (donanım güvenliği).
//  Bu kısıt aşılamaz — yazılım katmanı ne kadar yanlış hesaplasa da.
// ────────────────────────────────────────────────────────────────
void pca9685SetPWM(uint8_t addr, uint8_t ch, uint16_t value,
                   const RobotSettings* s) {
  // Donanım güvenlik sınırı
  value = (uint16_t)constrain((int)value, (int)s->servoMin, (int)s->servoMax);
  uint8_t reg = LED0_ON_L + 4u * ch;

  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write((uint8_t)(value & 0xFF));
  Wire.write((uint8_t)(value >> 8));
  Wire.endTransmission();
  xSemaphoreGive(wireMutex);
}

// ────────────────────────────────────────────────────────────────
// angleToPWM — Açı (°) → PWM birimi dönüşümü
//
//  –90° → servoMin   0° → orta   +90° → servoMax
//  Giriş sınır dışı olsa bile çıkış servoMin..servoMax içinde kalır.
// ────────────────────────────────────────────────────────────────
uint16_t angleToPWM(float angleDeg, const RobotSettings* s) {
  angleDeg    = constrain(angleDeg, -90.0f, 90.0f);
  float norm  = (angleDeg + 90.0f) / 180.0f;          // 0.0 → 1.0
  float range = (float)(s->servoMax - s->servoMin);
  return (uint16_t)((float)s->servoMin + norm * range);
}

// ────────────────────────────────────────────────────────────────
// writeAngles — Eklem açılarını + trim'i servoya yazar
//
//  Sınır aşımı kontrolü (Sanity Check):
//    · Coxa:  –90°..+90° → trim eklendikten sonra da kontrol edilir
//    · Femur: –90°..+90°
//    · Tibia: –90°..+90°
//
//  @param legIdx  Bacak indeksi (0–5)
//  @param s       Kullanılacak konfigürasyon snapshot'ı
// ────────────────────────────────────────────────────────────────
void writeAngles(uint8_t legIdx, const RobotSettings* s) {
  if (legIdx >= 6 || s == nullptr) return;

  const ServoMap&    sm  = LEG_SERVO[legIdx];
  const JointAngles& a   = legs[legIdx].angles;
  int8_t             dir = LEG_DIR[legIdx];

  // Trim ekleme ve ±90° kıpkısıt
  float coxaFinal  = constrain(a.coxa * (float)dir
                              + (float)s->servoTrim[legIdx][0], -90.0f, 90.0f);
  float femurFinal = constrain(a.femur
                              + (float)s->servoTrim[legIdx][1], -90.0f, 90.0f);
  float tibiaFinal = constrain(a.tibia
                              + (float)s->servoTrim[legIdx][2], -90.0f, 90.0f);

  pca9685SetPWM(sm.addr, sm.coxa,  angleToPWM(coxaFinal,  s), s);
  pca9685SetPWM(sm.addr, sm.femur, angleToPWM(femurFinal, s), s);
  pca9685SetPWM(sm.addr, sm.tibia, angleToPWM(tibiaFinal, s), s);
}
