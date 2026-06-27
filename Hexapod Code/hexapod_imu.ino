// ════════════════════════════════════════════════════════════════
//  hexapod_imu.ino  ·  MPU6050 + Filtre + PID  ·  v3.0.0
//
//  PID kazançları (kp, ki, kd), Complementary Filter katsayısı
//  (compAlpha) ve levelingLimit runtime'da cfg üzerinden güncellenir.
//  PID iç durumu (pidPitchInt, pidRollInt) syncHardware() çağrısında
//  sıfırlanır — kazanç değişince windup birikimini temizler.
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// mpuInit — MPU6050'yi başlatır, 100Hz örnekleme, DLPF=44Hz
// ────────────────────────────────────────────────────────────────
bool mpuInit() {
  // Wake-up: sleep bitini temizle
  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_PWR_MGMT);
  Wire.write(0x00);
  uint8_t err = Wire.endTransmission();
  xSemaphoreGive(wireMutex);

  if (err != 0) return false;
  delay(100);

  xSemaphoreTake(wireMutex, portMAX_DELAY);
  // Örnekleme hızı: 1000Hz / (1+9) = 100Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_SMPLRT_DIV); Wire.write(9);
  Wire.endTransmission();
  // DLPF: ~44Hz bant genişliği (titreşim filtresi)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_CONFIG_REG); Wire.write(3);
  Wire.endTransmission();
  xSemaphoreGive(wireMutex);

  Serial.println(F("[IMU] MPU6050 başlatıldı. 100Hz / DLPF=44Hz"));
  return true;
}

// ────────────────────────────────────────────────────────────────
// mpuReadRaw — Ham 6-eksen veri okur
//
//  Ölçek (varsayılan register ayarı):
//    ±2g ivme   → 16384 LSB/g
//    ±250°/s gyro → 131 LSB/°/s
// ────────────────────────────────────────────────────────────────
static void mpuReadRaw(float& ax, float& ay, float& az,
                       float& gx, float& gy, float& gz) {
  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  // 14 byte: AX(2) AY(2) AZ(2) TEMP(2) GX(2) GY(2) GZ(2)
  int16_t rAx = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rAy = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rAz = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Sıcaklık — kullanılmıyor
  int16_t rGx = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rGy = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rGz = ((int16_t)Wire.read() << 8) | Wire.read();
  xSemaphoreGive(wireMutex);

  ax = (float)rAx / 16384.0f;
  ay = (float)rAy / 16384.0f;
  az = (float)rAz / 16384.0f;
  gx = (float)rGx / 131.0f;
  gy = (float)rGy / 131.0f;
  gz = (float)rGz / 131.0f;
}

// ────────────────────────────────────────────────────────────────
// updateCompFilter — Complementary Filter ile Pitch/Roll günceller
//
//  cfg.compAlpha runtime'da değişebilir → her seferinde okunur.
//  imuMutex ile thread-safe güncelleme.
//
//  @param dt  Örnekleme aralığı (saniye)
//  @param s   Konfigürasyon snapshot'ı
// ────────────────────────────────────────────────────────────────
void updateCompFilter(float dt, const RobotSettings* s) {
  // dt güvenlik kısıtı
  if (dt < 0.001f || dt > 0.200f) return;

  float ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  // İvmemetre bazlı açılar (statik referans)
  // Az sıfır koruması: atan2 tanımsızlık önlemi
  float accNorm = sqrtf(ay * ay + az * az);
  if (accNorm < 0.01f) accNorm = 0.01f;
  float accPitch = RAD_TO_DEG * atan2f(-ax, accNorm);
  float accRoll  = RAD_TO_DEG * atan2f(ay, az);

  float alpha = s->compAlpha;   // cfg'den dinamik oku

  xSemaphoreTake(imuMutex, portMAX_DELAY);
  imuData.pitch = alpha * (imuData.pitch + gx * dt) + (1.0f - alpha) * accPitch;
  imuData.roll  = alpha * (imuData.roll  + gy * dt) + (1.0f - alpha) * accRoll;
  imuData.ax = ax; imuData.ay = ay; imuData.az = az;
  imuData.gx = gx; imuData.gy = gy; imuData.gz = gz;
  xSemaphoreGive(imuMutex);
}

// ────────────────────────────────────────────────────────────────
// computeLevelingOffset — Auto-leveling PID → bacak Z offsetleri
//
//  Tüm kazançlar ve limitler cfg'den okunur → runtime'da ayarlanabilir.
//
//  PID durumu (pidPitchInt, pidRollInt, pidPitchPrev, pidRollPrev)
//  sadece Core1'de erişilir → ek mutex gerekmez.
//
//  Pitch: Ön(0,3)=+, Orta(1,4)=0, Arka(2,5)=–
//  Roll : Sağ(0-2)=+, Sol(3-5)=–
//
//  @param dt      Delta zaman (saniye)
//  @param s       Konfigürasyon snapshot'ı
//  @param out[6]  Çıkış Z offset dizisi (mm)
// ────────────────────────────────────────────────────────────────
void computeLevelingOffset(float dt, const RobotSettings* s, float out[6]) {
  if (!s->levelingEnabled || dt < 0.001f) {
    for (int i = 0; i < 6; i++) out[i] = 0.0f;
    return;
  }

  xSemaphoreTake(imuMutex, portMAX_DELAY);
  float pitch = imuData.pitch;
  float roll  = imuData.roll;
  xSemaphoreGive(imuMutex);

  // ── Pitch PID ─────────────────────────────────────────────────
  float pErr     = -pitch;
  pidPitchInt    = constrain(pidPitchInt + pErr * dt,
                             -s->pidIntLimit, s->pidIntLimit);
  float pDeriv   = (pErr - pidPitchPrev) / dt;
  float pOut     = s->kp * pErr + s->ki * pidPitchInt + s->kd * pDeriv;
  pidPitchPrev   = pErr;

  // ── Roll PID ──────────────────────────────────────────────────
  float rErr     = -roll;
  pidRollInt     = constrain(pidRollInt + rErr * dt,
                             -s->pidIntLimit, s->pidIntLimit);
  float rDeriv   = (rErr - pidRollPrev) / dt;
  float rOut     = s->kp * rErr + s->ki * pidRollInt + s->kd * rDeriv;
  pidRollPrev    = rErr;

  // ── Bacak Z offset ────────────────────────────────────────────
  const float K = 8.0f;  // mm/° ölçek (mekanik yapıya göre ayarla)

  for (int i = 0; i < 6; i++) {
    float ps = (i==0||i==3) ?  1.0f       // Ön bacak: +pitch
             : (i==2||i==5) ? -1.0f : 0.0f; // Arka: –pitch, Orta: 0
    float rs = (i < 3)      ?  1.0f : -1.0f; // Sağ: +roll, Sol: –roll

    float offset = pOut * ps * K + rOut * rs * K;
    out[i] = constrain(offset, -s->levelingLimit, s->levelingLimit);
  }
}
