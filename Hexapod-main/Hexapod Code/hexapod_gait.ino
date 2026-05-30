// ════════════════════════════════════════════════════════════════
//  hexapod_gait.ino  ·  Yürüyüş Planlayıcı  ·  v3.0.0
//
//  Tüm hareket parametreleri (stepHeight, stepLength, stanceRadius,
//  stanceHeight, swingRatio, gaitSpeed) RobotSettings üzerinden
//  dinamik olarak okunur — runtime'da kumandayla değiştirilebilir.
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// getPhaseOffset — Gait tipine göre bacak faz ofseti
// ────────────────────────────────────────────────────────────────
float getPhaseOffset(uint8_t legIdx, GaitType gait) {
  if (legIdx >= 6) return 0.0f;
  switch (gait) {
    case TRIPOD: return TRIPOD_PHASE[legIdx];
    case RIPPLE: return RIPPLE_PHASE[legIdx];
    case WAVE:   return WAVE_PHASE[legIdx];
    default:     return 0.0f;
  }
}

// ────────────────────────────────────────────────────────────────
// computeStanceFootPos — Stance fazında bacağın anlık pozisyonu
//
//  Zemin çerçevesinde ayak sabit durur; gövde hareket eder.
//  Bacak koordinatlarında ayak, hareketin tersine kayar.
//
//  Yaw diferansiyeli: Bacak, gövde merkezine ne kadar uzaksa
//  o kadar fazla dönüş hız katkısı alır (gerçek arazi aracı dinamiği).
//
//  velYaw > 0 = saat yönünde (CW) dönüş (yukarıdan bakış):
//    Sağ bacaklar (0-1-2) geriye, sol bacaklar (3-4-5) öne gider.
//
//  @param stanceT  [0=stance başı, 1=stance sonu]
//  @param s        Konfigürasyon snapshot'ı
// ────────────────────────────────────────────────────────────────
Vec3 computeStanceFootPos(uint8_t legIdx, float stanceT,
                           float velX, float velY, float velYaw,
                           const RobotSettings* s) {
  stanceT = constrain(stanceT, 0.0f, 1.0f);

  float lx = LEG_ORIGIN[legIdx].x;
  float ly = LEG_ORIGIN[legIdx].y;
  float legDist = sqrtf(lx * lx + ly * ly);

  // Yaw katkı işareti: sağ bacaklar (0,1,2) negatif, sol pozitif
  float yawSign    = (legIdx < 3) ? -1.0f : 1.0f;
  float yawContrib = velYaw * legDist * 0.28f * yawSign;

  // Stance yayı: +0.5'ten –0.5'e (ortalandı)
  float phase = 0.5f - stanceT;

  Vec3 pos;
  pos.x = (velX    * s->stepLength + yawContrib) * phase;
  pos.y = s->stanceRadius + velY * s->stepLength * phase;
  pos.z = s->stanceHeight;
  return pos;
}

// ────────────────────────────────────────────────────────────────
// sitDown — Tüm bacakları güvenli oturma pozisyonuna getirir
//
//  STEPS adımlık interpolasyon ile ani servo hareketi önlenir.
//  Hedef pozu mevcut cfg'den hesaplar → parametre değişse bile doğru.
// ────────────────────────────────────────────────────────────────
void sitDown() {
  Serial.println(F("[GAIT] Oturma pozisyonuna geçiliyor..."));

  // Anlık cfg snapshot'ı al
  RobotSettings s;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  s = cfg;
  xSemaphoreGive(configMutex);

  // Oturma hedef koordinatları (bacak köküne göre, mm)
  // Femur ~45° eğimli, tibia ~dik → dengeli oturma pozu
  const float SIT_Y = s.coxaLen + s.femurLen * 0.707f;
  const float SIT_Z = -(s.femurLen * 0.707f + s.tibiaLen * 0.60f);

  const int STEPS = 60;    // 60 adım × 16ms ≈ 960ms

  for (int step = 0; step <= STEPS; step++) {
    float t = (float)step / (float)STEPS;   // 0.0 → 1.0

    for (uint8_t i = 0; i < 6; i++) {
      Vec3 target;
      target.x = 0.0f;
      target.y = s.stanceRadius * (1.0f - t) + SIT_Y * t;
      target.z = s.stanceHeight * (1.0f - t) + SIT_Z * t;

      if (solveIK(i, target, &s)) {
        writeAngles(i, &s);
      }
    }
    delay(16);
  }

  Serial.println(F("[GAIT] Oturma tamamlandı."));
}
