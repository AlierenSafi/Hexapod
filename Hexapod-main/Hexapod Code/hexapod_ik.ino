// ════════════════════════════════════════════════════════════════
//  hexapod_ik.ino  ·  Ters Kinematik + Sikloid Yörünge  ·  v3.0.0
//
//  Tüm kinematik hesaplamalar RobotSettings pointer'ından okur.
//  Bu, bacak uzunluklarının (coxa/femur/tibia) runtime'da
//  değiştirilmesine olanak sağlar — tek satır IK kodu değişmez.
//
//  IK Koordinat Sistemi (bacak köküne göre):
//    Y = Radyal dış yön (bacağın nominal baktığı yön)
//    X = Y'ye dik (ileri/geri swing yönü)
//    Z = Yukarı (+), Aşağı (–)
// ════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────
// solveIK — Hedef ayak pozisyonundan 3 eklem açısını hesaplar
//
//  Sanity Check: Hedef erişim alanı dışındaysa false döner.
//  Çağıran, false durumunda son geçerli açıları korumalıdır.
//
//  @param legIdx   0–5
//  @param target   Hedef pozisyon (bacak köküne göre, mm)
//  @param s        Konfigürasyon snapshot'ı
//  @return         true = geçerli çözüm
// ────────────────────────────────────────────────────────────────
bool solveIK(uint8_t legIdx, const Vec3& target, const RobotSettings* s) {
  if (legIdx >= 6 || s == nullptr) return false;

  JointAngles& a = legs[legIdx].angles;

  // ── Coxa: Yatay dönüş açısı ──────────────────────────────────
  // atan2 tanımsızlık: target.x=0, target.y=0 → coxa=0 (güvenli)
  a.coxa = RAD_TO_DEG * atan2f(target.x, target.y);

  // ── Femur pivot'a olan mesafeler ─────────────────────────────
  float horizDist = sqrtf(target.x * target.x + target.y * target.y);
  float L  = horizDist - s->coxaLen;  // Coxa çıktıktan sonra kalan yatay
  float Z  = target.z;

  // 2D mesafe: femur pivot → hedef ayak
  float D = sqrtf(L * L + Z * Z);

  // ── Erişim alanı denetimi (Sanity Check) ─────────────────────
  float dMax = s->femurLen + s->tibiaLen - 0.5f;  // Epsilon güvenlik marjı
  float dMin = fabsf(s->femurLen - s->tibiaLen) + 0.5f;

  if (!isfinite(D) || D > dMax || D < dMin) {
    // Hedefe ulaşılamıyor — çözüm yok
    return false;
  }

  // ── Tibia (Kosinüs Teoremi) ───────────────────────────────────
  // cos(θ_tibia) = (D² – f² – t²) / (2ft)
  float cosT2 = (D*D - s->femurLen*s->femurLen - s->tibiaLen*s->tibiaLen)
                / (2.0f * s->femurLen * s->tibiaLen);
  cosT2 = constrain(cosT2, -1.0f, 1.0f);
  // Servo referansı: tam uzanım = 0°, dik bükülmüş ≈ –90°
  a.tibia = RAD_TO_DEG * (acosf(cosT2) - (float)M_PI);

  // ── Femur (Kosinüs Teoremi + zemin referansı açısı) ──────────
  // α: yatay referanstan hedefe açı (Z negatif → ayak aşağıda)
  float alpha = atan2f(-Z, L);

  float cosT1 = (D*D + s->femurLen*s->femurLen - s->tibiaLen*s->tibiaLen)
                / (2.0f * D * s->femurLen);
  cosT1 = constrain(cosT1, -1.0f, 1.0f);
  float beta = acosf(cosT1);

  a.femur = RAD_TO_DEG * (alpha + beta);

  // ── Final açı sınır denetimi (Sanity Check) ──────────────────
  // Servo mekanik limitlerinin çok ötesindeki açılar red edilir
  if (!isfinite(a.coxa)  || fabsf(a.coxa)  > 91.0f) return false;
  if (!isfinite(a.femur) || fabsf(a.femur) > 91.0f) return false;
  if (!isfinite(a.tibia) || fabsf(a.tibia) > 91.0f) return false;

  return true;
}

// ────────────────────────────────────────────────────────────────
// cycloidTrajectory — Sarsıntısız swing yörüngesi (Sikloid)
//
//  Hız profili: v(t) = 1 – cos(2πt)
//  → t=0 ve t=1'de hız tam sıfır → yumuşak kalkış ve iniş
//
//  Yatay: x(t) = Δ × [t – sin(2πt) / 2π]   ← sikloid integrali
//  Dikey: z(t) = lerp(sz, ez, t) + height × sin(πt)
//
//  @param t       Normalize zaman [0.0, 1.0]
//  @param start   Swing başlangıç noktası
//  @param end     Swing bitiş noktası (hedef yer temas)
//  @param height  Tepe kaldırma yüksekliği (mm), cfg.stepHeight
// ────────────────────────────────────────────────────────────────
Vec3 cycloidTrajectory(float t, const Vec3& start, const Vec3& end, float height) {
  t = constrain(t, 0.0f, 1.0f);

  // Sikloid pozisyon integrali: t=0 → 0, t=1 → 1 (uçlarda türev=0)
  float tS = t - (sinf(TWO_PI * t) / TWO_PI);

  Vec3 pos;
  pos.x = start.x + (end.x - start.x) * tS;
  pos.y = start.y + (end.y - start.y) * tS;
  // Dikey: doğrusal + sinüsoidal kaldırma (t=0 ve t=1'de ekstra yükseklik=0)
  pos.z = (start.z + (end.z - start.z) * t) + height * sinf((float)M_PI * t);

  return pos;
}
