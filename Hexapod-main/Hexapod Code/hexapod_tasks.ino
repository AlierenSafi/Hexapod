// ════════════════════════════════════════════════════════════════
//  hexapod_tasks.ino  ·  FreeRTOS Görevleri  ·  v3.1.0
//
//  ┌─ Core 0  taskSensorComm ──────────────────── 100Hz (10ms) ─┐
//  │  • IMU oku → Complementary Filter (cfg.compAlpha)           │
//  │  • WiFi/WebSocket loop (wifiLoop)                           │
//  │  • Telemetri gönderimi (telemetryLoop)                      │
//  │  • Batarya izleme (battLoop)                                │
//  │  • Watchdog feed + Comm timeout (wdtLoop)                   │
//  │  • NRF24 poll → parseBinaryPacket()                         │
//  │  • 6× Switch poll → phaseResetReq[] bayrağı (lock-free)    │
//  └─────────────────────────────────────────────────────────────┘
//
//  ┌─ Core 1  taskKinematics ──────────────────── 50Hz (20ms) ──┐
//  │  • Watchdog feed                                            │
//  │  • cfg'nin thread-safe snapshot'ını al (döngü başında)      │
//  │  • configChanged bayrağını işle (PID sıfırla, gait senkronla)│
//  │  • Gait faz güncelle (snapshot.gaitSpeed, swingRatio)       │
//  │  • phaseResetReq[] → anında stance fazına dön               │
//  │  • Swing → cycloidTrajectory() (snapshot.stepHeight/Length) │
//  │  • Stance → computeStanceFootPos()                          │
//  │  • Auto-leveling Z offset (snapshot.kp/ki/kd)              │
//  │  • solveIK() → Sanity Check → writeAngles(snapshot)         │
//  └─────────────────────────────────────────────────────────────┘
//
//  cfg Snapshot Mimarisi:
//    Core1, her döngüde RobotSettings'in yerel bir kopyasını alır.
//    Bu sayede cfg değişirken kinematik hesaplamalar tutarsız
//    ara değer görmez (torn read koruması).
//    configChanged bayrağı snapshot'ı yenilemek için kullanılır.
// ════════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════════
// CORE 0 — Sensör ve Haberleşme
// ════════════════════════════════════════════════════════════════
void taskSensorComm(void* param) {
  Serial.println(F("[CORE0] taskSensorComm başlatıldı."));

  // Watchdog'a abone ol
  wdtSubscribeTask(xTaskGetCurrentTaskHandle(), "SensorTask");

  // IMU ilk birkaç okumayı filtreden geçirmek için ısınma
  if (mpuAvailable) {
    RobotSettings warmSnap;
    xSemaphoreTake(configMutex, portMAX_DELAY);
    warmSnap = cfg;
    xSemaphoreGive(configMutex);
    for (int w = 0; w < 20; w++) {
      updateCompFilter(0.01f, &warmSnap);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println(F("[CORE0] IMU ısınma tamamlandı."));
  }

  // Batarya izleme başlat
  battInit();

  uint32_t lastMs = millis();

  while (true) {
    // ── Delta zaman ───────────────────────────────────────────────
    uint32_t now = millis();
    float dt = (float)(now - lastMs) * 0.001f;
    dt = constrain(dt, 0.001f, 0.100f);  // Taşma / sıfır koruması
    lastMs = now;

    // ── Watchdog feed + Comm timeout + Fault handling ─────────────
    wdtLoop();

    // ── WiFi/WebSocket loop ───────────────────────────────────────
    wifiLoop();

    // ── Telemetri gönderimi ───────────────────────────────────────
    telemetryLoop();

    // ── Batarya izleme (1Hz, içsel kontrol) ───────────────────────
    battLoop();

    // ── IMU: cfg snapshot ile filtre güncelle ─────────────────────
    if (mpuAvailable) {
      // configMutex'i kısa süre al ve compAlpha'yı oku
      float alpha;
      xSemaphoreTake(configMutex, portMAX_DELAY);
      alpha = cfg.compAlpha;
      xSemaphoreGive(configMutex);

      // Minimal inline güncelleme (tam snapshot gereksiz)
      RobotSettings imuSnap;
      imuSnap.compAlpha = alpha;
      updateCompFilter(dt, &imuSnap);
    }

    // ── NRF24: Gelen paketleri boşalt ────────────────────────────
    nrfPoll();

    // ── Switch poll: Faz sıfırlama isteği ─────────────────────────
    if (cfg.phaseResetEnabled) {   // Tek volatile okuma — mutex gerekmez
      for (uint8_t i = 0; i < 6; i++) {
        bool contact = (digitalRead(SWITCH_PINS[i]) == LOW);

        // Yükselen kenar: yeni temas VE bacak swing'deydi
        if (contact && !legs[i].switchContact && legs[i].isSwing) {
          phaseResetReq[i] = true;  // volatile bool — lock-free sinyal
        }
        legs[i].switchContact = contact;
      }
    }

    // ── Task timing izleme ───────────────────────────────────────
    markSensorTaskLoop();

    vTaskDelay(pdMS_TO_TICKS(10));   // 100Hz
  }
}

// ════════════════════════════════════════════════════════════════
// CORE 1 — Kinematik ve Servo
// ════════════════════════════════════════════════════════════════
void taskKinematics(void* param) {
  Serial.println(F("[CORE1] taskKinematics başlatıldı."));

  // Watchdog'a abone ol
  wdtSubscribeTask(xTaskGetCurrentTaskHandle(), "KinTask");

  // ── Bacak başlangıç durumu ─────────────────────────────────────
  {
    RobotSettings s;
    xSemaphoreTake(configMutex, portMAX_DELAY);
    s = cfg;
    xSemaphoreGive(configMutex);

    for (uint8_t i = 0; i < 6; i++) {
      // Stance fazı ortasından başla: tüm bacaklar zeminde
      float off = getPhaseOffset(i, (GaitType)s.gaitType);
      legs[i].phase         = off + s.swingRatio;
      if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
      legs[i].isSwing       = false;
      legs[i].switchContact = false;
      legs[i].footPos       = {0.0f, s.stanceRadius, s.stanceHeight};
      legs[i].stancePos     = legs[i].footPos;
      legs[i].angles        = {0.0f, 0.0f, 0.0f};
    }
  }

  bool prevSwing[6] = {};   // Stance→Swing geçiş tespiti

  // ── vTaskDelayUntil referans zamanı ───────────────────────────
  TickType_t lastWake = xTaskGetTickCount();

  // ── Aktif cfg snapshot (configChanged bayrağıyla güncellenir) ──
  RobotSettings snap;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  snap = cfg;
  xSemaphoreGive(configMutex);

  while (true) {
    // ── Kesin 20ms zamanlama (50Hz, jitter ~0.5ms) ────────────────
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));

    // ── Watchdog feed ─────────────────────────────────────────────
    wdtFeed();

    // ── Task timing izleme ───────────────────────────────────────
    markKinTaskLoop();

    // ── configChanged: snapshot güncelle ─────────────────────────
    // volatile bool okuma atomik → mutex gerekmez
    if (configChanged) {
      xSemaphoreTake(configMutex, portMAX_DELAY);
      snap = cfg;
      xSemaphoreGive(configMutex);
      configChanged = false;

      // Gait tipi değiştiyse fazları senkronize et
      if (snap.gaitType != currentGait) {
        currentGait = snap.gaitType;
        for (uint8_t i = 0; i < 6; i++) {
          float off = getPhaseOffset(i, currentGait);
          legs[i].phase = off + snap.swingRatio;
          if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
          prevSwing[i] = false;
        }
        const char* gn[] = {"Tripod","Ripple","Wave"};
        Serial.printf("[KIN] Gait → %s\n", gn[(int)currentGait]);
      }
    }

    // ── Kontrol paketini oku ──────────────────────────────────────
    RadioPacket pkt;
    xSemaphoreTake(cmdMutex, portMAX_DELAY);
    pkt = ctrlPkt;
    xSemaphoreGive(cmdMutex);

    // Normalize hız (–1.0 .. +1.0)
    float velX   = (float)pkt.motion.xRate   / 100.0f;
    float velY   = (float)pkt.motion.yRate   / 100.0f;
    float velYaw = (float)pkt.motion.yawRate / 100.0f;

    // Hareket var mı? (ölü band ±5% — joystick titremesi önler)
    bool moving = (pkt.motion.type == 0x01) &&
                  (fabsf(velX)   > 0.05f ||
                   fabsf(velY)   > 0.05f ||
                   fabsf(velYaw) > 0.05f);

    // Gait değişim komutu
    if (pkt.motion.type == 0x03) {
      GaitType ng = (GaitType)(abs((int)pkt.motion.xRate) % 3);
      if (ng != currentGait) {
        currentGait = ng;
        // cfg üzerinde de güncelle (senkron)
        xSemaphoreTake(configMutex, portMAX_DELAY);
        cfg.gaitType = currentGait;
        snap.gaitType = currentGait;
        xSemaphoreGive(configMutex);
        for (uint8_t i = 0; i < 6; i++) {
          float off = getPhaseOffset(i, currentGait);
          legs[i].phase = off + snap.swingRatio;
          if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
          prevSwing[i] = false;
        }
        const char* gn[] = {"Tripod","Ripple","Wave"};
        Serial.printf("[KIN] Gait komutu → %s\n", gn[(int)currentGait]);
      }
    }

    // ── Auto-leveling Z offsetleri ────────────────────────────────
    float zOff[6] = {};
    if (mpuAvailable) {
      computeLevelingOffset(0.020f, &snap, zOff);
    }

    // ── Her bacak kinematik döngüsü ───────────────────────────────
    for (uint8_t i = 0; i < 6; i++) {

      // Phase Reset: Core0'dan lock-free sinyal
      if (phaseResetReq[i]) {
        float off = getPhaseOffset(i, currentGait);
        legs[i].phase   = off + snap.swingRatio;
        if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;
        legs[i].isSwing = false;
        prevSwing[i]    = false;
        phaseResetReq[i] = false;  // Bayrağı temizle
      }

      Vec3 target;

      if (!moving) {
        // ── DURMA: Zemin pozisyonunda kal (leveling aktif) ─────────
        legs[i].isSwing = false;
        prevSwing[i]    = false;
        target = {0.0f, snap.stanceRadius, snap.stanceHeight + zOff[i]};

      } else {
        // ── HAREKETTEYİZ: Faz güncelle ──────────────────────────────
        legs[i].phase += snap.gaitSpeed;
        if (legs[i].phase >= 1.0f) legs[i].phase -= 1.0f;

        // Bacağa özel bağıl faz
        float off      = getPhaseOffset(i, currentGait);
        float relPhase = legs[i].phase - off;
        if (relPhase < 0.0f) relPhase += 1.0f;

        bool nowSwing = (relPhase < snap.swingRatio);
        legs[i].isSwing = nowSwing;

        // Stance→Swing geçişi: swing başlangıcını güncelle
        if (!prevSwing[i] && nowSwing) {
          legs[i].stancePos = legs[i].footPos;
        }

        if (nowSwing) {
          // ── SWING: Sikloid yörünge ───────────────────────────────
          float swingT = relPhase / snap.swingRatio;
          swingT = constrain(swingT, 0.0f, 1.0f);

          // Swing bitiş hedefi: hareket yönüne göre yarım adım öte
          Vec3 swingEnd = {
            velX    * snap.stepLength * 0.55f,
            snap.stanceRadius + velY * snap.stepLength * 0.55f,
            snap.stanceHeight
          };

          target = cycloidTrajectory(swingT, legs[i].stancePos,
                                     swingEnd, snap.stepHeight);
        } else {
          // ── STANCE: Zemin itişi ─────────────────────────────────
          float stanceT = (relPhase - snap.swingRatio)
                          / (1.0f - snap.swingRatio);
          stanceT = constrain(stanceT, 0.0f, 1.0f);
          target  = computeStanceFootPos(i, stanceT,
                                         velX, velY, velYaw, &snap);
        }

        // Auto-leveling Z ekle
        target.z += zOff[i];
        prevSwing[i] = nowSwing;
      }

      // ── IK + Servo yazma ──────────────────────────────────────────
      legs[i].footPos = target;

      if (solveIK(i, target, &snap)) {
        writeAngles(i, &snap);

      } else {
        // ── IK Başarısız: Recovery ────────────────────────────────
        // Throttled log (500ms'de bir uyarı — döngüyü yavaşlatmaz)
        static uint32_t ikErrMs[6] = {};
        if ((millis() - ikErrMs[i]) > 500) {
          Serial.printf("[IK] Bacak%d ulaşılamaz: "
                        "X=%.1f Y=%.1f Z=%.1f\n",
                        i, target.x, target.y, target.z);
          ikErrMs[i] = millis();
        }

        // Güvenli stance merkezine çek (mekanik hasar önleme)
        Vec3 safe = {0.0f, snap.stanceRadius, snap.stanceHeight};
        if (solveIK(i, safe, &snap)) {
          writeAngles(i, &snap);
          legs[i].footPos = safe;  // Gerçek pozisyonu güncelle
        }
      }

    }  // for her bacak

  }  // while(true)
}
