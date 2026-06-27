#include "kinematics.h"
#include <freertos/task.h>

const float TRIPOD_PHASE[6] = {0.00f, 0.50f, 0.00f, 0.50f, 0.00f, 0.50f};
const float RIPPLE_PHASE[6] = {0.00f, 0.33f, 0.66f, 0.50f, 0.83f, 0.16f};
const float WAVE_PHASE[6]   = {0.00f, 0.17f, 0.33f, 0.50f, 0.67f, 0.83f};

GaitType currentGait = TRIPOD;

float getPhaseOffset(uint8_t legIdx, GaitType gait) {
  if (legIdx >= 6) return 0.0f;
  switch (gait) {
    case TRIPOD: return TRIPOD_PHASE[legIdx];
    case RIPPLE: return RIPPLE_PHASE[legIdx];
    case WAVE:   return WAVE_PHASE[legIdx];
    default:     return 0.0f;
  }
}

Vec3 computeStanceFootPos(uint8_t legIdx, float stanceT,
                           float velX, float velY, float velYaw,
                           const RobotSettings* s) {
  stanceT = constrain(stanceT, 0.0f, 1.0f);

  float lx = LEG_ORIGIN[legIdx].x;
  float ly = LEG_ORIGIN[legIdx].y;
  float legDist = sqrtf(lx * lx + ly * ly);

  float yawSign    = (legIdx < 3) ? -1.0f : 1.0f;
  float yawContrib = velYaw * legDist * 0.28f * yawSign;

  float phase = 0.5f - stanceT;

  Vec3 pos;
  pos.x = (velX    * s->stepLength + yawContrib) * phase;
  pos.y = s->stanceRadius + velY * s->stepLength * phase;
  pos.z = s->stanceHeight;
  return pos;
}

void sitDown() {
  Serial.println(F("[GAIT] Sitting down..."));

  RobotSettings s;
  xSemaphoreTake(configMutex, portMAX_DELAY);
  s = cfg;
  xSemaphoreGive(configMutex);

  const float SIT_Y = s.coxaLen + s.femurLen * 0.707f;
  const float SIT_Z = -(s.femurLen * 0.707f + s.tibiaLen * 0.60f);
  const int STEPS = 60;

  for (int step = 0; step <= STEPS; step++) {
    float t = (float)step / (float)STEPS;

    for (uint8_t i = 0; i < 6; i++) {
      Vec3 target;
      target.x = 0.0f;
      target.y = s.stanceRadius * (1.0f - t) + SIT_Y * t;
      target.z = s.stanceHeight * (1.0f - t) + SIT_Z * t;

      if (solveIK(i, target, &s)) {
        writeAngles(i, &s);
      }
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(16));
    } else {
      delay(16);
    }
  }

  Serial.println(F("[GAIT] Sitting down complete."));
}

bool solveIK(uint8_t legIdx, const Vec3& target, const RobotSettings* s) {
  if (legIdx >= 6 || s == nullptr) return false;

  JointAngles& a = legs[legIdx].angles;

  a.coxa = (float)RAD_TO_DEG * atan2f(target.x, target.y);

  float horizDist = sqrtf(target.x * target.x + target.y * target.y);
  float L  = horizDist - s->coxaLen;
  float Z  = target.z;

  float D = sqrtf(L * L + Z * Z);

  float dMax = s->femurLen + s->tibiaLen - 0.5f;
  float dMin = fabsf(s->femurLen - s->tibiaLen) + 0.5f;

  if (!isfinite(D) || D > dMax || D < dMin) {
    return false;
  }

  float cosT2 = (D*D - s->femurLen*s->femurLen - s->tibiaLen*s->tibiaLen)
                / (2.0f * s->femurLen * s->tibiaLen);
  cosT2 = constrain(cosT2, -1.0f, 1.0f);
  a.tibia = (float)RAD_TO_DEG * (acosf(cosT2) - (float)M_PI);

  float alpha = atan2f(-Z, L);
  float cosT1 = (D*D + s->femurLen*s->femurLen - s->tibiaLen*s->tibiaLen)
                / (2.0f * D * s->femurLen);
  cosT1 = constrain(cosT1, -1.0f, 1.0f);
  float beta = acosf(cosT1);

  a.femur = (float)RAD_TO_DEG * (alpha + beta);

  if (!isfinite(a.coxa)  || fabsf(a.coxa)  > 91.0f) return false;
  if (!isfinite(a.femur) || fabsf(a.femur) > 91.0f) return false;
  if (!isfinite(a.tibia) || fabsf(a.tibia) > 91.0f) return false;

  return true;
}

Vec3 cycloidTrajectory(float t, const Vec3& start, const Vec3& end, float height) {
  t = constrain(t, 0.0f, 1.0f);

  float tS = t - (sinf(TWO_PI * t) / TWO_PI);

  Vec3 pos;
  pos.x = start.x + (end.x - start.x) * tS;
  pos.y = start.y + (end.y - start.y) * tS;
  pos.z = (start.z + (end.z - start.z) * t) + height * sinf((float)M_PI * t);

  return pos;
}
