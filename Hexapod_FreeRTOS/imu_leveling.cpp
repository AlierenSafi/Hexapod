#include "imu_leveling.h"
#include "servo_drivers.h" // For wireMutex
#include <Wire.h>

IMUData imuData = {};
SemaphoreHandle_t imuMutex = nullptr;
bool mpuAvailable = false;

// PID States
float pidPitchInt = 0.0f, pidPitchPrev = 0.0f;
float pidRollInt = 0.0f, pidRollPrev = 0.0f;

bool mpuInit() {
  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_PWR_MGMT);
  Wire.write(0x00); // Wake up
  uint8_t err = Wire.endTransmission();
  xSemaphoreGive(wireMutex);

  if (err != 0) return false;
  delay(100);

  xSemaphoreTake(wireMutex, portMAX_DELAY);
  // Sample rate: 1000Hz / (1+9) = 100Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_SMPLRT_DIV); Wire.write(9);
  Wire.endTransmission();
  // DLPF: ~44Hz bandwidth (vibration filtering)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_CONFIG_REG); Wire.write(3);
  Wire.endTransmission();
  xSemaphoreGive(wireMutex);

  Serial.println(F("[IMU] MPU6050 initialized. 100Hz / DLPF=44Hz"));
  return true;
}

static void mpuReadRaw(float& ax, float& ay, float& az,
                       float& gx, float& gy, float& gz) {
  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  int16_t rAx = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rAy = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rAz = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Temperature (unused)
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

void updateCompFilter(float dt, const RobotSettings* s) {
  if (dt < 0.001f || dt > 0.200f) return;

  float ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  float accNorm = sqrtf(ay * ay + az * az);
  if (accNorm < 0.01f) accNorm = 0.01f;
  float accPitch = (float)RAD_TO_DEG * atan2f(-ax, accNorm);
  float accRoll  = (float)RAD_TO_DEG * atan2f(ay, az);

  float alpha = s->compAlpha;

  xSemaphoreTake(imuMutex, portMAX_DELAY);
  imuData.pitch = alpha * (imuData.pitch + gx * dt) + (1.0f - alpha) * accPitch;
  imuData.roll  = alpha * (imuData.roll  + gy * dt) + (1.0f - alpha) * accRoll;
  imuData.ax = ax; imuData.ay = ay; imuData.az = az;
  imuData.gx = gx; imuData.gy = gy; imuData.gz = gz;
  xSemaphoreGive(imuMutex);
}

void computeLevelingOffset(float dt, const RobotSettings* s, float out[6]) {
  if (!s->levelingEnabled || dt < 0.001f) {
    for (int i = 0; i < 6; i++) out[i] = 0.0f;
    return;
  }

  xSemaphoreTake(imuMutex, portMAX_DELAY);
  float pitch = imuData.pitch;
  float roll  = imuData.roll;
  xSemaphoreGive(imuMutex);

  // Pitch PID
  float pErr     = -pitch;
  pidPitchInt    = constrain(pidPitchInt + pErr * dt, -s->pidIntLimit, s->pidIntLimit);
  float pDeriv   = (pErr - pidPitchPrev) / dt;
  float pOut     = s->kp * pErr + s->ki * pidPitchInt + s->kd * pDeriv;
  pidPitchPrev   = pErr;

  // Roll PID
  float rErr     = -roll;
  pidRollInt     = constrain(pidRollInt + rErr * dt, -s->pidIntLimit, s->pidIntLimit);
  float rDeriv   = (rErr - pidRollPrev) / dt;
  float rOut     = s->kp * rErr + s->ki * pidRollInt + s->kd * rDeriv;
  pidRollPrev    = rErr;

  const float K = 8.0f; // Scale: mm/deg

  for (int i = 0; i < 6; i++) {
    float ps = (i==0||i==3) ?  1.0f
             : (i==2||i==5) ? -1.0f : 0.0f;
    float rs = (i < 3)      ?  1.0f : -1.0f;

    float offset = pOut * ps * K + rOut * rs * K;
    out[i] = constrain(offset, -s->levelingLimit, s->levelingLimit);
  }
}
