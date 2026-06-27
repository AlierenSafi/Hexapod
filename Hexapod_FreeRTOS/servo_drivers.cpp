#include "servo_drivers.h"
#include <Wire.h>

SemaphoreHandle_t wireMutex = nullptr;
LegState legs[6];

// Static mappings
const int8_t LEG_DIR[6] = {1, 1, 1, -1, -1, -1};

const ServoMap LEG_SERVO[6] = {
  {PCA9685_RIGHT,  0,  1,  2},  // Leg 0: Right Front
  {PCA9685_RIGHT,  3,  4,  5},  // Leg 1: Right Middle
  {PCA9685_RIGHT,  6,  7,  8},  // Leg 2: Right Back
  {PCA9685_LEFT,   0,  1,  2},  // Leg 3: Left Front
  {PCA9685_LEFT,   3,  4,  5},  // Leg 4: Left Middle
  {PCA9685_LEFT,   6,  7,  8},  // Leg 5: Left Back
};

const Vec2 LEG_ORIGIN[6] = {
  { 60.0f,  80.0f},  // 0: Right Front
  { 80.0f,   0.0f},  // 1: Right Middle
  { 60.0f, -80.0f},  // 2: Right Back
  {-60.0f,  80.0f},  // 3: Left Front
  {-80.0f,   0.0f},  // 4: Left Middle
  {-60.0f, -80.0f},  // 5: Left Back
};

void pca9685Init(uint8_t addr) {
  // Wake-up
  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(addr);
  Wire.write(PCA9685_MODE1);
  Wire.write(0x00);
  uint8_t err = Wire.endTransmission();
  xSemaphoreGive(wireMutex);

  if (err != 0) {
    Serial.printf("[DRV] ERROR: PCA9685 0x%02X not responding (err=%d)\n", addr, err);
    return;
  }
  delay(5);

  uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * (float)SERVO_FREQ) - 0.5f);

  xSemaphoreTake(wireMutex, portMAX_DELAY);
  // Sleep -> Prescale -> Restart -> Enable Auto-Increment (AI=0x20 in MODE1)
  Wire.beginTransmission(addr); Wire.write(PCA9685_MODE1);    Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(addr); Wire.write(PCA9685_PRESCALE); Wire.write(prescale); Wire.endTransmission();
  Wire.beginTransmission(addr); Wire.write(PCA9685_MODE1);    Wire.write(0x80); Wire.endTransmission();
  xSemaphoreGive(wireMutex);
  delay(5);

  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(addr); Wire.write(PCA9685_MODE1);    Wire.write(0x20); Wire.endTransmission(); // Set Auto-Increment
  xSemaphoreGive(wireMutex);

  Serial.printf("[DRV] PCA9685 0x%02X initialized. Prescale=%d (%dHz)\n", addr, prescale, SERVO_FREQ);
}

void pca9685SetPWM(uint8_t addr, uint8_t ch, uint16_t value, const RobotSettings* s) {
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

uint16_t angleToPWM(float angleDeg, const RobotSettings* s) {
  angleDeg    = constrain(angleDeg, -90.0f, 90.0f);
  float norm  = (angleDeg + 90.0f) / 180.0f;
  float range = (float)(s->servoMax - s->servoMin);
  return (uint16_t)((float)s->servoMin + norm * range);
}

void writeAngles(uint8_t legIdx, const RobotSettings* s) {
  if (legIdx >= 6 || s == nullptr) return;

  const ServoMap&    sm  = LEG_SERVO[legIdx];
  const JointAngles& a   = legs[legIdx].angles;
  int8_t             dir = LEG_DIR[legIdx];

  float coxaFinal  = constrain(a.coxa * (float)dir
                              + (float)s->servoTrim[legIdx][0], -90.0f, 90.0f);
  float femurFinal = constrain(a.femur
                              + (float)s->servoTrim[legIdx][1], -90.0f, 90.0f);
  float tibiaFinal = constrain(a.tibia
                              + (float)s->servoTrim[legIdx][2], -90.0f, 90.0f);

  uint16_t pwm[3];
  pwm[0] = angleToPWM(coxaFinal, s);
  pwm[1] = angleToPWM(femurFinal, s);
  pwm[2] = angleToPWM(tibiaFinal, s);

  // Auto-Increment Batch write - writes 3 servos in one single I2C transaction
  xSemaphoreTake(wireMutex, portMAX_DELAY);
  Wire.beginTransmission(sm.addr);
  Wire.write(LED0_ON_L + 4u * sm.coxa);
  for (int i = 0; i < 3; i++) {
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write((uint8_t)(pwm[i] & 0xFF));
    Wire.write((uint8_t)(pwm[i] >> 8));
  }
  Wire.endTransmission();
  xSemaphoreGive(wireMutex);
}
