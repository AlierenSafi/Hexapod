#ifndef SERVO_DRIVERS_H
#define SERVO_DRIVERS_H

#include <Arduino.h>
#include "robot_config.h"

#define PCA9685_RIGHT    0x40
#define PCA9685_LEFT     0x41
#define PCA9685_MODE1    0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L        0x06
#define SERVO_FREQ       50
#define OE_PIN           4

struct ServoMap { uint8_t addr, coxa, femur, tibia; };

extern const Vec2 LEG_ORIGIN[6];
extern const int8_t LEG_DIR[6];
extern const ServoMap LEG_SERVO[6];
extern SemaphoreHandle_t wireMutex;
extern bool pcaAvailable;

// Eklem açıları yapısı (aynı zamanda kinematics modülünde de kullanılır)
struct JointAngles {
  float coxa;
  float femur;
  float tibia;
};

// Bacak durum yapısı
struct LegState {
  Vec3        footPos;
  Vec3        stancePos;
  float       phase;
  bool        isSwing;
  bool        switchContact;
  bool        _pad[2];
  JointAngles angles;
};

extern LegState legs[6];

void pca9685Init(uint8_t addr);
void pca9685SetPWM(uint8_t addr, uint8_t ch, uint16_t value, const RobotSettings* s);
uint16_t angleToPWM(float angleDeg, const RobotSettings* s);
void writeAngles(uint8_t legIdx, const RobotSettings* s);

#endif
