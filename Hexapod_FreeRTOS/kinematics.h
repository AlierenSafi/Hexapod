#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "robot_config.h"
#include "servo_drivers.h"

// Phase offset lookup tables
extern const float TRIPOD_PHASE[6];
extern const float RIPPLE_PHASE[6];
extern const float WAVE_PHASE[6];

extern GaitType currentGait;

float getPhaseOffset(uint8_t legIdx, GaitType gait);
Vec3 computeStanceFootPos(uint8_t legIdx, float stanceT, float velX, float velY, float velYaw, const RobotSettings* s);
void sitDown();
bool solveIK(uint8_t legIdx, const Vec3& target, const RobotSettings* s);
Vec3 cycloidTrajectory(float t, const Vec3& start, const Vec3& end, float height);

#endif
