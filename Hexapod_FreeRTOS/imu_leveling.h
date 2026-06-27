#ifndef IMU_LEVELING_H
#define IMU_LEVELING_H

#include <Arduino.h>
#include "robot_config.h"

#define MPU6050_ADDR     0x68
#define MPU_PWR_MGMT     0x6B
#define MPU_ACCEL_XOUT   0x3B
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG_REG   0x1A

struct IMUData {
  float pitch, roll;     // Complementary Filter outputs (°)
  float ax, ay, az;      // Acceleration (g)
  float gx, gy, gz;      // Gyro rate (°/s)
};

extern IMUData imuData;
extern SemaphoreHandle_t imuMutex;
extern bool mpuAvailable;

// PID States for leveling
extern float pidPitchInt, pidPitchPrev;
extern float pidRollInt, pidRollPrev;

bool mpuInit();
void updateCompFilter(float dt, const RobotSettings* s);
void computeLevelingOffset(float dt, const RobotSettings* s, float out[6]);

#endif
