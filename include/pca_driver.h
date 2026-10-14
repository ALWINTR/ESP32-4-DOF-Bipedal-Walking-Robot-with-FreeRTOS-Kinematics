#ifndef PCA_DRIVER_H
#define PCA_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

struct JointState {
  int channel;
  const char* name;
  float currentAngle;
  float targetAngle;
  float homeAngle;
  float minLimit;
  float maxLimit;
  int lastPulseLen;
};

// Initializes I2C bus @ 400kHz and PCA9685 driver
bool initPCA();

// Returns true if PCA9685 is responsive on the I2C bus
bool isPCAFound();

// Attempts to reconnect if disconnected
bool checkPCAConnection();

// Set angle with automatic physical bracket clamping & pulse caching
void setServoAngle(int channel, float angle);

// Returns joint state by index (0 to NUM_SERVOS-1)
JointState* getJoint(int index);

// Returns joint index for a PCA9685 channel, or -1 if not found
int getJointIndex(int channel);

// Parks all 6 servos strictly in calibrated standing neutral posture
void parkNeutralStand();

// NVS Persistent Calibration Storage
void loadNVSJointOffsets();
void saveNVSJointOffsets();
void resetNVSJointOffsets();

#endif // PCA_DRIVER_H
