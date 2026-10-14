#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "config.h"
#include "pca_driver.h"

struct LocomotionTelemetry {
  float weightLeftGrams;
  float weightRightGrams;
  float groundForceNewtons;
  String contactState;
  String activeGait;
  bool isRunning;
};

// Initializes the kinematics engine
void initKinematics();

// Called periodically (strictly at 50Hz / 20ms) by FreeRTOS locomotion task
void updateLocomotion(float dt);

// Locomotion commands
void startLocomotion(const String& gait);
void stopLocomotion();
void executeSingleStep();
void runJointDiagnostics();

// Real-time parameter tuning
void setGaitParam(const String& param, float val);

// 2D Analytical Inverse Kinematics (from reference code)
struct IKResult {
  float hipDeg;
  float kneeDeg;
  float ankleDeg;
};

IKResult solveIK(float x, float z);
void pos(float x, float z, char leg);
void updateServoPos(float dHip, float dKnee, float dAnkle, char leg);
void takeStep(float stepLength, int stepVelocity);
void initialize();

// Telemetry accessor
LocomotionTelemetry getLocomotionTelemetry();

#endif // KINEMATICS_H
