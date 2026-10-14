#include "kinematics.h"
#include <math.h>

static float walkFreq         = DEFAULT_WALK_FREQ_HZ;
static float stepLength       = DEFAULT_STEP_LENGTH_CM;
static float stepClearance    = DEFAULT_STEP_CLEARANCE_CM;
static float stepHeight       = DEFAULT_STEP_HEIGHT_CM;

static float currentGaitScale = 0.0f;
static float walkTime         = 0.0f;
static bool  gaitActive       = false;
static String activeGaitName  = "stand";

// Dynamic Ground Reaction Force & Contact State
static float weightLeft       = 250.0f;
static float weightRight      = 250.0f;
static float groundForceN     = 4.905f;
static String contactState    = "DUAL STANCE";

// Low-pass EMA filter state for silky-smooth servo tracking across all 6 joints
static float filteredHipR  = 94.0f;
static float filteredKneeR =  8.0f;
static float filteredFootR = 94.0f;
static float filteredHipL  = 94.0f;
static float filteredKneeL = 94.0f;
static float filteredFootL = 94.0f;

static void resetFilters() {
  JointState* j0 = getJoint(0); if (j0) filteredHipR  = j0->homeAngle;
  JointState* j1 = getJoint(1); if (j1) filteredKneeR = j1->homeAngle;
  JointState* j2 = getJoint(2); if (j2) filteredFootR = j2->homeAngle;
  JointState* j3 = getJoint(3); if (j3) filteredHipL  = j3->homeAngle;
  JointState* j4 = getJoint(4); if (j4) filteredKneeL = j4->homeAngle;
  JointState* j5 = getJoint(5); if (j5) filteredFootL = j5->homeAngle;
}

void initKinematics() {
  currentGaitScale = 0.0f;
  walkTime         = 0.0f;
  gaitActive       = false;
  activeGaitName   = "stand";
  weightLeft       = 250.0f;
  weightRight      = 250.0f;
  groundForceN     = 4.905f;
  contactState     = "DUAL STANCE";
  resetFilters();
}

void startLocomotion(const String& gait) {
  gaitActive = true;
  activeGaitName = gait;
  currentGaitScale = 0.0f; // Smooth ease-in from standing posture
  walkTime = 0.0f;
  resetFilters();
}

void stopLocomotion() {
  gaitActive = false;
  activeGaitName = "stand";
  currentGaitScale = 0.0f;
  parkNeutralStand();
  resetFilters();
  weightLeft   = 250.0f;
  weightRight  = 250.0f;
  groundForceN = 4.905f;
  contactState = "DUAL STANCE";
}

// =============================================================================
// Analytical 2D Inverse Kinematics Engine (from Reference Code)
// Links: Thigh l1 = 5.0cm, Shin l2 = 5.7cm, Max Extended Height = 10.7cm
// =============================================================================
IKResult solveIK(float x, float z) {
  IKResult res;
  float hipRad2 = atan2(x, z);
  float hipDeg2 = hipRad2 * (180.0f / (float)M_PI);

  float z2 = sqrt(x * x + z * z);

  float val1 = constrain((sq(LINK_THIGH_L1_CM) + sq(z2) - sq(LINK_SHIN_L2_CM)) / (2.0f * LINK_THIGH_L1_CM * z2), -1.0f, 1.0f);
  float hipRad1 = acos(val1);
  float hipDeg1 = hipRad1 * (180.0f / (float)M_PI);

  float val2 = constrain((sq(LINK_THIGH_L1_CM) + sq(LINK_SHIN_L2_CM) - sq(z2)) / (2.0f * LINK_THIGH_L1_CM * LINK_SHIN_L2_CM), -1.0f, 1.0f);
  float kneeRad = (float)M_PI - acos(val2);
  float kneeDeg = kneeRad * (180.0f / (float)M_PI);

  float val3 = constrain((sq(LINK_SHIN_L2_CM) + sq(z2) - sq(LINK_THIGH_L1_CM)) / (2.0f * LINK_SHIN_L2_CM * z2), -1.0f, 1.0f);
  float ankleRad = ((float)M_PI / 2.0f) + hipRad2 - acos(val3);
  float ankleDeg = ankleRad * (180.0f / (float)M_PI);

  res.hipDeg   = hipDeg1 + hipDeg2;
  res.kneeDeg  = kneeDeg;
  res.ankleDeg = ankleDeg;
  return res;
}

void updateServoPos(float dHip, float dKnee, float dAnkle, char leg) {
  float safeLift = constrain(dKnee * 0.30f, 0.0f, 4.5f);
  float safeFlex = constrain(fabs(dAnkle) * 0.20f, 0.0f, 2.5f);

  if (leg == 'l') {
    float aHip  = constrain(getJoint(3)->homeAngle - dHip, 88.0f, 100.0f);
    float aKnee = getJoint(4)->homeAngle - safeLift;
    float aFoot = getJoint(5)->homeAngle - safeFlex;
    setServoAngle(4,  aHip);
    setServoAngle(8,  aKnee);
    setServoAngle(15, aFoot);
  } else if (leg == 'r') {
    float aHip  = constrain(getJoint(0)->homeAngle + dHip, 88.0f, 100.0f);
    float aKnee = getJoint(1)->homeAngle + safeLift;
    float aFoot = getJoint(2)->homeAngle + safeFlex;
    setServoAngle(0,  aHip);
    setServoAngle(1,  aKnee);
    setServoAngle(12, aFoot);
  }
}

void pos(float x, float z, char leg) {
  IKResult ik  = solveIK(x, z);
  IKResult nom = solveIK(0.0f, stepHeight);

  float dHip   = ik.hipDeg - nom.hipDeg;
  float dKnee  = ik.kneeDeg - nom.kneeDeg;
  float dAnkle = ik.ankleDeg - nom.ankleDeg;

  updateServoPos(dHip, dKnee, dAnkle, leg);
}

void takeStep(float sLength, int stepVelocity) {
  int delayMs = (stepVelocity > 0) ? stepVelocity : 25;
  float sClear = min(0.6f, stepClearance);

  // Phase 1: Right Leg stance (x: +L -> -L), Left Leg swing (x: -L -> +L)
  for (float i = sLength; i >= -sLength; i -= 0.15f) {
    pos(i, stepHeight, 'r');
    pos(-i, stepHeight - sClear, 'l');
    delay(delayMs);
  }

  // Phase 2: Right Leg swing (x: -L -> +L), Left Leg stance (x: +L -> -L)
  for (float i = sLength; i >= -sLength; i -= 0.15f) {
    pos(-i, stepHeight - sClear, 'r');
    pos(i, stepHeight, 'l');
    delay(delayMs);
  }

  // Smooth settle back to neutral stand (x: 0, z: stepHeight)
  for (float i = sLength; i >= 0.0f; i -= 0.15f) {
    pos(i, stepHeight, 'r');
    pos(-i, stepHeight, 'l');
    delay(delayMs);
  }
  parkNeutralStand();
}

void initialize() {
  for (float i = 10.7f; i >= stepHeight; i -= 0.05f) {
    pos(0.0f, i, 'l');
    pos(0.0f, i, 'r');
    delay(15);
  }
  parkNeutralStand();
}

void updateLocomotion(float dt) {
  // Smooth Anti-Jerk Envelope
  if (gaitActive) {
    if (currentGaitScale < 1.0f) {
      currentGaitScale = min(1.0f, currentGaitScale + 0.025f);
    }
    walkTime += dt;
  } else {
    if (currentGaitScale > 0.0f) {
      currentGaitScale = max(0.0f, currentGaitScale - 0.04f);
      walkTime += dt;
    } else {
      return; // Fully parked at neutral, zero unnecessary computation
    }
  }

  // Continuous Harmonic Phase (0 to 2*PI)
  float phase = fmod(2.0f * (float)M_PI * walkFreq * walkTime, 2.0f * (float)M_PI);
  if (phase < 0.0f) phase += 2.0f * (float)M_PI;

  float cosP = cos(phase);
  float sinP = sin(phase);

  // Cartesian Trajectory matching reference takeStep:
  // Phase 1 (0 to PI): Right Leg stance (+L -> -L), Left Leg swing (-L -> +L, lifted)
  // Phase 2 (PI to 2PI): Right Leg swing (-L -> +L, lifted), Left Leg stance (+L -> -L)
  float sLength = stepLength * currentGaitScale;
  float sClear  = min(0.6f, stepClearance) * currentGaitScale;
  float sHeight = stepHeight;

  float xr =  cosP * sLength;
  float xl = -cosP * sLength;
  float zr = sHeight;
  float zl = sHeight;

  // Active Knee Lift & Foot Leveling Flexion
  float kneeLiftR = 0.0f;
  float footFlexR = 0.0f;
  float kneeLiftL = 0.0f;
  float footFlexL = 0.0f;

  if (sinP >= 0.0f) {
    // Phase 1: Right Leg stance on ground, Left Leg swing in air
    zr = sHeight;
    zl = sHeight - sinP * sClear;

    kneeLiftR = 0.0f;
    footFlexR = 0.0f;

    // Left Knee lifts up 4.5°, Left Foot flexes 2.5° away from knee
    kneeLiftL = sinP * (4.5f * currentGaitScale);
    footFlexL = sinP * (2.5f * currentGaitScale);
  } else {
    // Phase 2: Right Leg swing in air, Left Leg stance on ground
    zr = sHeight - (-sinP) * sClear;
    zl = sHeight;

    // Right Knee lifts up 4.5°, Right Foot flexes 2.5° away from knee
    kneeLiftR = (-sinP) * (4.5f * currentGaitScale);
    footFlexR = (-sinP) * (2.5f * currentGaitScale);

    kneeLiftL = 0.0f;
    footFlexL = 0.0f;
  }

  // Solve Reference Inverse Kinematics for both hips
  IKResult ikR = solveIK(xr, zr);
  IKResult ikL = solveIK(xl, zl);
  IKResult nom = solveIK(0.0f, sHeight);

  float dHipR = ikR.hipDeg - nom.hipDeg;
  float dHipL = ikL.hipDeg - nom.hipDeg;

  // Target Joint Setpoints (ALL 6 JOINTS ACTIVELY DRIVEN):
  // Right Leg:
  //   CH 0  Hip:  Strides 88° to 100° (home: 94°)
  //   CH 1  Knee: Stance 8°, lifts to 12.5° in air (home: 8°)
  //   CH 12 Foot: Flat 94°, flexes to 96.5° in air away from knee (home: 94°)
  float targetHipR  = constrain(getJoint(0)->homeAngle + dHipR, 88.0f, 100.0f);
  float targetKneeR = getJoint(1)->homeAngle + kneeLiftR;
  float targetFootR = getJoint(2)->homeAngle + footFlexR;

  // Left Leg:
  //   CH 4  Hip:  Strides 100° to 88° (home: 94°)
  //   CH 8  Knee: Stance 94°, lifts to 89.5° in air (home: 94°)
  //   CH 15 Foot: Flat 94°, flexes to 91.5° in air away from knee (home: 94°)
  float targetHipL  = constrain(getJoint(3)->homeAngle - dHipL, 88.0f, 100.0f);
  float targetKneeL = getJoint(4)->homeAngle - kneeLiftL;
  float targetFootL = getJoint(5)->homeAngle - footFlexL;

  // Low-Pass Exponential Smoothing Filter across all 6 joints
  const float alpha = 0.35f;
  filteredHipR  += alpha * (targetHipR  - filteredHipR);
  filteredKneeR += alpha * (targetKneeR - filteredKneeR);
  filteredFootR += alpha * (targetFootR - filteredFootR);

  filteredHipL  += alpha * (targetHipL  - filteredHipL);
  filteredKneeL += alpha * (targetKneeL - filteredKneeL);
  filteredFootL += alpha * (targetFootL - filteredFootL);

  // Send filtered setpoints to PCA9685 driver
  setServoAngle(0,  filteredHipR);
  setServoAngle(1,  filteredKneeR);
  setServoAngle(12, filteredFootR);
  setServoAngle(4,  filteredHipL);
  setServoAngle(8,  filteredKneeL);
  setServoAngle(15, filteredFootL);

  // Real-Time Dynamic Ground Reaction Force Telemetry
  if (currentGaitScale > 0.10f) {
    if (sinP > 0.10f) {
      weightRight = max(0.0f, 250.0f * (1.0f - sinP));
      weightLeft  = min(600.0f, (ROBOT_TOTAL_WEIGHT_GRAMS - weightRight) * (1.0f + 0.12f * sinP));
      contactState = (sinP > 0.35f) ? "LEFT SINGLE SUPPORT" : "WEIGHT TRANSFER";
    } else if (sinP < -0.10f) {
      weightLeft   = max(0.0f, 250.0f * (1.0f - (-sinP)));
      weightRight  = min(600.0f, (ROBOT_TOTAL_WEIGHT_GRAMS - weightLeft) * (1.0f + 0.12f * (-sinP)));
      contactState = (-sinP > 0.35f) ? "RIGHT SINGLE SUPPORT" : "WEIGHT TRANSFER";
    } else {
      weightLeft   = 250.0f;
      weightRight  = 250.0f;
      contactState = "DUAL STANCE";
    }
  } else {
    weightLeft   = 250.0f;
    weightRight  = 250.0f;
    contactState = "DUAL STANCE";
  }
  groundForceN = (weightLeft + weightRight) * 0.00981f;
}

void executeSingleStep() {
  parkNeutralStand();
  delay(100);
  gaitActive = true;
  activeGaitName = "step";
  float period = 1.0f / walkFreq;
  unsigned long t0 = millis();
  unsigned long durationMs = (unsigned long)(period * 1000.0f);

  while (millis() - t0 < durationMs) {
    float u = (float)(millis() - t0) / (float)durationMs;
    currentGaitScale = sin(u * (float)M_PI); // Smooth half-sine window: 0 -> 1 -> 0
    walkTime = u * period;
    updateLocomotion(0.020f);
    delay(20);
  }
  stopLocomotion();
}

void runJointDiagnostics() {
  parkNeutralStand();
  delay(300);

  // Right Hip (CH 0): 94° -> 100° -> 94° (Forward swing)
  setServoAngle(0, 100.0f); delay(800);
  setServoAngle(0, 94.0f);  delay(400);

  // Right Knee (CH 1): 8° -> 12.5° -> 8° (Active knee lift)
  setServoAngle(1, 12.5f); delay(800);
  setServoAngle(1, 8.0f);  delay(400);

  // Right Foot (CH 12): 94° -> 96.5° -> 94° (Active foot leveling & clearance flex)
  setServoAngle(12, 96.5f); delay(800);
  setServoAngle(12, 94.0f); delay(400);

  // Left Hip (CH 4): 94° -> 88° -> 94° (Forward swing)
  setServoAngle(4, 88.0f); delay(800);
  setServoAngle(4, 94.0f); delay(400);

  // Left Knee (CH 8): 94° -> 89.5° -> 94° (Active knee lift)
  setServoAngle(8, 89.5f); delay(800);
  setServoAngle(8, 94.0f); delay(400);

  // Left Foot (CH 15): 94° -> 91.5° -> 94° (Active foot leveling & clearance flex)
  setServoAngle(15, 91.5f); delay(800);
  setServoAngle(15, 94.0f); delay(400);

  parkNeutralStand();
}

void setGaitParam(const String& param, float val) {
  if (param == "freq" || param == "speed") {
    walkFreq = constrain(val, 0.3f, 2.0f);
  } else if (param == "stride" || param == "length") {
    stepLength = constrain(val, 0.5f, 3.5f);
  } else if (param == "knee" || param == "clearance" || param == "lift") {
    stepClearance = constrain(val, 0.2f, 2.0f);
  } else if (param == "height" || param == "stance") {
    stepHeight = constrain(val, 8.5f, 10.7f);
  }
}

LocomotionTelemetry getLocomotionTelemetry() {
  LocomotionTelemetry t;
  t.weightLeftGrams    = weightLeft;
  t.weightRightGrams   = weightRight;
  t.groundForceNewtons = groundForceN;
  t.contactState       = contactState;
  t.activeGait         = activeGaitName;
  t.isRunning          = gaitActive;
  return t;
}
