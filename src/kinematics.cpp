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
  if (leg == 'l') {
    float aHip  = constrain(getJoint(3)->homeAngle - dHip, 76.0f, 110.0f);
    float aKnee = constrain(getJoint(4)->homeAngle - constrain(dKnee, 0.0f, 18.0f), 70.0f, 98.0f);
    float aFoot = constrain(getJoint(5)->homeAngle - dAnkle, 80.0f, 112.0f);
    setServoAngle(4,  aHip);
    setServoAngle(8,  aKnee);
    setServoAngle(15, aFoot);
  } else if (leg == 'r') {
    float aHip  = constrain(getJoint(0)->homeAngle + dHip, 78.0f, 112.0f);
    float aKnee = constrain(getJoint(1)->homeAngle + constrain(dKnee, 0.0f, 18.0f), 8.0f, 28.0f);
    float aFoot;
    if (dKnee > 1.0f) {
      // Swing phase: flex forward so heel stays pointing down and forward away from knee
      aFoot = constrain(getJoint(2)->homeAngle + max(0.0f, dAnkle + 6.0f), 94.0f, 108.0f);
    } else {
      // Stance phase: track hip angle to remain flat on the ground
      aFoot = constrain(getJoint(2)->homeAngle + dAnkle, 82.0f, 108.0f);
    }
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
  int delayMs = (stepVelocity > 0) ? stepVelocity : 20;
  float sClear = min(0.95f, stepClearance);

  // Phase 1: Right Leg stance (x: +L -> -L), Left Leg swing (x: -L -> +L, lifted)
  for (float i = sLength; i >= -sLength; i -= 0.12f) {
    pos(i, stepHeight, 'r');
    pos(-i, stepHeight - sClear, 'l');
    delay(delayMs);
  }

  // Phase 2: Right Leg swing (x: -L -> +L, lifted), Left Leg stance (x: +L -> -L)
  for (float i = sLength; i >= -sLength; i -= 0.12f) {
    pos(-i, stepHeight - sClear, 'r');
    pos(i, stepHeight, 'l');
    delay(delayMs);
  }

  // Smooth settle back to neutral stand (x: 0, z: stepHeight)
  for (float i = sLength; i >= 0.0f; i -= 0.12f) {
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

  float sLength = stepLength * currentGaitScale;
  float sClear  = min(0.95f, stepClearance) * currentGaitScale;
  float sHeight = stepHeight;

  float xr = 0.0f;
  float xl = 0.0f;
  float zr = sHeight;
  float zl = sHeight;

  if (activeGaitName == "backward" || activeGaitName == "back") {
    // Reverse Stride Walk
    xr = -cosP * sLength;
    xl =  cosP * sLength;
    if (sinP >= 0.0f) {
      zr = sHeight;
      zl = sHeight - sinP * sClear;
    } else {
      zr = sHeight - (-sinP) * sClear;
      zl = sHeight;
    }
  } else if (activeGaitName == "march") {
    // In-Place Dynamic Stepping (high knee lifts, zero forward drift)
    xr = 0.0f;
    xl = 0.0f;
    if (sinP >= 0.0f) {
      zr = sHeight;
      zl = sHeight - sinP * sClear * 1.2f;
    } else {
      zr = sHeight - (-sinP) * sClear * 1.2f;
      zl = sHeight;
    }
  } else if (activeGaitName == "turn_left" || activeGaitName == "left") {
    // Differential Turn Left
    xr =  cosP * sLength;
    xl =  cosP * (sLength * 0.4f);
    if (sinP >= 0.0f) {
      zr = sHeight;
      zl = sHeight - sinP * sClear;
    } else {
      zr = sHeight - (-sinP) * sClear;
      zl = sHeight;
    }
  } else if (activeGaitName == "turn_right" || activeGaitName == "right") {
    // Differential Turn Right
    xr = -cosP * (sLength * 0.4f);
    xl = -cosP * sLength;
    if (sinP >= 0.0f) {
      zr = sHeight;
      zl = sHeight - sinP * sClear;
    } else {
      zr = sHeight - (-sinP) * sClear;
      zl = sHeight;
    }
  } else if (activeGaitName == "crouch" || activeGaitName == "squat") {
    // Symmetrical Squat / Crouch & Rise
    xr = 0.0f;
    xl = 0.0f;
    float squat = (0.5f - 0.5f * cosP) * 1.2f * currentGaitScale;
    zr = sHeight - squat;
    zl = sHeight - squat;
  } else {
    // Default Dynamic Forward Walk ("walk", "forward", etc.)
    xr =  cosP * sLength;
    xl = -cosP * sLength;
    if (sinP >= 0.0f) {
      zr = sHeight;
      zl = sHeight - sinP * sClear;
    } else {
      zr = sHeight - (-sinP) * sClear;
      zl = sHeight;
    }
  }

  // Solve Reference Inverse Kinematics for both legs
  IKResult ikR = solveIK(xr, zr);
  IKResult ikL = solveIK(xl, zl);
  IKResult nom = solveIK(0.0f, sHeight);

  float dHipR   = ikR.hipDeg - nom.hipDeg;
  float dKneeR  = ikR.kneeDeg - nom.kneeDeg;
  float dAnkleR = ikR.ankleDeg - nom.ankleDeg;

  float dHipL   = ikL.hipDeg - nom.hipDeg;
  float dKneeL  = ikL.kneeDeg - nom.kneeDeg;
  float dAnkleL = ikL.ankleDeg - nom.ankleDeg;

  // Target Joint Setpoints (ALL 6 JOINTS KINEMATICALLY COUPLED):
  // -------------------------------------------------------------
  // Right Leg:
  // CH 0 Hip:   Follows dHipR directly (78° to 112°)
  // CH 1 Knee:  Holds 8° in stance; full swing lift up to 26° in air (18° lift)
  // CH 12 Foot: Blended continuously between stance and swing (ZERO step jumps!)
  float targetHipR  = constrain(getJoint(0)->homeAngle + dHipR, 78.0f, 112.0f);
  float kneeLiftR   = constrain(dKneeR, 0.0f, 18.0f);
  float targetKneeR = getJoint(1)->homeAngle + kneeLiftR;

  float liftR = max(0.0f, sHeight - zr);
  float uR = (sClear > 0.01f) ? constrain(liftR / sClear, 0.0f, 1.0f) : 0.0f;
  float stanceFootR = getJoint(2)->homeAngle + dAnkleR;
  float swingFootR  = getJoint(2)->homeAngle + max(0.0f, dAnkleR + 6.0f);
  float targetFootR = constrain((1.0f - uR) * stanceFootR + uR * swingFootR, 82.0f, 108.0f);

  // Left Leg:
  // CH 4 Hip:   Follows dHipL directly (76° to 110°)
  // CH 8 Knee:  Holds 94° in stance; lifts up in swing (down to 72°)
  // CH 15 Foot: Blended continuously between stance and swing
  float targetHipL  = constrain(getJoint(3)->homeAngle - dHipL, 76.0f, 110.0f);
  float kneeLiftL   = constrain(dKneeL, 0.0f, 18.0f);
  float targetKneeL = getJoint(4)->homeAngle - kneeLiftL;

  float liftL = max(0.0f, sHeight - zl);
  float uL = (sClear > 0.01f) ? constrain(liftL / sClear, 0.0f, 1.0f) : 0.0f;
  float stanceFootL = getJoint(5)->homeAngle - dAnkleL;
  float swingFootL  = getJoint(5)->homeAngle - dAnkleL + 4.0f;
  float targetFootL = constrain((1.0f - uL) * stanceFootL + uL * swingFootL, 80.0f, 112.0f);

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

  // Right Hip (CH 0): 94° -> 105° -> 94° (Forward swing)
  setServoAngle(0, 105.0f); delay(800);
  setServoAngle(0, 94.0f);   delay(400);

  // Right Knee (CH 1): 8° -> 20.0° -> 8° (Full swing ground clearance)
  setServoAngle(1, 20.0f); delay(800);
  setServoAngle(1, 8.0f);  delay(400);

  // Right Foot (CH 12): 94° -> 100.0° -> 94° (Active foot leveling & clearance flex)
  setServoAngle(12, 100.0f); delay(800);
  setServoAngle(12, 94.0f);  delay(400);

  // Left Hip (CH 4): 94° -> 83° -> 94° (Forward swing)
  setServoAngle(4, 83.0f); delay(800);
  setServoAngle(4, 94.0f); delay(400);

  // Left Knee (CH 8): 94° -> 82.0° -> 94° (Full swing ground clearance)
  setServoAngle(8, 82.0f); delay(800);
  setServoAngle(8, 94.0f); delay(400);

  // Left Foot (CH 15): 94° -> 100.0° -> 94° (Active foot leveling & clearance flex)
  setServoAngle(15, 100.0f); delay(800);
  setServoAngle(15, 94.0f);  delay(400);

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
