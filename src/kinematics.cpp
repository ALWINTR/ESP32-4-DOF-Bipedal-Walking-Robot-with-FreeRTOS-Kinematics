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

// Low-pass EMA filter state for silky-smooth servo tracking
static float filteredHipR  = 94.0f;
static float filteredKneeR =  8.0f;
static float filteredHipL  = 94.0f;
static float filteredKneeL = 94.0f;

static void resetFilters() {
  JointState* j0 = getJoint(0); if (j0) filteredHipR  = j0->homeAngle;
  JointState* j1 = getJoint(1); if (j1) filteredKneeR = j1->homeAngle;
  JointState* j3 = getJoint(3); if (j3) filteredHipL  = j3->homeAngle;
  JointState* j4 = getJoint(4); if (j4) filteredKneeL = j4->homeAngle;
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

  float sinP = sin(phase);

  // 1. Pure Harmonic Sine Stride (Starts from EXACT 0.0° neutral with zero initial kick)
  float hipSwingAmp = 5.5f * (stepLength / 1.6f) * currentGaitScale;

  // Symmetrical forward swing & stance propulsion:
  // Phase 0..PI: Right leg swings forward, Left leg drives stance backward
  // Phase PI..2PI: Left leg swings forward, Right leg drives stance backward
  float strideR =  sinP * hipSwingAmp;
  float strideL = -sinP * hipSwingAmp;

  // 2. Parabolic Knee Lift (Smooth liftoff and zero-velocity soft touchdown)
  // 4.0° lift: clean clearance, zero knee-foot contact
  float maxKneeLift = 4.0f * (stepClearance / 1.0f) * currentGaitScale;
  float kneeLiftR = 0.0f;
  float kneeLiftL = 0.0f;

  if (sinP >= 0.0f) {
    // Phase 1: Right Leg swings in air, Left Leg stance on ground
    kneeLiftR = (sinP * sinP) * maxKneeLift;
    kneeLiftL = 0.0f;
  } else {
    // Phase 2: Left Leg swings in air, Right Leg stance on ground
    kneeLiftR = 0.0f;
    kneeLiftL = (sinP * sinP) * maxKneeLift;
  }

  // 3. Raw target angles
  float targetHipR  = getJoint(0)->homeAngle + strideR;
  float targetKneeR = getJoint(1)->homeAngle + kneeLiftR;
  float targetFootR = getJoint(2)->homeAngle; // Strictly flat 94° neutral

  float targetHipL  = getJoint(3)->homeAngle - strideL;
  float targetKneeL = getJoint(4)->homeAngle - kneeLiftL;
  float targetFootL = getJoint(5)->homeAngle; // Strictly flat 94° neutral

  // 4. Low-Pass Exponential Smoothing Filter (Absorbs high-frequency gear chatter and deadband hunting)
  const float alpha = 0.40f;
  filteredHipR  += alpha * (targetHipR  - filteredHipR);
  filteredKneeR += alpha * (targetKneeR - filteredKneeR);
  filteredHipL  += alpha * (targetHipL  - filteredHipL);
  filteredKneeL += alpha * (targetKneeL - filteredKneeL);

  // Send filtered setpoints to PCA9685 driver
  setServoAngle(0,  filteredHipR);
  setServoAngle(1,  filteredKneeR);
  setServoAngle(12, targetFootR);
  setServoAngle(4,  filteredHipL);
  setServoAngle(8,  filteredKneeL);
  setServoAngle(15, targetFootL);

  // 4. Real-Time Dynamic Ground Reaction Force Telemetry
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

  // Right Hip (CH 0): 94° -> 102° -> 94°
  setServoAngle(0, 102.0f); delay(800);
  setServoAngle(0, 94.0f);  delay(400);

  // Right Knee (CH 1): 8° -> 14.0° -> 8° (safe sweep clear of foot)
  setServoAngle(1, 14.0f); delay(800);
  setServoAngle(1, 8.0f);  delay(400);

  // Left Hip (CH 4): 94° -> 86° -> 94°
  setServoAngle(4, 86.0f); delay(800);
  setServoAngle(4, 94.0f); delay(400);

  // Left Knee (CH 8): 94° -> 88.0° -> 94° (safe sweep clear of foot)
  setServoAngle(8, 88.0f); delay(800);
  setServoAngle(8, 94.0f); delay(400);

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
