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

void initKinematics() {
  currentGaitScale = 0.0f;
  walkTime         = 0.0f;
  gaitActive       = false;
  activeGaitName   = "stand";
  weightLeft       = 250.0f;
  weightRight      = 250.0f;
  groundForceN     = 4.905f;
  contactState     = "DUAL STANCE";
}

void startLocomotion(const String& gait) {
  gaitActive = true;
  activeGaitName = gait;
  currentGaitScale = 0.0f; // Smooth ease-in from standing posture
  walkTime = 0.0f;
}

void stopLocomotion() {
  gaitActive = false;
  activeGaitName = "stand";
  currentGaitScale = 0.0f;
  parkNeutralStand();
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

// Zero-Impact Double-Support Swing Lift (Soft parabolic liftoff & touchdown)
static inline float calculateSwingClearance(float stepPhase, float sClear) {
  const float ds = 0.15f * (float)M_PI; // 15% double support buffer on each transition (30% total)
  if (stepPhase < ds || stepPhase > ((float)M_PI - ds)) {
    return 0.0f; // Firm double-support ground contact
  }
  float u = (stepPhase - ds) / ((float)M_PI - 2.0f * ds);
  float s = sin(u * (float)M_PI);
  return sClear * (s * s); // Soft zero-impact bell curve
}

void updateServoPos(float dHip, float dKnee, float dAnkle, char leg) {
  // Safe Knee Lift: capped to 3.5 deg so knee bracket never hits foot bracket
  float safeLift = constrain(dKnee * 0.40f, 0.0f, 3.5f);

  if (leg == 'l') {
    // Left Leg:
    // Swing (safeLift > 0.2 deg): foot strictly locks to neutral 94 deg so heel never hits knee!
    // Stance (safeLift <= 0.2 deg): foot flexes to match ground reaction so foot sole stays completely flat
    float safeFlex = (safeLift > 0.2f) ? 0.0f : constrain(dAnkle * 0.80f, -3.0f, 3.0f);

    setServoAngle(4,  constrain(getJoint(3)->homeAngle - dHip,     85.0f, 103.0f));
    setServoAngle(8,  constrain(getJoint(4)->homeAngle - safeLift, 89.5f,  94.0f));
    setServoAngle(15, constrain(getJoint(5)->homeAngle - safeFlex, 91.0f,  97.0f));
  } else if (leg == 'r') {
    // Right Leg:
    float safeFlex = (safeLift > 0.2f) ? 0.0f : constrain(dAnkle * 0.80f, -3.0f, 3.0f);

    setServoAngle(0,  constrain(getJoint(0)->homeAngle + dHip,     85.0f, 103.0f));
    setServoAngle(1,  constrain(getJoint(1)->homeAngle + safeLift,  8.0f,  12.5f));
    setServoAngle(12, constrain(getJoint(2)->homeAngle + safeFlex, 91.0f,  97.0f));
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
  float sClear = min(0.35f, stepClearance);

  // Phase 1: Right Leg stance (x: +L -> -L), Left Leg swing (x: -L -> +L, soft parabolic arch)
  for (float i = sLength; i >= -sLength; i -= 0.10f) {
    float u = (sLength - i) / (2.0f * sLength); // 0.0 to 1.0
    float lift = sClear * sin(u * (float)M_PI);
    pos(i, stepHeight, 'r');
    pos(-i, stepHeight - lift, 'l');
    delay(delayMs);
  }

  // Phase 2: Right Leg swing (x: -L -> +L, soft parabolic arch), Left Leg stance (x: +L -> -L)
  for (float i = sLength; i >= -sLength; i -= 0.10f) {
    float u = (sLength - i) / (2.0f * sLength); // 0.0 to 1.0
    float lift = sClear * sin(u * (float)M_PI);
    pos(-i, stepHeight - lift, 'r');
    pos(i, stepHeight, 'l');
    delay(delayMs);
  }

  // Smooth settle back to neutral stand (x: 0, z: stepHeight)
  for (float i = sLength; i >= 0.0f; i -= 0.10f) {
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
  float sClear  = min(0.35f, stepClearance) * currentGaitScale;
  float sHeight = stepHeight;

  float xr = 0.0f;
  float xl = 0.0f;
  float zr = sHeight;
  float zl = sHeight;

  // Zero-impact swing clearance profiles with double-support weight transfer
  float liftL = (phase < (float)M_PI) ? calculateSwingClearance(phase, sClear) : 0.0f;
  float liftR = (phase >= (float)M_PI) ? calculateSwingClearance(phase - (float)M_PI, sClear) : 0.0f;

  if (activeGaitName == "backward" || activeGaitName == "back") {
    // Reverse Stride Walk
    xr = -cosP * sLength;
    xl =  cosP * sLength;
    zr = sHeight - liftR;
    zl = sHeight - liftL;
  } else if (activeGaitName == "march") {
    // In-Place Dynamic Stepping (high knee lifts, zero forward drift)
    xr = 0.0f;
    xl = 0.0f;
    zr = sHeight - liftR * 1.2f;
    zl = sHeight - liftL * 1.2f;
  } else if (activeGaitName == "turn_left" || activeGaitName == "left") {
    // Differential Turn Left
    xr =  cosP * sLength;
    xl =  cosP * (sLength * 0.35f);
    zr = sHeight - liftR;
    zl = sHeight - liftL;
  } else if (activeGaitName == "turn_right" || activeGaitName == "right") {
    // Differential Turn Right
    xr = -cosP * (sLength * 0.35f);
    xl = -cosP * sLength;
    zr = sHeight - liftR;
    zl = sHeight - liftL;
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
    zr = sHeight - liftR;
    zl = sHeight - liftL;
  }

  // Direct Inverse Kinematics execution via unified pos(x, z, leg)
  pos(xr, zr, 'r');
  pos(xl, zl, 'l');

  // Real-Time Dynamic Ground Reaction Force Telemetry
  if (currentGaitScale > 0.10f) {
    if (liftL > 0.05f) {
      weightRight = min(500.0f, ROBOT_TOTAL_WEIGHT_GRAMS * (0.85f + 0.15f * (liftL / sClear)));
      weightLeft  = max(0.0f, ROBOT_TOTAL_WEIGHT_GRAMS - weightRight);
      contactState = "RIGHT SINGLE SUPPORT";
    } else if (liftR > 0.05f) {
      weightLeft  = min(500.0f, ROBOT_TOTAL_WEIGHT_GRAMS * (0.85f + 0.15f * (liftR / sClear)));
      weightRight = max(0.0f, ROBOT_TOTAL_WEIGHT_GRAMS - weightLeft);
      contactState = "LEFT SINGLE SUPPORT";
    } else {
      weightLeft   = 250.0f;
      weightRight  = 250.0f;
      contactState = "DOUBLE SUPPORT";
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

  // Right Hip (CH 0): 94° -> 98° -> 94° (Forward swing)
  setServoAngle(0, 98.0f); delay(800);
  setServoAngle(0, 94.0f); delay(400);

  // Right Knee (CH 1): 8° -> 10.5° -> 8° (Safe clearance gap)
  setServoAngle(1, 10.5f); delay(800);
  setServoAngle(1, 8.0f);  delay(400);

  // Right Foot (CH 12): 94° -> 95.5° -> 94° (Active foot leveling)
  setServoAngle(12, 95.5f); delay(800);
  setServoAngle(12, 94.0f); delay(400);

  // Left Hip (CH 4): 94° -> 90° -> 94° (Forward swing)
  setServoAngle(4, 90.0f); delay(800);
  setServoAngle(4, 94.0f); delay(400);

  // Left Knee (CH 8): 94° -> 91.5° -> 94° (Safe clearance gap)
  setServoAngle(8, 91.5f); delay(800);
  setServoAngle(8, 94.0f); delay(400);

  // Left Foot (CH 15): 94° -> 95.5° -> 94° (Active foot leveling)
  setServoAngle(15, 95.5f); delay(800);
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
