#include "pca_driver.h"
#include <Preferences.h>

static Adafruit_PWMServoDriver pwm(PCA9685_I2C_ADDR);
static Preferences prefs;
static bool pcaConnected = false;
static JointState joints[NUM_SERVOS];

bool initPCA() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_FREQUENCY_HZ);
  Wire.setTimeOut(50);

  Wire.beginTransmission(PCA9685_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    pwm.begin();
    pwm.setPWMFreq(PCA9685_PWM_FREQ_HZ);
    delay(20);
    pcaConnected = true;

    for (int i = 0; i < NUM_SERVOS; i++) {
      joints[i].lastPulseLen = -1;
    }
    Serial.printf("[PCA9685] Detected and initialized at 0x%02X (%dHz PWM, 400kHz I2C)\n",
                  PCA9685_I2C_ADDR, PCA9685_PWM_FREQ_HZ);
    return true;
  }

  pcaConnected = false;
  Serial.printf("[PCA9685] NOT found at 0x%02X on SDA=%d, SCL=%d. Will auto-retry.\n",
                PCA9685_I2C_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
  return false;
}

bool isPCAFound() {
  return pcaConnected;
}

bool checkPCAConnection() {
  if (pcaConnected) return true;
  return initPCA();
}

int getJointIndex(int channel) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (joints[i].channel == channel) return i;
  }
  return -1;
}

JointState* getJoint(int index) {
  if (index >= 0 && index < NUM_SERVOS) {
    return &joints[index];
  }
  return nullptr;
}

void setServoAngle(int channel, float angle) {
  if (!pcaConnected) {
    if (!initPCA()) return;
  }

  int idx = getJointIndex(channel);
  if (idx != -1) {
    // Physical bracket safety clamp
    angle = constrain(angle, joints[idx].minLimit, joints[idx].maxLimit);
    joints[idx].currentAngle = angle;
  } else {
    angle = constrain(angle, 0.0f, 180.0f);
  }

  // Convert degrees to microseconds (544us to 2400us)
  float us = SERVOMIN_US + (angle / 180.0f) * (SERVOMAX_US - SERVOMIN_US);
  // Convert microseconds to 12-bit PCA9685 tick count (20ms cycle @ 50Hz)
  int pulselen = (int)round((us * 4096.0f) / 20000.0f);

  if (idx != -1) {
    if (joints[idx].lastPulseLen != pulselen) {
      joints[idx].lastPulseLen = pulselen;
      pwm.setPWM(channel, 0, pulselen);
    }
  } else {
    pwm.setPWM(channel, 0, pulselen);
  }
}

void parkNeutralStand() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    joints[i].targetAngle = joints[i].homeAngle;
    setServoAngle(joints[i].channel, joints[i].homeAngle);
  }
}

void loadNVSJointOffsets() {
  // Initialize joint array from config defaults first
  for (int i = 0; i < NUM_SERVOS; i++) {
    joints[i].channel      = DEFAULT_JOINTS[i].channel;
    joints[i].name         = DEFAULT_JOINTS[i].name;
    joints[i].homeAngle    = DEFAULT_JOINTS[i].homeAngle;
    joints[i].currentAngle = DEFAULT_JOINTS[i].homeAngle;
    joints[i].targetAngle  = DEFAULT_JOINTS[i].homeAngle;
    joints[i].minLimit     = DEFAULT_JOINTS[i].minLimit;
    joints[i].maxLimit     = DEFAULT_JOINTS[i].maxLimit;
    joints[i].lastPulseLen = -1;
  }

  prefs.begin("biped_chmap", true); // read-only
  // Check if saved offsets exist
  if (prefs.isKey("ch0")) {
    joints[0].homeAngle = prefs.getFloat("ch0",  DEFAULT_JOINTS[0].homeAngle);
    joints[1].homeAngle = prefs.getFloat("ch1",  DEFAULT_JOINTS[1].homeAngle);
    joints[2].homeAngle = prefs.getFloat("ch12", DEFAULT_JOINTS[2].homeAngle);
    joints[3].homeAngle = prefs.getFloat("ch4",  DEFAULT_JOINTS[3].homeAngle);
    joints[4].homeAngle = prefs.getFloat("ch8",  DEFAULT_JOINTS[4].homeAngle);
    joints[5].homeAngle = prefs.getFloat("ch15", DEFAULT_JOINTS[5].homeAngle);

    for (int i = 0; i < NUM_SERVOS; i++) {
      joints[i].currentAngle = joints[i].homeAngle;
      joints[i].targetAngle  = joints[i].homeAngle;
    }
    Serial.println("[NVS] Loaded saved neutral offsets from flash.");
  } else {
    Serial.println("[NVS] Using factory calibrated neutral baselines.");
  }
  prefs.end();
}

void saveNVSJointOffsets() {
  prefs.begin("biped_chmap", false); // read-write
  prefs.putFloat("ch0",  joints[0].currentAngle);
  prefs.putFloat("ch1",  joints[1].currentAngle);
  prefs.putFloat("ch12", joints[2].currentAngle);
  prefs.putFloat("ch4",  joints[3].currentAngle);
  prefs.putFloat("ch8",  joints[4].currentAngle);
  prefs.putFloat("ch15", joints[5].currentAngle);

  // Update in-memory home angles
  for (int i = 0; i < NUM_SERVOS; i++) {
    joints[i].homeAngle = joints[i].currentAngle;
  }
  prefs.end();
  Serial.println("[NVS] Saved current joint angles as permanent home offsets.");
}

void resetNVSJointOffsets() {
  prefs.begin("biped_chmap", false);
  prefs.clear();
  prefs.end();

  for (int i = 0; i < NUM_SERVOS; i++) {
    joints[i].homeAngle    = DEFAULT_JOINTS[i].homeAngle;
    joints[i].currentAngle = DEFAULT_JOINTS[i].homeAngle;
    joints[i].targetAngle  = DEFAULT_JOINTS[i].homeAngle;
  }
  parkNeutralStand();
  Serial.println("[NVS] Restored factory default offsets.");
}
