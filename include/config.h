#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// 1. WiFi & Network Configuration
// Priority #1: Active PC Network (MATHA ELECTRONICS_ Dwan -> 192.168.0.x)
// Priority #2: Backup Network (MATHA BSNL FTTH -> 192.168.1.x)
// Dedicated SoftAP: Always accessible at 192.168.4.1
// =============================================================================
struct WiFiCredential {
  const char* ssid;
  const char* password;
};

const WiFiCredential KNOWN_NETWORKS[] = {
  {"MATHA ELECTRONICS_ Dwan", "qwerty09"}, // PC Subnet 192.168.0.x
  {"MATHA BSNL FTTH",          "qwerty09"}  // Backup Subnet 192.168.1.x
};
const size_t NUM_NETWORKS = sizeof(KNOWN_NETWORKS) / sizeof(KNOWN_NETWORKS[0]);

const char* const AP_SSID = "ESP32-Biped-Robot";
const char* const AP_PASS = "12345678";
const char* const MDNS_HOSTNAME = "biped"; // Accessible via http://biped.local

// =============================================================================
// 2. Hardware Pinout & I2C Bus Configuration
// =============================================================================
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22
#define I2C_FREQUENCY_HZ    400000 // 400kHz Fast I2C Mode for Zero-Lag PCA9685
#define PCA9685_I2C_ADDR    0x40
#define PCA9685_PWM_FREQ_HZ 50

#define SERVOMIN_US         544
#define SERVOMAX_US         2400

// =============================================================================
// 3. 6-DOF Servo Configuration & Calibrated Neutral Baselines
// Exact Physical Wiring:
//   Right Leg: CH 0 (Hip 94°), CH 1 (Knee 8°),  CH 12 (Foot 94°)
//   Left Leg:  CH 4 (Hip 94°), CH 8 (Knee 94°), CH 15 (Foot 94°)
// =============================================================================
#define NUM_SERVOS 6

struct JointConfig {
  int channel;
  const char* name;
  float homeAngle;
  float minLimit;
  float maxLimit;
};

const JointConfig DEFAULT_JOINTS[NUM_SERVOS] = {
  {0,  "Right Hip",   94.0f, 78.0f, 112.0f}, // CH 0: 94° neutral, full kinematic stride 80°-110°
  {1,  "Right Knee",   8.0f,  8.0f,  28.0f}, // CH 1: 8° straight extension, full swing lift up to 26.0° in air
  {12, "Right Foot",  94.0f, 82.0f, 110.0f}, // CH 12: 94° flat ground, stance tracks hip, swing >= 94° (zero knee contact!)
  {4,  "Left Hip",    94.0f, 76.0f, 110.0f}, // CH 4: 94° neutral, full kinematic stride 78°-110°
  {8,  "Left Knee",   94.0f, 70.0f,  98.0f}, // CH 8: 94° straight extension, full swing lift to 72.0° in air
  {15, "Left Foot",   94.0f, 80.0f, 112.0f}  // CH 15: 94° flat ground, active stance and swing
};

// =============================================================================
// 4. Kinematics & Locomotion Tuning Defaults
// =============================================================================
// Link Dimensions from Reference Inverse Kinematics (cm)
#define LINK_THIGH_L1_CM           5.00f  // Thigh length l1
#define LINK_SHIN_L2_CM            5.70f  // Shin length l2

#define DEFAULT_WALK_FREQ_HZ       0.55f  // Stable, judder-free cadence frequency
#define DEFAULT_STEP_LENGTH_CM     1.40f  // Balanced Cartesian stride
#define DEFAULT_STEP_CLEARANCE_CM  0.95f  // Ample vertical foot lift (prevents ground catching)
#define DEFAULT_STEP_HEIGHT_CM     9.80f  // Relaxed standing height (prevents knee joint lockout)

#define ROBOT_TOTAL_WEIGHT_GRAMS   500.0f // Nominal robot mass

// FreeRTOS Locomotion Task Timing
#define LOCOMOTION_TASK_FREQ_HZ    50     // 50Hz real-time cycle (matches PCA9685 50Hz PWM)
#define LOCOMOTION_PERIOD_MS       (1000 / LOCOMOTION_TASK_FREQ_HZ)

#endif // CONFIG_H
