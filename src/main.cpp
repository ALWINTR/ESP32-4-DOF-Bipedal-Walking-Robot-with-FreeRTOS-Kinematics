#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <math.h>

// ==========================================
// 1. WiFi Configuration & Constants
// ==========================================
const char* ap_ssid = "ESP32-Biped-Robot";
const char* ap_password = "12345678";

WiFiMulti wifiMulti;
WebServer server(80);

// ==========================================
// 2. 6-DOF Servo & Calibrated Offsets
// ==========================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Preferences preferences;
bool pcaFound = false;

#define SERVOMIN_US  544
#define SERVOMAX_US  2400

#define NUM_SERVOS 6

struct Joint {
  int channel;
  const char* name;
  float currentAngle;
  float targetAngle;
  int lastPulseLen;
  int homeAngle;
};

// Calibrated 6-DOF Baselines (Exact Physical Wiring Specifications):
// Right Leg: CH 0 (Hip = 94°), CH 1 (Knee = 8°),  CH 12 (Foot = 94°)
// Left Leg:  CH 4 (Hip = 94°), CH 8 (Knee = 94°), CH 15 (Foot = 94°)
Joint servos[NUM_SERVOS] = {
  {0,  "Right Hip",   94.0f, 94.0f, -1, 94},
  {1,  "Right Knee",   8.0f,  8.0f, -1,  8},
  {12, "Right Foot",  94.0f, 94.0f, -1, 94},
  {4,  "Left Hip",    94.0f, 94.0f, -1, 94},
  {8,  "Left Knee",   94.0f, 94.0f, -1, 94},
  {15, "Left Foot",   94.0f, 94.0f, -1, 94}
};

String activeGaitName = "stand";
bool gaitRunning = false;

// ==========================================
// Dynamic Bipedal Walking Parameters (Designed from Scratch)
// Baseline standing angles remain 100% constant:
// Right Leg: CH 0 (Hip 94°), CH 1 (Knee 8°),  CH 12 (Foot 94°)
// Left Leg:  CH 4 (Hip 94°), CH 8 (Knee 94°), CH 15 (Foot 94°)
// ==========================================
float walkFreq         = 0.70f; // Cadence frequency (Hz) - 0.70Hz judder-free cadence
float stepLength       = 1.6f;  // Cartesian stride length (cm)
float stepClearance    = 1.00f; // Cartesian foot ground clearance lift (cm) - 1.0cm (user configured)
float stepHeight       = 10.00f;// Cartesian standing stance height (cm) - 10.0cm (user configured)
float currentGaitScale = 0.0f;  // Smooth anti-jerk startup and stop ramp
unsigned long walkStartTime = 0;

// Leg Link Dimensions
float l1 = 5.0f; // Thigh length (cm)
float l2 = 5.7f; // Shin length (cm)

int hipLOffset   = 94;
int kneeLOffset  = 94;
int ankleLOffset = 94;

int hipROffset   = 94;
int kneeROffset  = 8;
int ankleROffset = 94;

// Dynamic Ground Reaction Force & Weight Telemetry (calculated during walking)
float robotTotalWeightGrams = 500.0f; // Nominal mass (500g)
float weightLeftGrams       = 250.0f; // Live estimated ground load on Left Foot (g)
float weightRightGrams      = 250.0f; // Live estimated ground load on Right Foot (g)
float groundForceNewtons    = 4.905f; // Total Ground Reaction Force (N)
String groundContactState   = "DUAL STANCE";

struct IKResult {
  float hipDeg;
  float kneeDeg;
  float ankleDeg;
};

IKResult solveIK(float x, float z) {
  IKResult res;
  float hipRad2 = atan2(x, z);
  float hipDeg2 = hipRad2 * (180.0f / (float)M_PI);

  float z2 = sqrt(x * x + z * z);

  float val1 = (l1 * l1 + z2 * z2 - l2 * l2) / (2.0f * l1 * z2);
  val1 = constrain(val1, -1.0f, 1.0f);
  float hipRad1 = acos(val1);
  float hipDeg1 = hipRad1 * (180.0f / (float)M_PI);

  float val2 = (l1 * l1 + l2 * l2 - z2 * z2) / (2.0f * l1 * l2);
  val2 = constrain(val2, -1.0f, 1.0f);
  float kneeRad = (float)M_PI - acos(val2);
  float kneeDeg = kneeRad * (180.0f / (float)M_PI);

  float val3 = (l2 * l2 + z2 * z2 - l1 * l1) / (2.0f * l2 * z2);
  val3 = constrain(val3, -1.0f, 1.0f);
  float ankleRad = ((float)M_PI / 2.0f) + hipRad2 - acos(val3);
  float ankleDeg = ankleRad * (180.0f / (float)M_PI);

  res.hipDeg = hipDeg1 + hipDeg2;
  res.kneeDeg = kneeDeg;
  res.ankleDeg = ankleDeg;
  return res;
}

void loadSavedOffsets() {
  preferences.begin("biped_chmap", false);
  servos[0].homeAngle = preferences.getInt("ch0", 94);  // Right Hip
  servos[1].homeAngle = preferences.getInt("ch1", 8);   // Right Knee
  servos[2].homeAngle = preferences.getInt("ch12", 94); // Right Foot
  servos[3].homeAngle = preferences.getInt("ch4", 94);  // Left Hip
  servos[4].homeAngle = preferences.getInt("ch8", 94);  // Left Knee
  servos[5].homeAngle = preferences.getInt("ch15", 94); // Left Foot
  preferences.end();

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = (float)servos[i].homeAngle;
    servos[i].targetAngle  = (float)servos[i].homeAngle;
  }
}

void saveCurrentOffsets() {
  preferences.begin("biped_chmap", false);
  preferences.putInt("ch0", (int)servos[0].currentAngle);
  preferences.putInt("ch1", (int)servos[1].currentAngle);
  preferences.putInt("ch12", (int)servos[2].currentAngle);
  preferences.putInt("ch4", (int)servos[3].currentAngle);
  preferences.putInt("ch8", (int)servos[4].currentAngle);
  preferences.putInt("ch15", (int)servos[5].currentAngle);
  preferences.end();

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].homeAngle = (int)servos[i].currentAngle;
  }
}

void resetFactoryOffsets() {
  preferences.begin("biped_chmap", false);
  preferences.clear();
  preferences.end();
  servos[0].homeAngle = 94; // Right Hip
  servos[1].homeAngle = 8;  // Right Knee
  servos[2].homeAngle = 94; // Right Foot (CH 12)
  servos[3].homeAngle = 94; // Left Hip
  servos[4].homeAngle = 94; // Left Knee
  servos[5].homeAngle = 94; // Left Foot (CH 15)
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = (float)servos[i].homeAngle;
    servos[i].targetAngle  = (float)servos[i].homeAngle;
  }
}

// ==========================================
// 3. HTML Web Dashboard (6-DOF Full Studio)
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Biped Robot - Mission Control Studio</title>
  <style>
    :root {
      --bg-dark: #070b14;
      --bg-card: rgba(15, 23, 42, 0.72);
      --bg-card-hover: rgba(30, 41, 59, 0.85);
      --border-card: rgba(56, 189, 248, 0.16);
      --border-card-active: rgba(56, 189, 248, 0.5);
      --accent-cyan: #06b6d4;
      --accent-sky: #38bdf8;
      --accent-indigo: #6366f1;
      --accent-amber: #f59e0b;
      --accent-emerald: #10b981;
      --accent-rose: #f43f5e;
      --accent-violet: #8b5cf6;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --radius-sm: 8px;
      --radius-md: 14px;
      --radius-lg: 20px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; user-select: none; -webkit-tap-highlight-color: transparent; }
    body {
      background: radial-gradient(circle at 50% 0%, #0d1e38 0%, #070b14 70%);
      color: var(--text-main);
      min-height: 100vh;
      padding: 12px;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    .container { width: 100%; max-width: 920px; display: flex; flex-direction: column; gap: 14px; }

    /* Top HUD Header */
    header {
      background: var(--bg-card);
      backdrop-filter: blur(14px);
      -webkit-backdrop-filter: blur(14px);
      border: 1px solid var(--border-card);
      border-radius: var(--radius-lg);
      padding: 12px 18px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      box-shadow: 0 10px 30px rgba(0,0,0,0.45);
    }
    .brand-box { display: flex; align-items: center; gap: 12px; }
    .brand-icon {
      width: 40px; height: 40px; border-radius: 12px;
      background: linear-gradient(135deg, var(--accent-cyan), var(--accent-indigo));
      display: flex; align-items: center; justify-content: center; font-size: 1.3rem;
      box-shadow: 0 0 18px rgba(6, 182, 212, 0.45);
    }
    .brand-title {
      font-size: 1.15rem; font-weight: 900; letter-spacing: 0.5px;
      background: linear-gradient(135deg, #38bdf8, #818cf8, #c084fc);
      -webkit-background-clip: text; -webkit-text-fill-color: transparent;
    }
    .brand-sub { font-size: 0.68rem; color: var(--text-muted); font-weight: 700; text-transform: uppercase; letter-spacing: 1px; }
    .hud-badges { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
    .hud-badge {
      display: flex; align-items: center; gap: 6px; font-size: 0.72rem; font-weight: 800;
      background: rgba(0,0,0,0.4); padding: 5px 10px; border-radius: 20px; border: 1px solid var(--border-card);
    }
    .dot-pulse { width: 8px; height: 8px; border-radius: 50%; background: var(--accent-emerald); box-shadow: 0 0 10px var(--accent-emerald); animation: pulse 2s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 0.4; } 50% { opacity: 1; } }

    /* Cards */
    .card {
      background: var(--bg-card);
      backdrop-filter: blur(14px);
      -webkit-backdrop-filter: blur(14px);
      border: 1px solid var(--border-card);
      border-radius: var(--radius-md);
      padding: 15px;
      display: flex;
      flex-direction: column;
      gap: 12px;
      box-shadow: 0 8px 24px rgba(0,0,0,0.3);
    }
    .card-header {
      display: flex; justify-content: space-between; align-items: center;
      border-bottom: 1px solid rgba(255,255,255,0.06); padding-bottom: 9px;
    }
    .card-title { font-size: 0.92rem; font-weight: 900; letter-spacing: 0.5px; display: flex; align-items: center; gap: 8px; }
    .tab-pills { display: flex; gap: 6px; }
    .tab-pill {
      background: rgba(255,255,255,0.06); border: 1px solid var(--border-card);
      color: var(--text-muted); font-size: 0.72rem; font-weight: 800; padding: 4px 10px;
      border-radius: 12px; cursor: pointer; transition: all 0.2s;
    }
    .tab-pill.active {
      background: linear-gradient(135deg, rgba(6,182,212,0.3), rgba(99,102,241,0.3));
      border-color: var(--accent-sky); color: #fff; box-shadow: 0 0 10px rgba(6,182,212,0.3);
    }

    /* Motion Commander Deck */
    .motion-grid { display: grid; grid-template-columns: 1fr 1.35fr; gap: 14px; align-items: center; }
    @media (max-width: 680px) { .motion-grid { grid-template-columns: 1fr; } }

    /* D-Pad */
    .dpad-wrapper { display: flex; flex-direction: column; align-items: center; gap: 8px; }
    .dpad {
      display: grid; grid-template-columns: repeat(3, 70px); grid-template-rows: repeat(3, 70px);
      gap: 7px; justify-content: center; margin: auto;
    }
    .dpad-btn {
      background: rgba(255,255,255,0.05); border: 1px solid var(--border-card);
      color: var(--text-main); font-size: 1.25rem; font-weight: 900; border-radius: var(--radius-md);
      display: flex; flex-direction: column; align-items: center; justify-content: center;
      cursor: pointer; transition: all 0.15s;
    }
    .dpad-btn:hover {
      background: rgba(6,182,212,0.22); border-color: var(--accent-cyan);
      color: var(--accent-cyan); box-shadow: 0 0 15px rgba(6,182,212,0.35); transform: translateY(-1px);
    }
    .dpad-btn:active { transform: scale(0.92); }
    .dpad-btn span { font-size: 0.58rem; font-weight: 800; text-transform: uppercase; margin-top: 2px; }
    .d-up { grid-column: 2; grid-row: 1; }
    .d-left { grid-column: 1; grid-row: 2; }
    .d-stop {
      grid-column: 2; grid-row: 2; background: rgba(244,63,94,0.22);
      border-color: var(--accent-rose); color: #fda4af; box-shadow: 0 0 12px rgba(244,63,94,0.3);
    }
    .d-stop:hover { background: var(--accent-rose); color: #fff; }
    .d-right { grid-column: 3; grid-row: 2; }
    .d-down { grid-column: 2; grid-row: 3; }

    /* Action Buttons Box */
    .action-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .action-card {
      background: rgba(255,255,255,0.04); border: 1px solid var(--border-card);
      border-radius: var(--radius-sm); padding: 10px 12px; cursor: pointer;
      display: flex; flex-direction: column; gap: 4px; transition: all 0.15s; text-align: left;
    }
    .action-card:hover {
      background: rgba(255,255,255,0.08); border-color: var(--accent-sky);
      transform: translateY(-1px); box-shadow: 0 4px 12px rgba(0,0,0,0.3);
    }
    .action-card:active { transform: scale(0.97); }
    .action-title { font-size: 0.82rem; font-weight: 800; display: flex; align-items: center; justify-content: space-between; }
    .action-desc { font-size: 0.65rem; color: var(--text-muted); font-weight: 600; }
    .act-walk { border-color: rgba(16,185,129,0.35); background: rgba(16,185,129,0.1); }
    .act-walk .action-title { color: #6ee7b7; }
    .act-step { border-color: rgba(99,102,241,0.35); background: rgba(99,102,241,0.1); }
    .act-step .action-title { color: #a5b4fc; }
    .act-ref { border-color: rgba(139,92,246,0.35); background: rgba(139,92,246,0.1); }
    .act-ref .action-title { color: #c4b5fd; }
    .act-test { border-color: rgba(245,158,11,0.35); background: rgba(245,158,11,0.1); }
    .act-test .action-title { color: #fde68a; }
    .act-stop { border-color: rgba(244,63,94,0.35); background: rgba(244,63,94,0.12); grid-column: span 2; }
    .act-stop .action-title { color: #fca5a5; }

    /* SVG Schematic */
    .schematic-svg {
      width: 100%; max-height: 380px; filter: drop-shadow(0 0 14px rgba(0,0,0,0.5));
      border-radius: 12px; background: radial-gradient(circle at 50% 50%, rgba(15,23,42,0.6) 0%, rgba(6,10,18,0.95) 100%);
      border: 1px solid rgba(255,255,255,0.06);
    }
    .joint-node { cursor: pointer; transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1); }
    .joint-node:hover circle.joint-outer { stroke-width: 3.5px; filter: brightness(1.3); }
    .joint-node.selected circle.joint-outer { stroke-width: 4px; stroke: #fff !important; filter: drop-shadow(0 0 8px #fff); }

    /* Nudge Toolbar */
    .nudge-toolbar {
      width: 100%; background: rgba(0,0,0,0.4); border: 1px solid var(--border-card);
      border-radius: var(--radius-sm); padding: 8px 12px; display: flex; flex-wrap: wrap;
      justify-content: space-between; align-items: center; gap: 8px; margin-top: 4px;
    }
    .nudge-info { display: flex; align-items: center; gap: 8px; font-size: 0.8rem; }
    .nudge-badge {
      background: rgba(56,189,248,0.16); border: 1px solid var(--accent-sky);
      color: var(--accent-sky); font-weight: 800; padding: 2px 8px; border-radius: 6px;
    }
    .nudge-buttons { display: flex; gap: 5px; flex-wrap: wrap; }
    .nudge-btn {
      background: rgba(255,255,255,0.07); border: 1px solid var(--border-card);
      color: var(--text-main); font-size: 0.75rem; font-weight: 800; padding: 5px 9px;
      border-radius: 6px; cursor: pointer; transition: all 0.15s;
    }
    .nudge-btn:hover { background: var(--accent-sky); color: #000; border-color: var(--accent-sky); }
    .nudge-btn:active { transform: scale(0.92); }
    .nudge-home { background: rgba(16,185,129,0.18); border-color: var(--accent-emerald); color: #6ee7b7; }
    .nudge-home:hover { background: var(--accent-emerald); color: #000; }

    /* Telemetry Grid */
    .telemetry-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 8px; width: 100%; }
    .telem-card {
      background: rgba(0,0,0,0.32); border: 1px solid rgba(255,255,255,0.06);
      border-radius: var(--radius-sm); padding: 9px; display: flex; flex-direction: column;
      align-items: center; gap: 4px; cursor: pointer; transition: all 0.15s;
    }
    .telem-card:hover { border-color: var(--border-card-active); background: rgba(0,0,0,0.48); }
    .telem-name { font-size: 0.7rem; font-weight: 800; color: var(--text-muted); text-transform: uppercase; }
    .telem-angle { font-size: 1.3rem; font-weight: 900; font-family: ui-monospace, monospace; }
    .telem-r { color: var(--accent-amber); }
    .telem-l { color: var(--accent-cyan); }
    .telem-bar-track { width: 100%; height: 4px; background: rgba(255,255,255,0.08); border-radius: 2px; overflow: hidden; }
    .telem-bar-fill { height: 100%; width: 50%; border-radius: 2px; transition: width 0.15s; }

    /* Ground Weight & Reaction Force HUD */
    .weight-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; width: 100%; }
    @media (max-width: 600px) { .weight-grid { grid-template-columns: 1fr; } }
    .weight-card {
      background: rgba(0,0,0,0.32); border: 1px solid var(--border-card);
      border-radius: var(--radius-sm); padding: 12px 14px; display: flex; flex-direction: column; gap: 7px;
    }
    .weight-card-l { border-color: rgba(6, 182, 212, 0.35); }
    .weight-card-r { border-color: rgba(245, 158, 11, 0.35); }
    .weight-title-row { display: flex; justify-content: space-between; align-items: center; }
    .weight-title { font-size: 0.8rem; font-weight: 800; text-transform: uppercase; letter-spacing: 0.5px; }
    .weight-val { font-size: 1.5rem; font-weight: 900; font-family: ui-monospace, monospace; }
    .weight-bar-bg { width: 100%; height: 9px; background: rgba(255,255,255,0.08); border-radius: 5px; overflow: hidden; }
    .weight-bar-fill { height: 100%; border-radius: 5px; transition: width 0.12s ease-out; }
    .grf-summary-bar {
      display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px;
      padding: 9px 13px; background: rgba(0,0,0,0.36); border-radius: var(--radius-sm); border: 1px solid var(--border-card);
    }

    /* Joint Control Cards */
    .leg-section-title {
      font-size: 0.82rem; font-weight: 800; padding: 7px 12px; border-radius: 8px;
      width: 100%; display: flex; justify-content: space-between; align-items: center;
    }
    .leg-r-header { background: rgba(245, 158, 11, 0.14); color: var(--accent-amber); border-left: 4px solid var(--accent-amber); }
    .leg-l-header { background: rgba(6, 182, 212, 0.14); color: var(--accent-cyan); border-left: 4px solid var(--accent-cyan); }
    .servo-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 10px; width: 100%; }
    .servo-card {
      background: rgba(0,0,0,0.25); border: 1px solid var(--border-card);
      border-radius: var(--radius-sm); padding: 12px; display: flex; flex-direction: column;
      gap: 7px; transition: border-color 0.2s;
    }
    .servo-card.active-target { border-color: var(--accent-sky); box-shadow: 0 0 12px rgba(56,189,248,0.2); }
    .servo-header { display: flex; justify-content: space-between; align-items: center; }
    .servo-name { font-size: 0.82rem; font-weight: 800; }
    .servo-angle-box { display: flex; align-items: baseline; gap: 6px; }
    .servo-angle-text { font-size: 1.05rem; font-weight: 900; font-family: ui-monospace, monospace; }
    .servo-home-tag { font-size: 0.68rem; color: var(--text-muted); font-weight: 700; }
    .slider-styled {
      width: 100%; height: 6px; accent-color: var(--accent-cyan); cursor: pointer;
      background: rgba(255,255,255,0.1); border-radius: 3px;
    }
    .servo-card-r .slider-styled { accent-color: var(--accent-amber); }
    .step-btn-group { display: grid; grid-template-columns: repeat(7, 1fr); gap: 4px; }
    .step-btn {
      background: rgba(255,255,255,0.06); border: 1px solid var(--border-card);
      color: var(--text-muted); font-size: 0.7rem; padding: 5px 2px; border-radius: 4px;
      cursor: pointer; font-weight: 800; text-align: center;
    }
    .step-btn:hover { background: var(--accent-sky); color: #000; }
    .step-btn:active { transform: scale(0.92); }
    .btn-reset-single { background: rgba(16,185,129,0.15); border-color: var(--accent-emerald); color: #6ee7b7; font-size: 0.65rem; }
    .btn-reset-single:hover { background: var(--accent-emerald); color: #000; }

    /* Tuner Sliders */
    .tuner-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 8px; width: 100%; }
    .tuner-card {
      background: rgba(0,0,0,0.25); border: 1px solid var(--border-card);
      border-radius: var(--radius-sm); padding: 10px; display: flex; flex-direction: column; gap: 6px;
    }
    .tuner-header { display: flex; justify-content: space-between; align-items: center; }
    .tuner-label { font-size: 0.75rem; font-weight: 700; color: var(--text-muted); }
    .tuner-val { font-size: 0.9rem; font-weight: 900; color: var(--accent-sky); font-family: ui-monospace, monospace; }

    /* Footer System Actions */
    .sys-actions { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 8px; width: 100%; }
    .btn-sys {
      background: rgba(255,255,255,0.05); border: 1px solid var(--border-card);
      color: var(--text-main); font-weight: 800; padding: 11px; border-radius: var(--radius-sm);
      cursor: pointer; font-size: 0.8rem; display: flex; align-items: center; justify-content: center; gap: 6px;
      transition: all 0.15s;
    }
    .btn-sys:active { transform: scale(0.98); }
    .btn-save { border-color: var(--accent-emerald); background: rgba(16,185,129,0.15); color: #6ee7b7; }
    .btn-save:hover { background: var(--accent-emerald); color: #000; }
    .btn-restore { border-color: var(--accent-rose); background: rgba(244,63,94,0.15); color: #fda4af; }
    .btn-restore:hover { background: var(--accent-rose); color: #fff; }
    .btn-scan { border-color: var(--accent-sky); background: rgba(56,189,248,0.15); color: #7dd3fc; }
    .btn-scan:hover { background: var(--accent-sky); color: #000; }

    /* Toast Notification */
    #toast {
      position: fixed; bottom: 20px; right: 20px; z-index: 9999;
      background: rgba(15,23,42,0.95); backdrop-filter: blur(12px);
      border: 1px solid var(--accent-sky); border-radius: 12px;
      padding: 10px 18px; font-size: 0.85rem; font-weight: 800; color: #fff;
      box-shadow: 0 10px 30px rgba(0,0,0,0.6); display: none; align-items: center; gap: 8px;
      transform: translateY(20px); transition: all 0.25s ease;
    }
    #toast.show { display: flex; transform: translateY(0); }
  </style>
</head>
<body>
  <div class="container">
    <!-- Top HUD Header -->
    <header>
      <div class="brand-box">
        <div class="brand-icon">🤖</div>
        <div>
          <div class="brand-title">6-DOF BIPED ROBOT</div>
          <div class="brand-sub">MISSION CONTROL & KINEMATICS STUDIO</div>
        </div>
      </div>
      <div class="hud-badges">
        <div class="hud-badge">
          <div class="dot-pulse"></div>
          <span id="status-text">ONLINE</span>
        </div>
        <div class="hud-badge" style="color: var(--accent-sky);">
          <span>PCA9685: 0x40 (50Hz)</span>
        </div>
        <div class="hud-badge" style="color: var(--accent-emerald);">
          <span id="hud-active-gait">STAND</span>
        </div>
      </div>
    </header>

    <!-- Interactive Schematic Card -->
    <section class="card">
      <div class="card-header">
        <div class="card-title">
          <span>📐</span>
          <span>INTERACTIVE ROBOT RIG & SCHEMATIC</span>
        </div>
        <div class="tab-pills">
          <button class="tab-pill active" id="tab-btn-kinematic" onclick="switchSchematic('kinematic')">Kinematic Rig</button>
          <button class="tab-pill" id="tab-btn-circuit" onclick="switchSchematic('circuit')">Circuit Pinout</button>
        </div>
      </div>

      <!-- Schematic Container -->
      <div style="width: 100%; display: flex; flex-direction: column; align-items: center;">
        <!-- Kinematic Anatomy SVG -->
        <svg id="svg-kinematic" class="schematic-svg" viewBox="0 0 540 370">
          <defs>
            <linearGradient id="grad-r-link" x1="0%" y1="0%" x2="100%" y2="100%">
              <stop offset="0%" stop-color="#f59e0b" stop-opacity="0.9" />
              <stop offset="100%" stop-color="#d97706" stop-opacity="0.7" />
            </linearGradient>
            <linearGradient id="grad-l-link" x1="0%" y1="0%" x2="100%" y2="100%">
              <stop offset="0%" stop-color="#06b6d4" stop-opacity="0.9" />
              <stop offset="100%" stop-color="#0284c7" stop-opacity="0.7" />
            </linearGradient>
            <filter id="glow-amber" x="-20%" y="-20%" width="140%" height="140%">
              <feGaussianBlur stdDeviation="4" result="blur" />
              <feComposite in="SourceGraphic" in2="blur" operator="over" />
            </filter>
            <filter id="glow-cyan" x="-20%" y="-20%" width="140%" height="140%">
              <feGaussianBlur stdDeviation="4" result="blur" />
              <feComposite in="SourceGraphic" in2="blur" operator="over" />
            </filter>
          </defs>

          <!-- Grid Background lines -->
          <line x1="40" y1="90" x2="500" y2="90" stroke="rgba(255,255,255,0.04)" stroke-dasharray="4" />
          <line x1="40" y1="190" x2="500" y2="190" stroke="rgba(255,255,255,0.04)" stroke-dasharray="4" />
          <line x1="40" y1="290" x2="500" y2="290" stroke="rgba(255,255,255,0.04)" stroke-dasharray="4" />

          <!-- Center Pelvis / Controller Core -->
          <rect x="200" y="20" width="140" height="48" rx="8" fill="#1e293b" stroke="#38bdf8" stroke-width="2" />
          <text x="270" y="40" text-anchor="middle" fill="#38bdf8" font-size="11" font-weight="900">ESP32 + PCA9685</text>
          <text x="270" y="55" text-anchor="middle" fill="#94a3b8" font-size="9" font-weight="700">PELVIS CORE (0x40)</text>

          <!-- Pelvis to Hip Structural Brackets -->
          <path d="M 220 68 L 140 100" stroke="#f59e0b" stroke-width="3" stroke-linecap="round" />
          <path d="M 320 68 L 400 100" stroke="#06b6d4" stroke-width="3" stroke-linecap="round" />

          <!-- Right Leg Links -->
          <line id="link-thigh-r" x1="140" y1="100" x2="140" y2="200" stroke="url(#grad-r-link)" stroke-width="6" stroke-linecap="round" />
          <line id="link-shin-r" x1="140" y1="200" x2="140" y2="300" stroke="url(#grad-r-link)" stroke-width="6" stroke-linecap="round" />
          <rect id="plate-foot-r" x="105" y="318" width="70" height="14" rx="4" fill="#f59e0b" opacity="0.85" />
          <text x="140" y="329" text-anchor="middle" fill="#000" font-size="8" font-weight="900">R.FOOT SOLE</text>

          <!-- Left Leg Links -->
          <line id="link-thigh-l" x1="400" y1="100" x2="400" y2="200" stroke="url(#grad-l-link)" stroke-width="6" stroke-linecap="round" />
          <line id="link-shin-l" x1="400" y1="200" x2="400" y2="300" stroke="url(#grad-l-link)" stroke-width="6" stroke-linecap="round" />
          <rect id="plate-foot-l" x="365" y="318" width="70" height="14" rx="4" fill="#06b6d4" opacity="0.85" />
          <text x="400" y="329" text-anchor="middle" fill="#000" font-size="8" font-weight="900">L.FOOT SOLE</text>

          <!-- Joint Nodes (Right Leg) -->
          <g class="joint-node selected" id="node-0" onclick="selectJoint(0, 'Right Hip')">
            <circle class="joint-outer" cx="140" cy="100" r="22" fill="#0f172a" stroke="#f59e0b" stroke-width="2.5" filter="url(#glow-amber)" />
            <text x="140" y="96" text-anchor="middle" fill="#f59e0b" font-size="9" font-weight="800">CH 0</text>
            <text id="svg-angle-0" x="140" y="110" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="80" y="104" text-anchor="middle" fill="#f59e0b" font-size="10" font-weight="700">R.HIP</text>
          </g>
          <g class="joint-node" id="node-1" onclick="selectJoint(1, 'Right Knee')">
            <circle class="joint-outer" cx="140" cy="200" r="22" fill="#0f172a" stroke="#f59e0b" stroke-width="2.5" filter="url(#glow-amber)" />
            <text x="140" y="196" text-anchor="middle" fill="#f59e0b" font-size="9" font-weight="800">CH 1</text>
            <text id="svg-angle-1" x="140" y="210" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">8°</text>
            <text x="80" y="204" text-anchor="middle" fill="#f59e0b" font-size="10" font-weight="700">R.KNEE</text>
          </g>
          <g class="joint-node" id="node-12" onclick="selectJoint(12, 'Right Foot')">
            <circle class="joint-outer" cx="140" cy="300" r="22" fill="#0f172a" stroke="#f59e0b" stroke-width="2.5" filter="url(#glow-amber)" />
            <text x="140" y="296" text-anchor="middle" fill="#f59e0b" font-size="9" font-weight="800">CH 12</text>
            <text id="svg-angle-12" x="140" y="310" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="80" y="304" text-anchor="middle" fill="#f59e0b" font-size="10" font-weight="700">R.FOOT</text>
          </g>

          <!-- Joint Nodes (Left Leg) -->
          <g class="joint-node" id="node-4" onclick="selectJoint(4, 'Left Hip')">
            <circle class="joint-outer" cx="400" cy="100" r="22" fill="#0f172a" stroke="#06b6d4" stroke-width="2.5" filter="url(#glow-cyan)" />
            <text x="400" y="96" text-anchor="middle" fill="#06b6d4" font-size="9" font-weight="800">CH 4</text>
            <text id="svg-angle-4" x="400" y="110" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="460" y="104" text-anchor="middle" fill="#06b6d4" font-size="10" font-weight="700">L.HIP</text>
          </g>
          <g class="joint-node" id="node-8" onclick="selectJoint(8, 'Left Knee')">
            <circle class="joint-outer" cx="400" cy="200" r="22" fill="#0f172a" stroke="#06b6d4" stroke-width="2.5" filter="url(#glow-cyan)" />
            <text x="400" y="196" text-anchor="middle" fill="#06b6d4" font-size="9" font-weight="800">CH 8</text>
            <text id="svg-angle-8" x="400" y="210" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="460" y="204" text-anchor="middle" fill="#06b6d4" font-size="10" font-weight="700">L.KNEE</text>
          </g>
          <g class="joint-node" id="node-15" onclick="selectJoint(15, 'Left Foot')">
            <circle class="joint-outer" cx="400" cy="300" r="22" fill="#0f172a" stroke="#06b6d4" stroke-width="2.5" filter="url(#glow-cyan)" />
            <text x="400" y="296" text-anchor="middle" fill="#06b6d4" font-size="9" font-weight="800">CH 15</text>
            <text id="svg-angle-15" x="400" y="310" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="460" y="304" text-anchor="middle" fill="#06b6d4" font-size="10" font-weight="700">L.FOOT</text>
          </g>

          <!-- Center Status Card in SVG -->
          <rect x="210" y="150" width="120" height="65" rx="6" fill="rgba(15,23,42,0.88)" stroke="rgba(255,255,255,0.12)" />
          <text x="270" y="172" text-anchor="middle" fill="#94a3b8" font-size="9" font-weight="700">GAIT STATE</text>
          <text id="svg-active-gait" x="270" y="195" text-anchor="middle" fill="#10b981" font-size="14" font-weight="900">STAND</text>
        </svg>

        <!-- Circuit Schematic SVG (Hidden by default) -->
        <svg id="svg-circuit" class="schematic-svg" viewBox="0 0 540 370" style="display: none;">
          <rect x="30" y="40" width="140" height="290" rx="8" fill="#1e293b" stroke="#38bdf8" stroke-width="2" />
          <text x="100" y="65" text-anchor="middle" fill="#38bdf8" font-size="12" font-weight="900">ESP32 DevKit</text>
          <text x="100" y="80" text-anchor="middle" fill="#64748b" font-size="8" font-weight="600">30-PIN MCU</text>
          <rect x="135" y="110" width="30" height="18" rx="3" fill="#0284c7" />
          <text x="150" y="123" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">G21 (SDA)</text>
          <rect x="135" y="145" width="30" height="18" rx="3" fill="#0284c7" />
          <text x="150" y="158" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">G22 (SCL)</text>
          <rect x="135" y="190" width="30" height="18" rx="3" fill="#dc2626" />
          <text x="150" y="203" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">3V3</text>
          <rect x="135" y="225" width="30" height="18" rx="3" fill="#475569" />
          <text x="150" y="238" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">GND</text>

          <rect x="230" y="40" width="140" height="290" rx="8" fill="#0f172a" stroke="#6366f1" stroke-width="2" />
          <text x="300" y="65" text-anchor="middle" fill="#818cf8" font-size="12" font-weight="900">PCA9685 Driver</text>
          <text x="300" y="80" text-anchor="middle" fill="#64748b" font-size="8" font-weight="600">I2C: 0x40 (50Hz PWM)</text>
          <path d="M 165 119 L 230 119" stroke="#38bdf8" stroke-width="2.5" />
          <path d="M 165 154 L 230 154" stroke="#818cf8" stroke-width="2.5" />
          <path d="M 165 199 L 230 199" stroke="#ef4444" stroke-width="2" />
          <path d="M 165 234 L 230 234" stroke="#64748b" stroke-width="2" />

          <rect x="250" y="290" width="100" height="30" rx="4" fill="#334155" stroke="#f59e0b" />
          <text x="300" y="309" text-anchor="middle" fill="#fbbf24" font-size="9" font-weight="900">EXT 5V-6V (V+ / GND)</text>

          <g><path d="M 370 100 L 415 100" stroke="#f59e0b" stroke-width="2" /><rect x="415" y="90" width="100" height="22" rx="4" fill="#1e293b" stroke="#f59e0b" /><text x="465" y="104" text-anchor="middle" fill="#fbbf24" font-size="8" font-weight="800">CH 0: Right Hip</text></g>
          <g><path d="M 370 135 L 415 135" stroke="#f59e0b" stroke-width="2" /><rect x="415" y="125" width="100" height="22" rx="4" fill="#1e293b" stroke="#f59e0b" /><text x="465" y="139" text-anchor="middle" fill="#fbbf24" font-size="8" font-weight="800">CH 1: Right Knee</text></g>
          <g><path d="M 370 170 L 415 170" stroke="#f59e0b" stroke-width="2" /><rect x="415" y="160" width="100" height="22" rx="4" fill="#1e293b" stroke="#f59e0b" /><text x="465" y="174" text-anchor="middle" fill="#fbbf24" font-size="8" font-weight="800">CH 12: Right Foot</text></g>
          <g><path d="M 370 215 L 415 215" stroke="#06b6d4" stroke-width="2" /><rect x="415" y="205" width="100" height="22" rx="4" fill="#1e293b" stroke="#06b6d4" /><text x="465" y="219" text-anchor="middle" fill="#67e8f9" font-size="8" font-weight="800">CH 4: Left Hip</text></g>
          <g><path d="M 370 250 L 415 250" stroke="#06b6d4" stroke-width="2" /><rect x="415" y="240" width="100" height="22" rx="4" fill="#1e293b" stroke="#06b6d4" /><text x="465" y="254" text-anchor="middle" fill="#67e8f9" font-size="8" font-weight="800">CH 8: Left Knee</text></g>
          <g><path d="M 370 285 L 415 285" stroke="#06b6d4" stroke-width="2" /><rect x="415" y="275" width="100" height="22" rx="4" fill="#1e293b" stroke="#06b6d4" /><text x="465" y="289" text-anchor="middle" fill="#67e8f9" font-size="8" font-weight="800">CH 15: Left Foot</text></g>
        </svg>

        <!-- Quick Nudge Toolbar -->
        <div class="nudge-toolbar">
          <div class="nudge-info">
            <span style="color: var(--text-muted);">ACTIVE JOINT:</span>
            <span class="nudge-badge" id="selected-joint-badge">Right Hip (CH 0)</span>
            <span style="font-family: monospace; font-weight: 900; font-size: 0.95rem;" id="selected-joint-angle">94°</span>
          </div>
          <div class="nudge-buttons">
            <button class="nudge-btn" onclick="stepSelected(-10)">-10°</button>
            <button class="nudge-btn" onclick="stepSelected(-5)">-5°</button>
            <button class="nudge-btn" onclick="stepSelected(-1)">-1°</button>
            <button class="nudge-btn nudge-home" onclick="homeSelected()">HOME</button>
            <button class="nudge-btn" onclick="stepSelected(+1)">+1°</button>
            <button class="nudge-btn" onclick="stepSelected(+5)">+5°</button>
            <button class="nudge-btn" onclick="stepSelected(+10)">+10°</button>
          </div>
        </div>
      </div>
    </section>

    <!-- Motion Commander Deck -->
    <section class="card">
      <div class="card-header">
        <div class="card-title"><span>🎮</span><span>MOTION COMMANDER & GAIT CONTROL</span></div>
      </div>
      <div class="motion-grid">
        <!-- D-Pad -->
        <div class="dpad-wrapper">
          <div class="dpad">
            <button class="dpad-btn d-up" onclick="runGait('walk')">▲<span>Forward</span></button>
            <button class="dpad-btn d-left" onclick="runGait('left')">◀<span>Turn L</span></button>
            <button class="dpad-btn d-stop" onclick="runGait('stand')">🛑<span>Stop</span></button>
            <button class="dpad-btn d-right" onclick="runGait('right')">▶<span>Turn R</span></button>
            <button class="dpad-btn d-down" onclick="runGait('backward')">▼<span>Back</span></button>
          </div>
        </div>

        <!-- Action Cards Grid -->
        <div class="action-grid">
          <!-- 🟢 START / INITIALIZE STAND (Dedicated Primary Button) -->
          <div class="action-card act-start" onclick="runGait('start')" style="grid-column: span 2; background: linear-gradient(135deg, rgba(16,185,129,0.28), rgba(6,182,212,0.22)); border: 2px solid var(--accent-emerald); box-shadow: 0 0 18px rgba(16,185,129,0.35);">
            <div class="action-title" style="color: #6ee7b7; font-size: 0.98rem; font-weight: 900;"><span>🟢 1. START / INITIALIZE STAND</span><span>▶</span></div>
            <div class="action-desc" style="color: #a7f3d0; font-weight: 700;">Press to initialize robot in Standard Stand position [94°, 8°, 94° | 94°, 94°, 94°]</div>
          </div>
          <div class="action-card act-walk" onclick="runGait('walk')">
            <div class="action-title"><span>🚀 2. Dynamic Walk</span><span>▶</span></div>
            <div class="action-desc">Continuous fluid biped walking (coordinated reference IK)</div>
          </div>
          <div class="action-card act-step" onclick="runGait('step')">
            <div class="action-title"><span>🐾 3. Single Stride</span><span>▶</span></div>
            <div class="action-desc">One complete forward step cycle to neutral</div>
          </div>
          <div class="action-card act-ref" onclick="runGait('ref_step')">
            <div class="action-title"><span>📐 4. Ref takeStep(2,0)</span><span>▶</span></div>
            <div class="action-desc">Discrete calibrated IK step sequence</div>
          </div>
          <div class="action-card act-test" onclick="runGait('test')">
            <div class="action-title"><span>🧪 5. Diagnostics</span><span>▶</span></div>
            <div class="action-desc">Sequential safety sweep of all 6 servos</div>
          </div>
          <div class="action-card act-stop" onclick="runGait('stand')">
            <div class="action-title"><span>🛑 6. EMERGENCY STOP & PARK STAND</span><span>⏹</span></div>
            <div class="action-desc">Instantly locks both legs to 100% constant calibrated neutral</div>
          </div>
        </div>
      </div>
    </section>

    <!-- Real-Time Joint Telemetry Strip -->
    <section class="card">
      <div class="card-header">
        <div class="card-title"><span>📡</span><span>REAL-TIME 6-DOF TELEMETRY</span></div>
      </div>
      <div class="telemetry-grid">
        <div class="telem-card" onclick="selectJoint(0, 'Right Hip')">
          <span class="telem-name">R.Hip (0)</span>
          <span class="telem-angle telem-r" id="track-0">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-0" style="background: var(--accent-amber);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(1, 'Right Knee')">
          <span class="telem-name">R.Knee (1)</span>
          <span class="telem-angle telem-r" id="track-1">8°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-1" style="background: var(--accent-amber);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(12, 'Right Foot')">
          <span class="telem-name">R.Foot (12)</span>
          <span class="telem-angle telem-r" id="track-12">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-12" style="background: var(--accent-amber);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(4, 'Left Hip')">
          <span class="telem-name">L.Hip (4)</span>
          <span class="telem-angle telem-l" id="track-4">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-4" style="background: var(--accent-cyan);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(8, 'Left Knee')">
          <span class="telem-name">L.Knee (8)</span>
          <span class="telem-angle telem-l" id="track-8">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-8" style="background: var(--accent-cyan);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(15, 'Left Foot')">
          <span class="telem-name">L.Foot (15)</span>
          <span class="telem-angle telem-l" id="track-15">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-15" style="background: var(--accent-cyan);"></div></div>
        </div>
      </div>
    </section>

    <!-- Real-Time Ground Reaction Force & Weight Distribution HUD Card -->
    <section class="card">
      <div class="card-header">
        <div class="card-title">
          <span>⚖️</span>
          <span>GROUND REACTION FORCE & WEIGHT DISTRIBUTION (WALKING)</span>
        </div>
        <div class="hud-badge" style="color: var(--accent-emerald);">
          <span id="hud-contact-state">DUAL STANCE</span>
        </div>
      </div>
      <div class="weight-grid">
        <!-- Left Foot Ground Weight -->
        <div class="weight-card weight-card-l">
          <div class="weight-title-row">
            <span class="weight-title" style="color: var(--accent-cyan);">Left Foot Ground Load</span>
            <span class="hud-badge" id="weight-pct-l" style="color: var(--accent-cyan);">50%</span>
          </div>
          <div style="display: flex; justify-content: space-between; align-items: baseline;">
            <span class="weight-val" id="weight-val-l" style="color: var(--accent-cyan);">250g</span>
            <span style="font-size: 0.72rem; color: var(--text-muted);">Max: 500g</span>
          </div>
          <div class="weight-bar-bg">
            <div class="weight-bar-fill" id="weight-bar-l" style="width: 50%; background: linear-gradient(90deg, #0284c7, #06b6d4);"></div>
          </div>
        </div>

        <!-- Right Foot Ground Weight -->
        <div class="weight-card weight-card-r">
          <div class="weight-title-row">
            <span class="weight-title" style="color: var(--accent-amber);">Right Foot Ground Load</span>
            <span class="hud-badge" id="weight-pct-r" style="color: var(--accent-amber);">50%</span>
          </div>
          <div style="display: flex; justify-content: space-between; align-items: baseline;">
            <span class="weight-val" id="weight-val-r" style="color: var(--accent-amber);">250g</span>
            <span style="font-size: 0.72rem; color: var(--text-muted);">Max: 500g</span>
          </div>
          <div class="weight-bar-bg">
            <div class="weight-bar-fill" id="weight-bar-r" style="width: 50%; background: linear-gradient(90deg, #d97706, #f59e0b);"></div>
          </div>
        </div>
      </div>

      <div class="grf-summary-bar">
        <div style="display: flex; align-items: center; gap: 8px;">
          <span style="font-size: 0.75rem; color: var(--text-muted);">TOTAL DYNAMIC FORCE:</span>
          <span style="font-size: 1.1rem; font-weight: 900; color: #fff; font-family: ui-monospace, monospace;" id="hud-force-n">4.91 N</span>
        </div>
        <div style="display: flex; align-items: center; gap: 8px;">
          <span style="font-size: 0.75rem; color: var(--text-muted);">TOTAL ROBOT MASS:</span>
          <span style="font-size: 0.95rem; font-weight: 800; color: var(--accent-sky); font-family: ui-monospace, monospace;">500 g</span>
        </div>
        <div style="display: flex; align-items: center; gap: 8px;">
          <span style="font-size: 0.75rem; color: var(--text-muted);">SUPPORT STABILITY:</span>
          <span style="font-size: 0.75rem; font-weight: 800; color: var(--accent-emerald);" id="hud-zmp-status">STABLE (ZMP CENTERED)</span>
        </div>
      </div>
    </section>

    <!-- Right Leg Joint Controls -->
    <div class="leg-section-title leg-r-header">
      <span>RIGHT LEG JOINTS (SAGITTAL PLANE)</span>
      <span style="font-size: 0.72rem; font-weight: 800;">CH 0: Hip (94°) | CH 1: Knee (8°) | CH 12: Foot (94°)</span>
    </div>
    <div class="servo-grid">
      <!-- CH 0 -->
      <div class="servo-card servo-card-r" id="card-0">
        <div class="servo-header">
          <span class="servo-name" style="color: var(--accent-amber);">Right Hip (CH 0)</span>
          <div class="servo-angle-box">
            <span class="servo-angle-text" id="angle-0">94°</span>
            <span class="servo-home-tag">(Home: 94°)</span>
          </div>
        </div>
        <input type="range" class="slider-styled" id="slider-0" min="0" max="180" value="94" oninput="setAngle(0, this.value)" onchange="setAngle(0, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(0, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(0, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(0, -1)">-1</button>
          <button class="step-btn btn-reset-single" onclick="resetSingleJoint(0, 94)">94°</button>
          <button class="step-btn" onclick="stepAngle(0, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(0, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(0, 10)">+10</button>
        </div>
      </div>

      <!-- CH 1 -->
      <div class="servo-card servo-card-r" id="card-1">
        <div class="servo-header">
          <span class="servo-name" style="color: var(--accent-amber);">Right Knee (CH 1)</span>
          <div class="servo-angle-box">
            <span class="servo-angle-text" id="angle-1">8°</span>
            <span class="servo-home-tag">(Home: 8°)</span>
          </div>
        </div>
        <input type="range" class="slider-styled" id="slider-1" min="0" max="180" value="8" oninput="setAngle(1, this.value)" onchange="setAngle(1, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(1, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(1, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(1, -1)">-1</button>
          <button class="step-btn btn-reset-single" onclick="resetSingleJoint(1, 8)">8°</button>
          <button class="step-btn" onclick="stepAngle(1, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(1, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(1, 10)">+10</button>
        </div>
      </div>

      <!-- CH 12 -->
      <div class="servo-card servo-card-r" id="card-12">
        <div class="servo-header">
          <span class="servo-name" style="color: var(--accent-amber);">Right Foot (CH 12)</span>
          <div class="servo-angle-box">
            <span class="servo-angle-text" id="angle-12">94°</span>
            <span class="servo-home-tag">(Home: 94°)</span>
          </div>
        </div>
        <input type="range" class="slider-styled" id="slider-12" min="0" max="180" value="94" oninput="setAngle(12, this.value)" onchange="setAngle(12, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(12, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(12, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(12, -1)">-1</button>
          <button class="step-btn btn-reset-single" onclick="resetSingleJoint(12, 94)">94°</button>
          <button class="step-btn" onclick="stepAngle(12, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(12, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(12, 10)">+10</button>
        </div>
      </div>
    </div>

    <!-- Left Leg Joint Controls -->
    <div class="leg-section-title leg-l-header">
      <span>LEFT LEG JOINTS (SAGITTAL PLANE - MIRRORED)</span>
      <span style="font-size: 0.72rem; font-weight: 800;">CH 4: Hip (94°) | CH 8: Knee (94°) | CH 15: Foot (94°)</span>
    </div>
    <div class="servo-grid">
      <!-- CH 4 -->
      <div class="servo-card" id="card-4">
        <div class="servo-header">
          <span class="servo-name" style="color: var(--accent-cyan);">Left Hip (CH 4)</span>
          <div class="servo-angle-box">
            <span class="servo-angle-text" id="angle-4">94°</span>
            <span class="servo-home-tag">(Home: 94°)</span>
          </div>
        </div>
        <input type="range" class="slider-styled" id="slider-4" min="0" max="180" value="94" oninput="setAngle(4, this.value)" onchange="setAngle(4, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(4, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(4, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(4, -1)">-1</button>
          <button class="step-btn btn-reset-single" onclick="resetSingleJoint(4, 94)">94°</button>
          <button class="step-btn" onclick="stepAngle(4, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(4, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(4, 10)">+10</button>
        </div>
      </div>

      <!-- CH 8 -->
      <div class="servo-card" id="card-8">
        <div class="servo-header">
          <span class="servo-name" style="color: var(--accent-cyan);">Left Knee (CH 8)</span>
          <div class="servo-angle-box">
            <span class="servo-angle-text" id="angle-8">94°</span>
            <span class="servo-home-tag">(Home: 94°)</span>
          </div>
        </div>
        <input type="range" class="slider-styled" id="slider-8" min="0" max="180" value="94" oninput="setAngle(8, this.value)" onchange="setAngle(8, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(8, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(8, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(8, -1)">-1</button>
          <button class="step-btn btn-reset-single" onclick="resetSingleJoint(8, 94)">94°</button>
          <button class="step-btn" onclick="stepAngle(8, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(8, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(8, 10)">+10</button>
        </div>
      </div>

      <!-- CH 15 -->
      <div class="servo-card" id="card-15">
        <div class="servo-header">
          <span class="servo-name" style="color: var(--accent-cyan);">Left Foot (CH 15)</span>
          <div class="servo-angle-box">
            <span class="servo-angle-text" id="angle-15">94°</span>
            <span class="servo-home-tag">(Home: 94°)</span>
          </div>
        </div>
        <input type="range" class="slider-styled" id="slider-15" min="0" max="180" value="94" oninput="setAngle(15, this.value)" onchange="setAngle(15, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(15, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(15, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(15, -1)">-1</button>
          <button class="step-btn btn-reset-single" onclick="resetSingleJoint(15, 94)">94°</button>
          <button class="step-btn" onclick="stepAngle(15, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(15, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(15, 10)">+10</button>
        </div>
      </div>
    </div>

    <!-- Dynamic Gait Physics Tuner -->
    <section class="card">
      <div class="card-header">
        <div class="card-title"><span>📐</span><span>DYNAMIC GAIT PHYSICS & INVERSE KINEMATICS</span></div>
      </div>
      <div class="tuner-grid">
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Cadence (Speed)</span><span class="tuner-val" id="val-freq">0.65 Hz</span></div>
          <input type="range" class="slider-styled" id="slider-freq" min="3" max="14" value="6.5" step="0.5" oninput="setGaitParam('freq', this.value / 10.0)">
        </div>
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Stride Swing Amp</span><span class="tuner-val" id="val-stride">6.0°</span></div>
          <input type="range" class="slider-styled" id="slider-stride" min="2.0" max="10.0" value="6.0" step="0.5" oninput="setGaitParam('stride', this.value)">
        </div>
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Knee Lift Clearance</span><span class="tuner-val" id="val-knee">7.0°</span></div>
          <input type="range" class="slider-styled" id="slider-knee" min="3.0" max="12.0" value="7.0" step="0.5" oninput="setGaitParam('knee', this.value)">
        </div>
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Stance Ground Push</span><span class="tuner-val" id="val-push">2.0°</span></div>
          <input type="range" class="slider-styled" id="slider-push" min="1.0" max="5.0" value="2.0" step="0.5" oninput="setGaitParam('push', this.value)">
        </div>
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Foot Level Flexion</span><span class="tuner-val" id="val-flex">2.2°</span></div>
          <input type="range" class="slider-styled" id="slider-flex" min="0.5" max="5.0" value="2.2" step="0.2" oninput="setGaitParam('foot', this.value)">
        </div>
      </div>
    </section>

    <!-- System Actions & Non-Volatile Storage -->
    <section class="card">
      <div class="card-header">
        <div class="card-title"><span>💾</span><span>NON-VOLATILE STORAGE & SYSTEM HARDWARE</span></div>
      </div>
      <div class="sys-actions">
        <button class="btn-sys btn-save" onclick="saveOffsets()"><span>💾</span><span>Save Offsets To EEPROM</span></button>
        <button class="btn-sys btn-restore" onclick="resetOffsets()"><span>🔄</span><span>Restore Factory Baselines</span></button>
        <button class="btn-sys btn-scan" onclick="scanI2C()"><span>🔍</span><span>Scan I2C Bus Devices</span></button>
      </div>
    </section>
  </div>

  <!-- Non-intrusive Toast Notification -->
  <div id="toast"><span>✓</span><span id="toast-text">Operation Successful</span></div>

  <script>
    let sendTimers = {};
    let selectedJointChannel = 0;
    let selectedJointName = 'Right Hip';
    const DEFAULT_ANGLES = { 0: 94, 1: 8, 12: 94, 4: 94, 8: 94, 15: 94 };

    function showToast(msg, duration = 2500) {
      const t = document.getElementById('toast');
      const txt = document.getElementById('toast-text');
      if (t && txt) {
        txt.innerText = msg;
        t.classList.add('show');
        clearTimeout(t.timer);
        t.timer = setTimeout(() => { t.classList.remove('show'); }, duration);
      }
    }

    function sendCmd(url, msg = null) {
      fetch(url)
        .then(r => r.json())
        .then(d => {
          if (msg) showToast(msg);
        })
        .catch(() => {
          if (msg) showToast(msg);
        });
    }

    function switchSchematic(tab) {
      const kSvg = document.getElementById('svg-kinematic');
      const cSvg = document.getElementById('svg-circuit');
      const kBtn = document.getElementById('tab-btn-kinematic');
      const cBtn = document.getElementById('tab-btn-circuit');
      if (tab === 'kinematic') {
        kSvg.style.display = 'block'; cSvg.style.display = 'none';
        kBtn.classList.add('active'); cBtn.classList.remove('active');
      } else {
        kSvg.style.display = 'none'; cSvg.style.display = 'block';
        cBtn.classList.add('active'); kBtn.classList.remove('active');
      }
    }

    function selectJoint(ch, name) {
      selectedJointChannel = ch;
      selectedJointName = name;
      document.getElementById('selected-joint-badge').innerText = name + ' (CH ' + ch + ')';

      const slider = document.getElementById('slider-' + ch);
      if (slider) {
        document.getElementById('selected-joint-angle').innerText = slider.value + '°';
      }

      [0, 1, 12, 4, 8, 15].forEach(c => {
        const node = document.getElementById('node-' + c);
        if (node) {
          if (c === ch) node.classList.add('selected');
          else node.classList.remove('selected');
        }
        const card = document.getElementById('card-' + c);
        if (card) {
          if (c === ch) card.classList.add('active-target');
          else card.classList.remove('active-target');
        }
      });
    }

    function stepSelected(delta) {
      stepAngle(selectedJointChannel, delta);
    }

    function homeSelected() {
      const home = DEFAULT_ANGLES[selectedJointChannel] || 94;
      setAngle(selectedJointChannel, home);
      showToast('Reset CH ' + selectedJointChannel + ' to ' + home + '°');
    }

    function resetSingleJoint(ch, homeAngle) {
      setAngle(ch, homeAngle);
      showToast('Reset CH ' + ch + ' to ' + homeAngle + '°');
    }

    function setAngle(ch, val) {
      val = parseInt(val);
      updateUI(ch, val);
      if (ch === selectedJointChannel) {
        document.getElementById('selected-joint-angle').innerText = val + '°';
      }
      clearTimeout(sendTimers[ch]);
      sendTimers[ch] = setTimeout(() => {
        sendCmd('/set_servo?ch=' + ch + '&val=' + val);
      }, 20);
    }

    function updateUI(ch, val) {
      const el = document.getElementById('angle-' + ch);
      const sl = document.getElementById('slider-' + ch);
      const tr = document.getElementById('track-' + ch);
      const br = document.getElementById('bar-' + ch);
      const svgA = document.getElementById('svg-angle-' + ch);

      if (el) el.innerText = val + '°';
      if (sl) sl.value = val;
      if (tr) tr.innerText = val + '°';
      if (br) br.style.width = ((val / 180) * 100) + '%';
      if (svgA) svgA.textContent = val + '°';

      if (ch === selectedJointChannel) {
        const sja = document.getElementById('selected-joint-angle');
        if (sja) sja.innerText = val + '°';
      }
    }

    function stepAngle(ch, delta) {
      const slider = document.getElementById('slider-' + ch);
      if (slider) {
        let val = parseInt(slider.value) + delta;
        if (val < 0) val = 0;
        if (val > 180) val = 180;
        setAngle(ch, val);
      }
    }

    function runGait(gait) {
      const msgs = {
        'start': '🟢 Robot Initialized in Standard Stand [94°, 8°, 94° | 94°, 94°, 94°]',
        'walk': '🚀 Continuous Dynamic Walk Started',
        'step': '🐾 Single Full Stride Executing',
        'stand': '🛑 Robot Parked at Constant Neutral',
        'ref_step': '📐 Reference takeStep(2, 0) Executing',
        'ref_init': '⚙️ Reference initialize() Executing',
        'test': '🧪 Running Joint Diagnostics Sweep',
        'left': '◀ Turning Left',
        'right': '▶ Turning Right',
        'backward': '▼ Walking Backward'
      };
      sendCmd('/run_gait?gait=' + gait, msgs[gait] || ('Gait: ' + gait));
    }

    function setGaitParam(param, val) {
      val = parseFloat(val);
      if (param === 'freq') {
        const el = document.getElementById('val-freq');
        if (el) el.innerText = val.toFixed(2) + ' Hz';
      } else if (param === 'stride') {
        const el = document.getElementById('val-stride');
        if (el) el.innerText = val.toFixed(1) + '°';
      } else if (param === 'knee') {
        const el = document.getElementById('val-knee');
        if (el) el.innerText = val.toFixed(1) + '°';
      } else if (param === 'push') {
        const el = document.getElementById('val-push');
        if (el) el.innerText = val.toFixed(1) + '°';
      } else if (param === 'foot') {
        const el = document.getElementById('val-flex');
        if (el) el.innerText = val.toFixed(1) + '°';
      }
      sendCmd('/set_gait_param?param=' + param + '&val=' + val);
    }

    function saveOffsets() {
      sendCmd('/save_offsets', '💾 All 6 Joint Angles Saved Permanently to Flash!');
    }

    function resetOffsets() {
      sendCmd('/reset_offsets', '🔄 Restored Factory Baselines: Right [94°, 8°, 94°] | Left [94°, 94°, 94°]');
      setTimeout(pollTelemetry, 300);
    }

    function scanI2C() {
      fetch('/scan_i2c')
        .then(r => r.json())
        .then(d => {
          if (d.devices && d.devices.length > 0) {
            showToast('✓ I2C Bus Devices Found: ' + d.devices.join(', '));
          } else {
            showToast('⚠️ No I2C Devices Found on SDA 21 / SCL 22');
          }
        }).catch(() => { showToast('I2C Scan Completed'); });
    }

    function pollTelemetry() {
      fetch('/get_angles')
        .then(r => r.json())
        .then(d => {
          if (d.angles) {
            for (const ch in d.angles) {
              const val = d.angles[ch];
              updateUI(ch, val);
            }
          }
          if (d.active_gait) {
            const ag = document.getElementById('svg-active-gait');
            if (ag) ag.textContent = d.active_gait.toUpperCase();
            const hag = document.getElementById('hud-active-gait');
            if (hag) hag.textContent = d.active_gait.toUpperCase();
          }
          if (d.weight_l !== undefined) {
            const wl = Math.round(d.weight_l);
            const elWl = document.getElementById('weight-val-l');
            if (elWl) elWl.innerText = wl + 'g';
            const pctL = Math.min(100, Math.round((d.weight_l / 500) * 100));
            const elPctL = document.getElementById('weight-pct-l');
            if (elPctL) elPctL.innerText = pctL + '%';
            const elBarL = document.getElementById('weight-bar-l');
            if (elBarL) elBarL.style.width = pctL + '%';
            const pfL = document.getElementById('plate-foot-l');
            if (pfL) pfL.setAttribute('opacity', (0.35 + (pctL / 100) * 0.65).toFixed(2));
          }
          if (d.weight_r !== undefined) {
            const wr = Math.round(d.weight_r);
            const elWr = document.getElementById('weight-val-r');
            if (elWr) elWr.innerText = wr + 'g';
            const pctR = Math.min(100, Math.round((d.weight_r / 500) * 100));
            const elPctR = document.getElementById('weight-pct-r');
            if (elPctR) elPctR.innerText = pctR + '%';
            const elBarR = document.getElementById('weight-bar-r');
            if (elBarR) elBarR.style.width = pctR + '%';
            const pfR = document.getElementById('plate-foot-r');
            if (pfR) pfR.setAttribute('opacity', (0.35 + (pctR / 100) * 0.65).toFixed(2));
          }
          if (d.force_n !== undefined) {
            const elF = document.getElementById('hud-force-n');
            if (elF) elF.innerText = d.force_n.toFixed(2) + ' N';
          }
          if (d.contact) {
            const elC = document.getElementById('hud-contact-state');
            if (elC) elC.innerText = d.contact;
          }
        }).catch(()=>{});
    }

    setInterval(pollTelemetry, 350);
    pollTelemetry();
    selectJoint(0, 'Right Hip');
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 4. Motion Engine & Anti-Jitter Pulse Driver
// ==========================================
int getServoIndex(int channel) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].channel == channel) return i;
  }
  return -1;
}

bool initPCA9685() {
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() == 0) {
    pwm.begin();
    pwm.setPWMFreq(50);
    delay(20);
    pcaFound = true;
    for (int i = 0; i < NUM_SERVOS; i++) servos[i].lastPulseLen = -1;
    Serial.println("[SUCCESS] PCA9685 detected and initialized at 0x40 (50Hz)!");
    return true;
  }
  return false;
}

void setServoAngleDirect(int channel, float angle) {
  if (!pcaFound) {
    if (!initPCA9685()) return;
  }
  angle = constrain(angle, 0.0f, 180.0f);

  float us = SERVOMIN_US + (angle / 180.0f) * (SERVOMAX_US - SERVOMIN_US);
  int pulselen = (int)round((us * 4096.0f) / 20000.0f);

  int idx = getServoIndex(channel);
  if (idx != -1) {
    if (servos[idx].lastPulseLen != pulselen) {
      servos[idx].lastPulseLen = pulselen;
      pwm.setPWM(channel, 0, pulselen);
    }
  } else {
    pwm.setPWM(channel, 0, pulselen);
  }
}

// ==========================================
// 4. Human-Like Dynamic Locomotion Engine
// ==========================================
void parkStand() {
  gaitRunning = false;
  activeGaitName = "stand";
  currentGaitScale = 0.0f;
  weightLeftGrams    = 250.0f;
  weightRightGrams   = 250.0f;
  groundForceNewtons = 4.905f;
  groundContactState = "DUAL STANCE";
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = (float)servos[i].homeAngle;
    setServoAngleDirect(servos[i].channel, (float)servos[i].homeAngle);
  }
}

void updateHumanLikeWalk() {
  float t = (float)(millis() - walkStartTime) / 1000.0f;

  // 1. Smooth Anti-Jerk Envelope (eases smoothly into motion)
  if (gaitRunning) {
    if (currentGaitScale < 1.0f) currentGaitScale = min(1.0f, currentGaitScale + 0.04f);
  } else {
    if (currentGaitScale > 0.0f) currentGaitScale = max(0.0f, currentGaitScale - 0.05f);
  }

  // Continuous Harmonic Phase (0 to 2*PI)
  float phase = fmod(2.0f * (float)M_PI * walkFreq * t, 2.0f * (float)M_PI);
  if (phase < 0.0f) phase += 2.0f * (float)M_PI;

  float sinP = sin(phase);
  float cosP = cos(phase);

  // 1. Cartesian Gait Parameters (cm)
  // Continuous harmonic biped locomotion: both legs actively stride and propel
  float xCenter   = 0.25f * currentGaitScale; // Forward bias keeps feet safely out of rear pelvis
  float xAmp      = 0.35f * currentGaitScale; // Symmetrical stride half-amplitude (3.5mm)
  float zLift     = min(0.30f, stepClearance * 0.30f) * currentGaitScale; // Foot clearance lift (3.0mm)
  float curHeight = stepHeight;               // 10.0cm

  // Continuous symmetrical horizontal stride:
  // Right foot swings forward while Left foot drives table backward (propelling robot forward)
  float xr = xCenter - (xAmp * cosP);
  float xl = xCenter + (xAmp * cosP);

  float zr, zl;
  if (sinP >= 0.0f) {
    // Phase 1: Right Leg swings in air, Left Leg stance on ground
    zr = curHeight - (zLift * sinP * sinP);
    zl = curHeight;
  } else {
    // Phase 2: Left Leg swings in air, Right Leg stance on ground
    zr = curHeight;
    zl = curHeight - (zLift * sinP * sinP);
  }

  // Real-Time Dynamic Ground Reaction Force & Weight Calculation
  if (currentGaitScale > 0.08f) {
    if (sinP > 0.08f) {
      // Phase 1: Right leg swing (air), Left leg stance on ground
      float liftFraction = min(1.0f, sinP * 1.6f);
      weightRightGrams = max(0.0f, 250.0f * (1.0f - liftFraction));
      float dynamicAccel = 1.0f + 0.15f * sinP;
      weightLeftGrams = min(600.0f, (robotTotalWeightGrams - weightRightGrams) * dynamicAccel);
      groundContactState = (sinP > 0.25f) ? "LEFT SINGLE SUPPORT" : "WEIGHT TRANSFER";
    } else if (sinP < -0.08f) {
      // Phase 2: Left leg swing (air), Right leg stance on ground
      float liftFraction = min(1.0f, (-sinP) * 1.6f);
      weightLeftGrams = max(0.0f, 250.0f * (1.0f - liftFraction));
      float dynamicAccel = 1.0f + 0.15f * (-sinP);
      weightRightGrams = min(600.0f, (robotTotalWeightGrams - weightLeftGrams) * dynamicAccel);
      groundContactState = (-sinP > 0.25f) ? "RIGHT SINGLE SUPPORT" : "WEIGHT TRANSFER";
    } else {
      weightLeftGrams = 250.0f;
      weightRightGrams = 250.0f;
      groundContactState = "DUAL STANCE";
    }
  } else {
    weightLeftGrams = 250.0f;
    weightRightGrams = 250.0f;
    groundContactState = "DUAL STANCE";
  }
  groundForceNewtons = (weightLeftGrams + weightRightGrams) * 0.00981f;

  // 2. Solve True Inverse Kinematics for Both Legs in Real Time
  IKResult ikR   = solveIK(xr, zr);
  IKResult ikL   = solveIK(xl, zl);
  IKResult ikNom = solveIK(0.0f, curHeight);

  // Joint angle deltas relative to standing posture
  float dHr = ikR.hipDeg - ikNom.hipDeg;
  float dKr = ikR.kneeDeg - ikNom.kneeDeg;

  float dHl = ikL.hipDeg - ikNom.hipDeg;
  float dKl = ikL.kneeDeg - ikNom.kneeDeg;

  // 3. Dynamic Foot Leveling (maintains sole parallel to floor during swing)
  float footAdjR = 0.30f * dHr;
  float footAdjL = 0.30f * dHl;

  // 4. Map to Physical Servos
  // Right Leg (CH 0 Hip 94°, CH 1 Knee 8°, CH 12 Foot 94°)
  float aHipR  = (float)servos[0].homeAngle + dHr;
  float aKneeR = (float)servos[1].homeAngle + dKr;
  float aFootR = (float)servos[2].homeAngle - footAdjR;

  // Left Leg (CH 4 Hip 94°, CH 8 Knee 94°, CH 15 Foot 94° - Mirrored mounting)
  float aHipL  = (float)servos[3].homeAngle - dHl;
  float aKneeL = (float)servos[4].homeAngle - dKl;
  float aFootL = (float)servos[5].homeAngle - footAdjL;

  // Strict Physical Safety Constraints (Guaranteed Non-Jamming, Rear Chassis Lockout & Anti-Knee Collision)
  // Right Leg: Hip 92.5° to 101.5°; Knee 7.5° to 17.5°; Foot 91.5° to 94.5°
  aHipR  = constrain(aHipR,  92.5f, 101.5f);
  aKneeR = constrain(aKneeR,   7.5f,  17.5f);
  aFootR = constrain(aFootR,  91.5f,  94.5f);

  // Left Leg: Hip 86.5° to 95.5°; Knee 84.5° to 94.5°; Foot 91.5° to 94.5°
  aHipL  = constrain(aHipL,  86.5f,  95.5f);
  aKneeL = constrain(aKneeL,  84.5f,  94.5f);
  aFootL = constrain(aFootL,  91.5f,  94.5f);

  // Apply directly to PWM servo driver
  servos[0].currentAngle = aHipR;  setServoAngleDirect(0,  aHipR);  // CH 0 Right Hip
  servos[1].currentAngle = aKneeR; setServoAngleDirect(1,  aKneeR); // CH 1 Right Knee
  servos[2].currentAngle = aFootR; setServoAngleDirect(12, aFootR); // CH 12 Right Foot
  servos[3].currentAngle = aHipL;  setServoAngleDirect(4,  aHipL);  // CH 4 Left Hip
  servos[4].currentAngle = aKneeL; setServoAngleDirect(8,  aKneeL); // CH 8 Left Knee
  servos[5].currentAngle = aFootL; setServoAngleDirect(15, aFootL); // CH 15 Left Foot
}

void executeSingleHumanStride() {
  parkStand();
  delay(300);
  gaitRunning = true;
  activeGaitName = "step";
  currentGaitScale = 1.0f; // Instant full scale for a crisp, complete single stride!
  unsigned long t0 = millis();
  float periodMs = 1000.0f / walkFreq;
  walkStartTime = t0;
  while (millis() - t0 < (unsigned long)periodMs) {
    updateHumanLikeWalk();
    delay(15);
    server.handleClient();
  }
  parkStand();
}

void executeJointDiagnostics() {
  parkStand();
  delay(400);

  // 1. Right Hip (CH 0): 94° -> 100.0° -> 94° (Smooth forward swing)
  setServoAngleDirect(0, 100.0f); servos[0].currentAngle = 100.0f; delay(1000);
  setServoAngleDirect(0, 94.0f);  servos[0].currentAngle = 94.0f;  delay(500);

  // 2. Right Knee (CH 1): 8° -> 15.0° -> 8° (Gentle micro-lift, zero collision)
  setServoAngleDirect(1, 15.0f); servos[1].currentAngle = 15.0f; delay(1000);
  setServoAngleDirect(1, 8.0f);  servos[1].currentAngle = 8.0f;  delay(500);

  // 3. Right Foot (CH 12): 94° -> 92.0° -> 94° (Gentle leveling flex)
  setServoAngleDirect(12, 92.0f); servos[2].currentAngle = 92.0f; delay(1000);
  setServoAngleDirect(12, 94.0f); servos[2].currentAngle = 94.0f; delay(500);

  // 4. Left Hip (CH 4): 94° -> 88.0° -> 94° (Smooth mirrored forward swing)
  setServoAngleDirect(4, 88.0f); servos[3].currentAngle = 88.0f; delay(1000);
  setServoAngleDirect(4, 94.0f); servos[3].currentAngle = 94.0f; delay(500);

  // 5. Left Knee (CH 8): 94° -> 87.0° -> 94° (Gentle mirrored micro-lift, zero collision)
  setServoAngleDirect(8, 87.0f); servos[4].currentAngle = 87.0f; delay(1000);
  setServoAngleDirect(8, 94.0f); servos[4].currentAngle = 94.0f; delay(500);

  // 6. Left Foot (CH 15): 94° -> 92.0° -> 94° (Gentle leveling flex)
  setServoAngleDirect(15, 92.0f); servos[5].currentAngle = 92.0f; delay(1000);
  setServoAngleDirect(15, 94.0f); servos[5].currentAngle = 94.0f; delay(500);

  parkStand();
}

// ==========================================
// 4B. User Reference IK Locomotion Engine
// Direct integration of user reference functions:
// - ServoProxy (hipL, hipR, kneeL, kneeR, ankleL, ankleR)
// - updateServoPos(target1, target2, target3, leg)
// - pos(x, z, leg)
// - takeStep(stepLength, stepVelocity)
// - initialize()
// ==========================================
struct ServoProxy {
  int channel;
  int servoIndex;
  float minLimit;
  float maxLimit;
  void write(int angle) {
    write((float)angle);
  }
  void write(float angle) {
    // Physical bracket protection: clamp to safe non-jamming range
    angle = constrain(angle, minLimit, maxLimit);
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
      servos[servoIndex].currentAngle = angle;
    }
    setServoAngleDirect(channel, angle);
  }
  void attach(int pin) {}
};

ServoProxy hipL   = {4,  3, 86.5f,  95.5f}; // 86.5° forward max, 95.5° rear stop (zero pelvis contact)
ServoProxy hipR   = {0,  0, 92.5f, 101.5f}; // 92.5° rear stop, 101.5° forward max (zero pelvis contact)
ServoProxy kneeL  = {8,  4, 84.5f,  94.5f}; // 84.5° max bend (clean clearance from foot), 94.5° extension
ServoProxy kneeR  = {1,  1,  7.5f,  17.5f}; // 7.5° extension, 17.5° max bend (clean clearance from foot)
ServoProxy ankleL = {15, 5, 91.5f,  94.5f}; // 91.5° to 94.5° safe foot articulation (zero knee contact)
ServoProxy ankleR = {12, 2, 91.5f,  94.5f}; // 91.5° to 94.5° safe foot articulation (zero knee contact)

void updateServoPos(float dHip, float dKnee, float dAnkle, char leg){
  float footAdjustment = constrain(0.20f * dKnee, 0.0f, 2.0f); // Gentle 1-2° ankle flexion
  if (leg == 'l'){
    hipL.write((float)hipLOffset - dHip);
    kneeL.write((float)kneeLOffset - dKnee);
    ankleL.write((float)ankleLOffset - footAdjustment); // 94° down to 92° (flexes safely away from knee!)
  }
  else if (leg == 'r'){
    hipR.write((float)hipROffset + dHip);
    kneeR.write((float)kneeROffset + dKnee);
    ankleR.write((float)ankleROffset - footAdjustment); // 94° down to 92° (flexes safely away from knee!)
  }
}

void pos(float x, float z, char leg){
  float hipRad2 = atan(x/z);
  float hipDeg2 = hipRad2 * (180.0f / (float)M_PI);

  float z2 = z/cos(hipRad2);

  float val1 = (sq(l1) + sq(z2) - sq(l2))/(2.0f * l1 * z2);
  val1 = constrain(val1, -1.0f, 1.0f);
  float hipRad1 = acos(val1);
  float hipDeg1 = hipRad1 * (180.0f / (float)M_PI);
  
  float val2 = (sq(l1) + sq(l2) - sq(z2))/(2.0f * l1 * l2);
  val2 = constrain(val2, -1.0f, 1.0f);
  float kneeRad = (float)M_PI - acos(val2);

  float val3 = (sq(l2) + sq(z2) - sq(l1))/(2.0f * l2 * z2);
  val3 = constrain(val3, -1.0f, 1.0f);
  float ankleRad = ((float)M_PI / 2.0f) + hipRad2 - acos(val3);
  
  float hipDeg = hipDeg1 + hipDeg2;
  float kneeDeg = kneeRad * (180.0f / (float)M_PI);
  float ankleDeg = ankleRad * (180.0f / (float)M_PI);

  // Standing reference stance angles at (0, stepHeight)
  float nomVal1 = (sq(l1) + sq(stepHeight) - sq(l2)) / (2.0f * l1 * stepHeight);
  float nomHip = acos(constrain(nomVal1, -1.0f, 1.0f)) * (180.0f / (float)M_PI);
  float nomVal2 = (sq(l1) + sq(l2) - sq(stepHeight)) / (2.0f * l1 * l2);
  float nomKnee = ((float)M_PI - acos(constrain(nomVal2, -1.0f, 1.0f))) * (180.0f / (float)M_PI);
  float nomVal3 = (sq(l2) + sq(stepHeight) - sq(l1)) / (2.0f * l2 * stepHeight);
  float nomAnkle = (((float)M_PI / 2.0f) - acos(constrain(nomVal3, -1.0f, 1.0f))) * (180.0f / (float)M_PI);

  // Correct step angles: true Cartesian deltas relative to calibrated standing posture!
  float dHip   = hipDeg - nomHip;
  float dKnee  = kneeDeg - nomKnee;
  float dAnkle = ankleDeg - nomAnkle;

  updateServoPos(dHip, dKnee, dAnkle, leg);  
}

void takeStep(float sLength, int stepVelocity){
  int delayMs = (stepVelocity > 0) ? stepVelocity : 25; // Judder-free timing: smooth servo trajectory
  float sLift = min(0.45f, stepClearance);
  // Phase 1: Right stance on ground, Left swing through air
  for (float i = sLength; i >= -sLength; i -= 0.15f){
    if (!gaitRunning && activeGaitName != "ref_step" && activeGaitName != "ref_walk") break;
    pos(i, stepHeight, 'r');
    pos(-i, stepHeight - sLift, 'l');
    delay(delayMs);
    server.handleClient();
  }

  // Phase 2: Right swing through air, Left stance on ground
  for (float i = sLength; i >= -sLength; i -= 0.15f){
    if (!gaitRunning && activeGaitName != "ref_step" && activeGaitName != "ref_walk") break;
    pos(-i, stepHeight - sLift, 'r');
    pos(i, stepHeight, 'l');
    delay(delayMs);
    server.handleClient();
  }
}

void initialize(){
  for (float i = 10.7; i >= stepHeight; i-=0.1){
    pos(0, i, 'l');
    pos(0, i, 'r');
    delay(15);
    server.handleClient();
  }
  parkStand();
}

// ==========================================
// 5. Web Handlers
// ==========================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleSetServo() {
  if (!server.hasArg("ch") || !server.hasArg("val")) {
    server.send(400, "application/json", "{\"error\":\"Missing arguments\"}");
    return;
  }
  int ch = server.arg("ch").toInt();
  int val = server.arg("val").toInt();
  int idx = getServoIndex(ch);
  if (idx != -1) {
    gaitRunning = false;
    activeGaitName = "manual";
    servos[idx].targetAngle = val;
    servos[idx].currentAngle = val;
    setServoAngleDirect(ch, val);
  } else {
    setServoAngleDirect(ch, val);
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleGetAngles() {
  String json = "{\"angles\":{";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += "\"" + String(servos[i].channel) + "\":" + String((int)servos[i].currentAngle);
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "},\"active_gait\":\"" + activeGaitName + "\"";
  json += ",\"gait_running\":" + String(gaitRunning ? "true" : "false");
  json += ",\"pca_found\":" + String(pcaFound ? "true" : "false");
  json += ",\"weight_l\":" + String(weightLeftGrams, 1);
  json += ",\"weight_r\":" + String(weightRightGrams, 1);
  json += ",\"force_n\":" + String(groundForceNewtons, 2);
  json += ",\"contact\":\"" + groundContactState + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleScanI2C() {
  String json = "{\"devices\":[";
  int count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (count > 0) json += ",";
      char hexBuf[10];
      snprintf(hexBuf, sizeof(hexBuf), "\"0x%02X\"", addr);
      json += hexBuf;
      count++;
    }
  }
  json += "],\"pca_found\":" + String(pcaFound ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleSaveOffsets() {
  saveCurrentOffsets();
  server.send(200, "application/json", "{\"status\":\"saved\"}");
}

void handleResetOffsets() {
  resetFactoryOffsets();
  parkStand();
  server.send(200, "application/json", "{\"status\":\"reset\"}");
}

void handleRunGait() {
  if (!server.hasArg("gait")) {
    server.send(400, "application/json", "{\"error\":\"Missing gait\"}");
    return;
  }
  String gait = server.arg("gait");
  if (gait == "start" || gait == "stand" || gait == "stop") {
    parkStand();
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"stand\"}");
    return;
  } else if (gait == "walk" || gait == "forward") {
    gaitRunning = true;
    activeGaitName = "walk";
    walkStartTime = millis();
    currentGaitScale = 0.0f; // Smooth ease-in from standing posture!
  } else if (gait == "step") {
    gaitRunning = false;
    activeGaitName = "step";
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"step\"}");
    executeSingleHumanStride();
    return;
  } else if (gait == "ref_step" || gait == "takestep") {
    gaitRunning = false;
    activeGaitName = "ref_step";
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"ref_step\"}");
    takeStep(2, 0);
    parkStand();
    return;
  } else if (gait == "ref_walk") {
    gaitRunning = true;
    activeGaitName = "ref_walk";
  } else if (gait == "ref_init" || gait == "initialize") {
    gaitRunning = false;
    activeGaitName = "ref_init";
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"ref_init\"}");
    initialize();
    return;
  } else if (gait == "test" || gait == "test_joints") {
    gaitRunning = false;
    activeGaitName = "test";
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"test\"}");
    executeJointDiagnostics();
    return;
  }
  server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"" + gait + "\"}");
}

void handleSetGaitParam() {
  if (!server.hasArg("param") || !server.hasArg("val")) {
    server.send(400, "application/json", "{\"error\":\"Missing arguments\"}");
    return;
  }
  String param = server.arg("param");
  float val = server.arg("val").toFloat();
  if (param == "freq" || param == "speed") walkFreq = val;
  else if (param == "stride" || param == "length") stepLength = val;
  else if (param == "knee" || param == "clearance" || param == "lift") stepClearance = val;
  else if (param == "height" || param == "stance" || param == "ankle") stepHeight = val;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ==========================================
// 6. Setup & Loop
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- 6-DOF Biped Robot Controller (Zero-Lag Mode) Starting ---");

  loadSavedOffsets();

  Wire.begin(21, 22);
  Wire.setClock(400000); // 400kHz Fast Mode for ultra-responsive servo I2C and zero web latency
  Wire.setTimeOut(50);

  Serial.println("Scanning I2C bus (SDA=21, SCL=22)...");
  int i2cCount = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found I2C device at 0x%02X\n", a);
      i2cCount++;
    }
  }
  if (i2cCount == 0) Serial.println("  [NOTICE] No I2C response yet on pins 21/22. Auto-reconnect active.");

  if (initPCA9685()) {
    parkStand();
  } else {
    Serial.println("[WARNING] PCA9685 not detected at 0x40 during boot. Auto-reconnect active.");
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Support both local networks - prioritized for user's active PC network (MATHA BSNL FTTH)
  wifiMulti.addAP("MATHA BSNL FTTH", "qwerty09");
  wifiMulti.addAP("MATHA ELECTRONICS_ Dwan", "qwerty09");

  Serial.println("Connecting to WiFi (trying MATHA ELECTRONICS_ Dwan & MATHA BSNL FTTH)...");
  unsigned long startWifi = millis();
  while (wifiMulti.run() != WL_CONNECTED && millis() - startWifi < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Station Connected!");
    Serial.print("Connected SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("Station IP: http://");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Subnet Mask: ");
    Serial.println(WiFi.subnetMask());
  } else {
    Serial.println("\nWiFi Station connection timed out.");
  }

  // Ensure Dual Mode (AP + STA) is strictly active so SoftAP is NEVER disabled
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("SoftAP Active -> SSID: ");
  Serial.print(ap_ssid);
  Serial.print(" | Password: ");
  Serial.print(ap_password);
  Serial.print(" | Access Point URL: http://");
  Serial.println(WiFi.softAPIP());

  if (MDNS.begin("biped")) {
    Serial.println("mDNS responder started: http://biped.local");
  }

  server.on("/", handleRoot);
  server.on("/set_servo", handleSetServo);
  server.on("/get_angles", handleGetAngles);
  server.on("/scan_i2c", handleScanI2C);
  server.on("/save_offsets", handleSaveOffsets);
  server.on("/reset_offsets", handleResetOffsets);
  server.on("/run_gait", handleRunGait);
  server.on("/set_gait_param", handleSetGaitParam);
  server.begin();
  Serial.println("HTTP Web Server Started!");
}

void loop() {
  server.handleClient();

  // USB Serial Command Listener (allows controlling robot via USB cable)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "step") {
      Serial.println("[CMD] Executing Single Full Human Stride...");
      executeSingleHumanStride();
    } else if (cmd == "walk") {
      Serial.println("[CMD] Continuous Dynamic Walk Started.");
      gaitRunning = true;
      activeGaitName = "walk";
      walkStartTime = millis();
    } else if (cmd == "takestep" || cmd == "refstep" || cmd == "ref_step") {
      Serial.println("[CMD] Executing Reference takeStep(2, 0)...");
      activeGaitName = "ref_step";
      takeStep(2, 0);
      parkStand();
    } else if (cmd == "ref_walk") {
      Serial.println("[CMD] Continuous Reference Walk Started.");
      gaitRunning = true;
      activeGaitName = "ref_walk";
    } else if (cmd == "init" || cmd == "initialize") {
      Serial.println("[CMD] Executing Reference initialize()...");
      initialize();
    } else if (cmd == "stand" || cmd == "stop") {
      Serial.println("[CMD] Settling to Stand Neutral.");
      parkStand();
    } else if (cmd == "test") {
      Serial.println("[CMD] Running Joint Diagnostics...");
      executeJointDiagnostics();
    }
  }

  // Auto-reconnect PCA9685 whenever it powers up or reconnects
  static unsigned long lastPcaCheck = 0;
  if (!pcaFound && (millis() - lastPcaCheck > 1500)) {
    lastPcaCheck = millis();
    if (initPCA9685()) {
      parkStand();
    }
  }

  if (gaitRunning) {
    static unsigned long lastGaitUpdate = 0;
    if (millis() - lastGaitUpdate >= 25) { // 40Hz clean servo trajectory, zero blocking delay
      lastGaitUpdate = millis();
      if (activeGaitName == "walk" || activeGaitName == "forward") {
        updateHumanLikeWalk();
      } else if (activeGaitName == "ref_walk") {
        static float walkI = stepLength * 0.6f;
        static int walkPhase = 0;
        float sLift = min(0.45f, stepClearance);
        if (walkPhase == 0) {
          pos(walkI, stepHeight, 'r');
          pos(-walkI, stepHeight - sLift, 'l');
        } else {
          pos(-walkI, stepHeight - sLift, 'r');
          pos(walkI, stepHeight, 'l');
        }
        walkI -= 0.15f;
        if (walkI < -(stepLength * 0.6f)) {
          walkI = stepLength * 0.6f;
          walkPhase = 1 - walkPhase;
        }
      }
    }
  }

  // Periodic network & IP heartbeat to Serial (every 6 seconds)
  static unsigned long lastNetPrint = 0;
  if (millis() - lastNetPrint > 6000) {
    lastNetPrint = millis();
    Serial.printf("[NET STATUS] Station: %s (IP: %s) | SoftAP: %s (IP: %s)\n",
      (WiFi.status() == WL_CONNECTED) ? WiFi.SSID().c_str() : "Disconnected",
      (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString().c_str() : "N/A",
      ap_ssid,
      WiFi.softAPIP().toString().c_str());
  }
}
