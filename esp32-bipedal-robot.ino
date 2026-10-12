#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <math.h>

// ==========================================
// 1. WiFi Configuration & Constants
// ==========================================
const char* ssid = "MATHA BSNL FTTH";
const char* password = "qwerty09";

const char* ap_ssid = "ESP32-Biped-Robot";
const char* ap_password = "12345678";

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
// Right Leg: CH 0 (Hip = 94°), CH 1 (Knee = 8°),  CH 2 (Foot = 94°)
// Left Leg:  CH 4 (Hip = 94°), CH 8 (Knee = 94°), CH 12 (Foot = 94°)
Joint servos[NUM_SERVOS] = {
  {0,  "Right Hip",   94.0f, 94.0f, -1, 94},
  {1,  "Right Knee",   8.0f,  8.0f, -1,  8},
  {2,  "Right Foot",  94.0f, 94.0f, -1, 94},
  {4,  "Left Hip",    94.0f, 94.0f, -1, 94},
  {8,  "Left Knee",   94.0f, 94.0f, -1, 94},
  {12, "Left Foot",   94.0f, 94.0f, -1, 94}
};

String activeGaitName = "stand";
bool gaitRunning = false;

// Micro-Amplitude Dynamic Human Walking Parameters (All 6 joints moving, small angle distance)
float walkFreq = 0.65f;        // 0.65 Hz calm cadence (1.54s per full cycle)
float strideAmp = 4.5f;        // 4.5° Forward hip swing
float kneeAmp = 3.0f;          // 3.0° Micro knee lift flexion (lifts foot ~2mm, 100% cap safe!)
float ankleAmp = 2.0f;         // 2.0° Ankle ground level compensation
float pushHip = 1.0f;          // 1.0° Stance hip push
float pushAnk = 1.5f;          // 1.5° Stance foot push-off
float currentGaitScale = 0.0f; // Smooth anti-jerk startup and stop ramp
unsigned long walkStartTime = 0;

void loadSavedOffsets() {
  preferences.begin("biped_chmap", false);
  servos[0].homeAngle = preferences.getInt("ch0", 94);  // Right Hip
  servos[1].homeAngle = preferences.getInt("ch1", 8);   // Right Knee
  servos[2].homeAngle = preferences.getInt("ch2", 94);  // Right Foot
  servos[3].homeAngle = preferences.getInt("ch4", 94);  // Left Hip
  servos[4].homeAngle = preferences.getInt("ch8", 94);  // Left Knee
  servos[5].homeAngle = preferences.getInt("ch12", 94); // Left Foot
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
  preferences.putInt("ch2", (int)servos[2].currentAngle);
  preferences.putInt("ch4", (int)servos[3].currentAngle);
  preferences.putInt("ch8", (int)servos[4].currentAngle);
  preferences.putInt("ch12", (int)servos[5].currentAngle);
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
  servos[2].homeAngle = 94; // Right Foot
  servos[3].homeAngle = 94; // Left Hip
  servos[4].homeAngle = 94; // Left Knee
  servos[5].homeAngle = 94; // Left Foot
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
  <title>Biped Robot - 6-DOF Mission Control & Schematic Studio</title>
  <style>
    :root {
      --bg-base: #070b14;
      --bg-card: rgba(15, 23, 42, 0.75);
      --bg-card-hover: rgba(30, 41, 59, 0.85);
      --border-card: rgba(56, 189, 248, 0.15);
      --border-card-active: rgba(56, 189, 248, 0.45);
      --accent-cyan: #06b6d4;
      --accent-sky: #38bdf8;
      --accent-indigo: #6366f1;
      --accent-violet: #8b5cf6;
      --accent-amber: #f59e0b;
      --accent-emerald: #10b981;
      --accent-rose: #f43f5e;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --text-dim: #64748b;
      --radius-sm: 8px;
      --radius-md: 14px;
      --radius-lg: 20px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", sans-serif; user-select: none; }
    body {
      background: radial-gradient(circle at 50% 0%, #0d1b33 0%, #070b14 75%);
      color: var(--text-main);
      min-height: 100vh;
      padding: 14px;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    .container { width: 100%; max-width: 900px; display: flex; flex-direction: column; gap: 16px; }

    /* Header */
    header {
      background: var(--bg-card);
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
      border: 1px solid var(--border-card);
      border-radius: var(--radius-lg);
      padding: 14px 20px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      box-shadow: 0 10px 30px rgba(0,0,0,0.4);
    }
    .logo-container { display: flex; align-items: center; gap: 12px; }
    .logo-icon { width: 36px; height: 36px; border-radius: 10px; background: linear-gradient(135deg, var(--accent-cyan), var(--accent-indigo)); display: flex; align-items: center; justify-content: center; font-size: 1.2rem; box-shadow: 0 0 15px rgba(6, 182, 212, 0.4); }
    .logo-title { font-size: 1.15rem; font-weight: 800; letter-spacing: 0.5px; background: linear-gradient(135deg, #38bdf8, #818cf8, #c084fc); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
    .logo-subtitle { font-size: 0.7rem; color: var(--text-muted); font-weight: 600; text-transform: uppercase; letter-spacing: 1px; }
    .status-group { display: flex; align-items: center; gap: 8px; }
    .status-badge { display: flex; align-items: center; gap: 6px; font-size: 0.75rem; font-weight: 700; background: rgba(0,0,0,0.35); padding: 6px 12px; border-radius: 20px; border: 1px solid var(--border-card); }
    .status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--accent-emerald); box-shadow: 0 0 10px var(--accent-emerald); animation: pulse 2s infinite; }
    @keyframes pulse { 0% { opacity: 0.4; } 50% { opacity: 1; } 100% { opacity: 0.4; } }

    /* Card standard */
    .card {
      background: var(--bg-card);
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
      border: 1px solid var(--border-card);
      border-radius: var(--radius-md);
      padding: 16px;
      display: flex;
      flex-direction: column;
      gap: 12px;
      box-shadow: 0 8px 24px rgba(0,0,0,0.3);
      position: relative;
      overflow: hidden;
    }
    .card-header-bar { display: flex; justify-content: space-between; align-items: center; width: 100%; border-bottom: 1px solid rgba(255,255,255,0.06); padding-bottom: 10px; }
    .card-title { font-size: 0.95rem; font-weight: 800; letter-spacing: 0.5px; display: flex; align-items: center; gap: 8px; }
    .tab-pills { display: flex; gap: 6px; }
    .tab-pill { background: rgba(255,255,255,0.06); border: 1px solid var(--border-card); color: var(--text-muted); font-size: 0.75rem; font-weight: 700; padding: 5px 12px; border-radius: 14px; cursor: pointer; transition: all 0.2s; }
    .tab-pill.active { background: linear-gradient(135deg, rgba(6,182,212,0.3), rgba(99,102,241,0.3)); border-color: var(--accent-sky); color: #fff; box-shadow: 0 0 12px rgba(6,182,212,0.3); }

    /* SVG Schematic Styling */
    .schematic-wrapper { width: 100%; display: flex; flex-direction: column; align-items: center; }
    .schematic-svg { width: 100%; max-height: 400px; filter: drop-shadow(0 0 15px rgba(0,0,0,0.5)); border-radius: 10px; background: radial-gradient(circle at 50% 50%, rgba(15,23,42,0.6) 0%, rgba(6,10,18,0.95) 100%); border: 1px solid rgba(255,255,255,0.05); }
    .joint-node { cursor: pointer; transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1); }
    .joint-node:hover circle.joint-outer { stroke-width: 3.5px; filter: brightness(1.3); }
    .joint-node.selected circle.joint-outer { stroke-width: 4px; stroke: #fff !important; filter: drop-shadow(0 0 8px #fff); }

    /* Interactive Nudge Toolbar */
    .nudge-toolbar {
      width: 100%;
      background: rgba(0,0,0,0.4);
      border: 1px solid var(--border-card);
      border-radius: var(--radius-sm);
      padding: 10px 14px;
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      align-items: center;
      gap: 10px;
      margin-top: 6px;
    }
    .nudge-info { display: flex; align-items: center; gap: 8px; font-size: 0.82rem; }
    .nudge-badge { background: rgba(56,189,248,0.15); border: 1px solid var(--accent-sky); color: var(--accent-sky); font-weight: 800; padding: 3px 8px; border-radius: 6px; }
    .nudge-buttons { display: flex; gap: 6px; flex-wrap: wrap; }
    .nudge-btn { background: rgba(255,255,255,0.07); border: 1px solid var(--border-card); color: var(--text-main); font-size: 0.78rem; font-weight: 800; padding: 6px 10px; border-radius: 6px; cursor: pointer; transition: all 0.15s; }
    .nudge-btn:hover { background: var(--accent-sky); color: #000; border-color: var(--accent-sky); }
    .nudge-btn:active { transform: scale(0.92); }

    /* Telemetry Grid */
    .telemetry-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; width: 100%; }
    .telem-card { background: rgba(0,0,0,0.3); border: 1px solid rgba(255,255,255,0.06); border-radius: var(--radius-sm); padding: 10px; display: flex; flex-direction: column; align-items: center; gap: 5px; cursor: pointer; transition: all 0.15s; }
    .telem-card:hover { border-color: var(--border-card-active); background: rgba(0,0,0,0.45); }
    .telem-name { font-size: 0.72rem; font-weight: 700; color: var(--text-muted); text-transform: uppercase; }
    .telem-angle { font-size: 1.35rem; font-weight: 900; font-family: ui-monospace, monospace; }
    .telem-r { color: var(--accent-amber); }
    .telem-l { color: var(--accent-cyan); }
    .telem-bar-track { width: 100%; height: 5px; background: rgba(255,255,255,0.08); border-radius: 3px; overflow: hidden; }
    .telem-bar-fill { height: 100%; width: 50%; border-radius: 3px; transition: width 0.15s; }

    /* D-Pad & Locomotion Controls */
    .motion-deck { display: grid; grid-template-columns: 1fr 1.3fr; gap: 16px; width: 100%; align-items: center; }
    @media (max-width: 650px) { .motion-deck { grid-template-columns: 1fr; } }
    .dpad-container { display: grid; grid-template-columns: repeat(3, 72px); grid-template-rows: repeat(3, 72px); gap: 8px; justify-content: center; margin: auto; }
    .dpad-btn { background: rgba(255, 255, 255, 0.05); border: 1px solid var(--border-card); color: var(--text-main); font-size: 1.3rem; font-weight: 800; border-radius: var(--radius-md); display: flex; flex-direction: column; align-items: center; justify-content: center; cursor: pointer; transition: all 0.15s; }
    .dpad-btn:hover { background: rgba(6, 182, 212, 0.2); border-color: var(--accent-cyan); color: var(--accent-cyan); box-shadow: 0 0 15px rgba(6,182,212,0.3); }
    .dpad-btn:active { transform: scale(0.92); }
    .dpad-btn span { font-size: 0.62rem; font-weight: 700; text-transform: uppercase; margin-top: 2px; }
    .btn-up { grid-column: 2; grid-row: 1; }
    .btn-left { grid-column: 1; grid-row: 2; }
    .btn-stop-center { grid-column: 2; grid-row: 2; background: rgba(244, 63, 94, 0.25); border-color: var(--accent-rose); color: #fda4af; box-shadow: 0 0 12px rgba(244,63,94,0.3); }
    .btn-stop-center:hover { background: var(--accent-rose); color: #fff; }
    .btn-right { grid-column: 3; grid-row: 2; }
    .btn-down { grid-column: 2; grid-row: 3; }

    .gait-actions-box { display: flex; flex-direction: column; gap: 10px; width: 100%; }
    .gait-action-btn { background: rgba(255,255,255,0.04); border: 1px solid var(--border-card); color: var(--text-main); padding: 12px 14px; border-radius: var(--radius-sm); font-size: 0.85rem; font-weight: 700; cursor: pointer; display: flex; align-items: center; justify-content: space-between; transition: all 0.15s; }
    .gait-action-btn:hover { background: rgba(255,255,255,0.08); border-color: var(--accent-sky); transform: translateX(2px); }
    .gait-action-btn:active { transform: scale(0.98); }
    .action-walk { border-color: rgba(16,185,129,0.3); background: rgba(16,185,129,0.12); color: #6ee7b7; }
    .action-step { border-color: rgba(99,102,241,0.3); background: rgba(99,102,241,0.12); color: #a5b4fc; }
    .action-test { border-color: rgba(245,158,11,0.3); background: rgba(245,158,11,0.12); color: #fde68a; }

    /* Sliders / Tuners */
    .tuner-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 10px; width: 100%; }
    .tuner-card { background: rgba(0,0,0,0.25); border: 1px solid var(--border-card); border-radius: var(--radius-sm); padding: 12px; display: flex; flex-direction: column; gap: 8px; }
    .tuner-header { display: flex; justify-content: space-between; align-items: center; }
    .tuner-label { font-size: 0.8rem; font-weight: 700; color: var(--text-muted); }
    .tuner-val { font-size: 0.95rem; font-weight: 900; color: var(--accent-sky); font-family: ui-monospace, monospace; }
    .slider-styled { width: 100%; height: 6px; accent-color: var(--accent-cyan); cursor: pointer; background: rgba(255,255,255,0.1); border-radius: 3px; }

    /* Joint Sliders Deck */
    .leg-section-title { font-size: 0.85rem; font-weight: 800; padding: 7px 12px; border-radius: 8px; margin-top: 6px; width: 100%; display: flex; justify-content: space-between; align-items: center; }
    .leg-r-header { background: rgba(245, 158, 11, 0.15); color: var(--accent-amber); border-left: 4px solid var(--accent-amber); }
    .leg-l-header { background: rgba(6, 182, 212, 0.15); color: var(--accent-cyan); border-left: 4px solid var(--accent-cyan); }
    .servo-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 10px; width: 100%; }
    .servo-card { background: rgba(0,0,0,0.25); border: 1px solid var(--border-card); border-radius: var(--radius-sm); padding: 12px; display: flex; flex-direction: column; gap: 8px; transition: border-color 0.2s; }
    .servo-card.active-target { border-color: var(--accent-sky); box-shadow: 0 0 12px rgba(56,189,248,0.2); }
    .servo-header { display: flex; justify-content: space-between; align-items: center; }
    .servo-name { font-size: 0.82rem; font-weight: 700; }
    .servo-angle-text { font-size: 0.95rem; font-weight: 800; font-family: ui-monospace, monospace; }
    .step-btn-group { display: grid; grid-template-columns: repeat(6, 1fr); gap: 4px; }
    .step-btn { background: rgba(255,255,255,0.06); border: 1px solid var(--border-card); color: var(--text-muted); font-size: 0.72rem; padding: 6px 2px; border-radius: 4px; cursor: pointer; font-weight: 700; text-align: center; }
    .step-btn:hover { background: var(--accent-sky); color: #000; }
    .step-btn:active { transform: scale(0.92); }

    /* Footer / Memory Actions */
    .save-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; width: 100%; }
    .btn-save { background: rgba(16, 185, 129, 0.2); border: 1px solid var(--accent-emerald); color: #a7f3d0; font-weight: 800; padding: 12px; border-radius: var(--radius-sm); cursor: pointer; font-size: 0.85rem; }
    .btn-save:hover { background: var(--accent-emerald); color: #000; }
    .btn-reset { background: rgba(244, 63, 94, 0.2); border: 1px solid var(--accent-rose); color: #fecdd3; font-weight: 800; padding: 12px; border-radius: var(--radius-sm); cursor: pointer; font-size: 0.85rem; }
    .btn-reset:hover { background: var(--accent-rose); color: #fff; }
  </style>
</head>
<body>
  <div class="container">
    <!-- Header -->
    <header>
      <div class="logo-container">
        <div class="logo-icon">🤖</div>
        <div>
          <div class="logo-title">6-DOF BIPEDAL ROBOT</div>
          <div class="logo-subtitle">MISSION CONTROL & SCHEMATIC STUDIO</div>
        </div>
      </div>
      <div class="status-group">
        <div class="status-badge">
          <div class="status-dot"></div>
          <span id="status-text">ONLINE (0x40 50Hz)</span>
        </div>
      </div>
    </header>

    <!-- Interactive Robot Schematic Card -->
    <section class="card">
      <div class="card-header-bar">
        <div class="card-title">
          <span>📐</span>
          <span>INTERACTIVE ROBOT SCHEMATIC</span>
        </div>
        <div class="tab-pills">
          <button class="tab-pill active" id="tab-btn-kinematic" onclick="switchSchematic('kinematic')">Kinematic Anatomy</button>
          <button class="tab-pill" id="tab-btn-circuit" onclick="switchSchematic('circuit')">Circuit & Pinout</button>
        </div>
      </div>

      <!-- Schematic SVG Container -->
      <div class="schematic-wrapper">
        <!-- 1. Kinematic Anatomy SVG -->
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
          <text x="270" y="40" text-anchor="middle" fill="#38bdf8" font-size="11" font-weight="800">ESP32 + PCA9685</text>
          <text x="270" y="55" text-anchor="middle" fill="#94a3b8" font-size="9" font-weight="600">PELVIS CORE (0x40)</text>

          <!-- Pelvis to Hip Structural Brackets -->
          <path d="M 220 68 L 140 100" stroke="#f59e0b" stroke-width="3" stroke-linecap="round" />
          <path d="M 320 68 L 400 100" stroke="#06b6d4" stroke-width="3" stroke-linecap="round" />

          <!-- RIGHT LEG LINKS (Thigh & Shin) -->
          <line id="link-thigh-r" x1="140" y1="100" x2="140" y2="200" stroke="url(#grad-r-link)" stroke-width="6" stroke-linecap="round" />
          <line id="link-shin-r" x1="140" y1="200" x2="140" y2="300" stroke="url(#grad-r-link)" stroke-width="6" stroke-linecap="round" />
          <!-- Right Foot Plate -->
          <rect x="105" y="318" width="70" height="14" rx="4" fill="#f59e0b" opacity="0.85" />
          <text x="140" y="329" text-anchor="middle" fill="#000" font-size="8" font-weight="900">R.FOOT SOLE</text>

          <!-- LEFT LEG LINKS (Thigh & Shin) -->
          <line id="link-thigh-l" x1="400" y1="100" x2="400" y2="200" stroke="url(#grad-l-link)" stroke-width="6" stroke-linecap="round" />
          <line id="link-shin-l" x1="400" y1="200" x2="400" y2="300" stroke="url(#grad-l-link)" stroke-width="6" stroke-linecap="round" />
          <!-- Left Foot Plate -->
          <rect x="365" y="318" width="70" height="14" rx="4" fill="#06b6d4" opacity="0.85" />
          <text x="400" y="329" text-anchor="middle" fill="#000" font-size="8" font-weight="900">L.FOOT SOLE</text>

          <!-- JOINT NODES -->
          <!-- Right Hip (CH 0) -->
          <g class="joint-node" id="node-0" onclick="selectJoint(0, 'Right Hip')">
            <circle class="joint-outer" cx="140" cy="100" r="22" fill="#0f172a" stroke="#f59e0b" stroke-width="2.5" filter="url(#glow-amber)" />
            <text x="140" y="96" text-anchor="middle" fill="#f59e0b" font-size="9" font-weight="800">CH 0</text>
            <text id="svg-angle-0" x="140" y="110" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="80" y="104" text-anchor="middle" fill="#f59e0b" font-size="10" font-weight="700">R.HIP</text>
          </g>

          <!-- Right Knee (CH 1) -->
          <g class="joint-node" id="node-1" onclick="selectJoint(1, 'Right Knee')">
            <circle class="joint-outer" cx="140" cy="200" r="22" fill="#0f172a" stroke="#f59e0b" stroke-width="2.5" filter="url(#glow-amber)" />
            <text x="140" y="196" text-anchor="middle" fill="#f59e0b" font-size="9" font-weight="800">CH 1</text>
            <text id="svg-angle-1" x="140" y="210" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">8°</text>
            <text x="80" y="204" text-anchor="middle" fill="#f59e0b" font-size="10" font-weight="700">R.KNEE</text>
          </g>

          <!-- Right Foot (CH 2) -->
          <g class="joint-node" id="node-2" onclick="selectJoint(2, 'Right Foot')">
            <circle class="joint-outer" cx="140" cy="300" r="22" fill="#0f172a" stroke="#f59e0b" stroke-width="2.5" filter="url(#glow-amber)" />
            <text x="140" y="296" text-anchor="middle" fill="#f59e0b" font-size="9" font-weight="800">CH 2</text>
            <text id="svg-angle-2" x="140" y="310" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="80" y="304" text-anchor="middle" fill="#f59e0b" font-size="10" font-weight="700">R.FOOT</text>
          </g>

          <!-- Left Hip (CH 4) -->
          <g class="joint-node" id="node-4" onclick="selectJoint(4, 'Left Hip')">
            <circle class="joint-outer" cx="400" cy="100" r="22" fill="#0f172a" stroke="#06b6d4" stroke-width="2.5" filter="url(#glow-cyan)" />
            <text x="400" y="96" text-anchor="middle" fill="#06b6d4" font-size="9" font-weight="800">CH 4</text>
            <text id="svg-angle-4" x="400" y="110" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="460" y="104" text-anchor="middle" fill="#06b6d4" font-size="10" font-weight="700">L.HIP</text>
          </g>

          <!-- Left Knee (CH 8) -->
          <g class="joint-node" id="node-8" onclick="selectJoint(8, 'Left Knee')">
            <circle class="joint-outer" cx="400" cy="200" r="22" fill="#0f172a" stroke="#06b6d4" stroke-width="2.5" filter="url(#glow-cyan)" />
            <text x="400" y="196" text-anchor="middle" fill="#06b6d4" font-size="9" font-weight="800">CH 8</text>
            <text id="svg-angle-8" x="400" y="210" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="460" y="204" text-anchor="middle" fill="#06b6d4" font-size="10" font-weight="700">L.KNEE</text>
          </g>

          <!-- Left Foot (CH 12) -->
          <g class="joint-node" id="node-12" onclick="selectJoint(12, 'Left Foot')">
            <circle class="joint-outer" cx="400" cy="300" r="22" fill="#0f172a" stroke="#06b6d4" stroke-width="2.5" filter="url(#glow-cyan)" />
            <text x="400" y="296" text-anchor="middle" fill="#06b6d4" font-size="9" font-weight="800">CH 12</text>
            <text id="svg-angle-12" x="400" y="310" text-anchor="middle" fill="#fff" font-size="11" font-weight="900">94°</text>
            <text x="460" y="304" text-anchor="middle" fill="#06b6d4" font-size="10" font-weight="700">L.FOOT</text>
          </g>

          <!-- Center Status Card in SVG -->
          <rect x="210" y="150" width="120" height="65" rx="6" fill="rgba(15,23,42,0.85)" stroke="rgba(255,255,255,0.1)" />
          <text x="270" y="172" text-anchor="middle" fill="#94a3b8" font-size="9" font-weight="700">GAIT STATE</text>
          <text id="svg-active-gait" x="270" y="195" text-anchor="middle" fill="#10b981" font-size="14" font-weight="900">STAND</text>
        </svg>

        <!-- 2. Electrical Circuit & Pinout SVG (Hidden by default) -->
        <svg id="svg-circuit" class="schematic-svg" viewBox="0 0 540 370" style="display: none;">
          <!-- ESP32 Block -->
          <rect x="30" y="40" width="140" height="290" rx="8" fill="#1e293b" stroke="#38bdf8" stroke-width="2" />
          <text x="100" y="65" text-anchor="middle" fill="#38bdf8" font-size="12" font-weight="800">ESP32 DevKit</text>
          <text x="100" y="80" text-anchor="middle" fill="#64748b" font-size="8" font-weight="600">30-PIN MICROCONTROLLER</text>
          
          <rect x="135" y="110" width="30" height="18" rx="3" fill="#0284c7" />
          <text x="150" y="123" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">G21 (SDA)</text>
          <rect x="135" y="145" width="30" height="18" rx="3" fill="#0284c7" />
          <text x="150" y="158" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">G22 (SCL)</text>

          <rect x="135" y="190" width="30" height="18" rx="3" fill="#dc2626" />
          <text x="150" y="203" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">3V3</text>
          <rect x="135" y="225" width="30" height="18" rx="3" fill="#475569" />
          <text x="150" y="238" text-anchor="middle" fill="#fff" font-size="8" font-weight="800">GND</text>

          <!-- PCA9685 Block -->
          <rect x="230" y="40" width="140" height="290" rx="8" fill="#0f172a" stroke="#6366f1" stroke-width="2" />
          <text x="300" y="65" text-anchor="middle" fill="#818cf8" font-size="12" font-weight="800">PCA9685 Driver</text>
          <text x="300" y="80" text-anchor="middle" fill="#64748b" font-size="8" font-weight="600">I2C: 0x40 (50Hz PWM)</text>

          <!-- I2C Bus Wires -->
          <path d="M 165 119 L 230 119" stroke="#38bdf8" stroke-width="2.5" />
          <path d="M 165 154 L 230 154" stroke="#818cf8" stroke-width="2.5" />
          <path d="M 165 199 L 230 199" stroke="#ef4444" stroke-width="2" />
          <path d="M 165 234 L 230 234" stroke="#64748b" stroke-width="2" />

          <!-- External Servo Power Terminal -->
          <rect x="250" y="290" width="100" height="30" rx="4" fill="#334155" stroke="#f59e0b" />
          <text x="300" y="309" text-anchor="middle" fill="#fbbf24" font-size="9" font-weight="800">EXT 5V-6V (V+ / GND)</text>

          <!-- Servo Channel Outputs -->
          <!-- Right Leg Servos -->
          <g>
            <path d="M 370 100 L 415 100" stroke="#f59e0b" stroke-width="2" />
            <rect x="415" y="90" width="100" height="22" rx="4" fill="#1e293b" stroke="#f59e0b" />
            <text x="465" y="104" text-anchor="middle" fill="#fbbf24" font-size="8" font-weight="800">CH 0: Right Hip</text>
          </g>
          <g>
            <path d="M 370 135 L 415 135" stroke="#f59e0b" stroke-width="2" />
            <rect x="415" y="125" width="100" height="22" rx="4" fill="#1e293b" stroke="#f59e0b" />
            <text x="465" y="139" text-anchor="middle" fill="#fbbf24" font-size="8" font-weight="800">CH 1: Right Knee</text>
          </g>
          <g>
            <path d="M 370 170 L 415 170" stroke="#f59e0b" stroke-width="2" />
            <rect x="415" y="160" width="100" height="22" rx="4" fill="#1e293b" stroke="#f59e0b" />
            <text x="465" y="174" text-anchor="middle" fill="#fbbf24" font-size="8" font-weight="800">CH 2: Right Foot</text>
          </g>

          <!-- Left Leg Servos -->
          <g>
            <path d="M 370 215 L 415 215" stroke="#06b6d4" stroke-width="2" />
            <rect x="415" y="205" width="100" height="22" rx="4" fill="#1e293b" stroke="#06b6d4" />
            <text x="465" y="219" text-anchor="middle" fill="#67e8f9" font-size="8" font-weight="800">CH 4: Left Hip</text>
          </g>
          <g>
            <path d="M 370 250 L 415 250" stroke="#06b6d4" stroke-width="2" />
            <rect x="415" y="240" width="100" height="22" rx="4" fill="#1e293b" stroke="#06b6d4" />
            <text x="465" y="254" text-anchor="middle" fill="#67e8f9" font-size="8" font-weight="800">CH 8: Left Knee</text>
          </g>
          <g>
            <path d="M 370 285 L 415 285" stroke="#06b6d4" stroke-width="2" />
            <rect x="415" y="275" width="100" height="22" rx="4" fill="#1e293b" stroke="#06b6d4" />
            <text x="465" y="289" text-anchor="middle" fill="#67e8f9" font-size="8" font-weight="800">CH 12: Left Foot</text>
          </g>
        </svg>

        <!-- Interactive Quick Nudge Toolbar -->
        <div class="nudge-toolbar">
          <div class="nudge-info">
            <span style="color: var(--text-muted);">ACTIVE JOINT:</span>
            <span class="nudge-badge" id="selected-joint-badge">Right Hip (CH 0)</span>
            <span style="font-family: monospace; font-weight: 800; font-size: 0.9rem;" id="selected-joint-angle">94°</span>
          </div>
          <div class="nudge-buttons">
            <button class="nudge-btn" onclick="stepSelected(-10)">-10°</button>
            <button class="nudge-btn" onclick="stepSelected(-5)">-5°</button>
            <button class="nudge-btn" onclick="stepSelected(-1)">-1°</button>
            <button class="nudge-btn" onclick="stepSelected(+1)">+1°</button>
            <button class="nudge-btn" onclick="stepSelected(+5)">+5°</button>
            <button class="nudge-btn" onclick="stepSelected(+10)">+10°</button>
          </div>
        </div>
      </div>
    </section>

    <!-- Telemetry Cards -->
    <section class="card">
      <div class="card-header-bar">
        <div class="card-title"><span>📡</span><span>REAL-TIME JOINT TELEMETRY</span></div>
      </div>
      <div class="telemetry-grid">
        <div class="telem-card" onclick="selectJoint(0, 'Right Hip')">
          <span class="telem-name">R.Hip (CH 0)</span>
          <span class="telem-angle telem-r" id="track-0">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-0" style="background: var(--accent-amber);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(1, 'Right Knee')">
          <span class="telem-name">R.Knee (CH 1)</span>
          <span class="telem-angle telem-r" id="track-1">8°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-1" style="background: var(--accent-amber);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(2, 'Right Foot')">
          <span class="telem-name">R.Foot (CH 2)</span>
          <span class="telem-angle telem-r" id="track-2">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-2" style="background: var(--accent-amber);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(4, 'Left Hip')">
          <span class="telem-name">L.Hip (CH 4)</span>
          <span class="telem-angle telem-l" id="track-4">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-4" style="background: var(--accent-cyan);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(8, 'Left Knee')">
          <span class="telem-name">L.Knee (CH 8)</span>
          <span class="telem-angle telem-l" id="track-8">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-8" style="background: var(--accent-cyan);"></div></div>
        </div>
        <div class="telem-card" onclick="selectJoint(12, 'Left Foot')">
          <span class="telem-name">L.Foot (CH 12)</span>
          <span class="telem-angle telem-l" id="track-12">94°</span>
          <div class="telem-bar-track"><div class="telem-bar-fill" id="bar-12" style="background: var(--accent-cyan);"></div></div>
        </div>
      </div>
    </section>

    <!-- Mission Control Locomotion Deck -->
    <section class="card">
      <div class="card-header-bar">
        <div class="card-title"><span>🎮</span><span>LOCOMOTION & MISSION CONTROL</span></div>
      </div>
      <div class="motion-deck">
        <!-- D-Pad -->
        <div class="dpad-container">
          <button class="dpad-btn btn-up" onclick="runGait('walk')">▲<span>Forward</span></button>
          <button class="dpad-btn btn-left" onclick="runGait('left')">◀<span>Turn L</span></button>
          <button class="dpad-btn btn-stop-center" onclick="runGait('stand')">🛑<span>Stop</span></button>
          <button class="dpad-btn btn-right" onclick="runGait('right')">▶<span>Turn R</span></button>
          <button class="dpad-btn btn-down" onclick="runGait('backward')">▼<span>Back</span></button>
        </div>

        <!-- Stride Actions -->
        <div class="gait-actions-box">
          <button class="gait-action-btn action-walk" onclick="runGait('walk')">
            <span>🚀 1. Continuous Dynamic Walk</span>
            <span>▶</span>
          </button>
          <button class="gait-action-btn action-step" onclick="runGait('step')">
            <span>🐾 2. Single Full Stride Cycle</span>
            <span>▶</span>
          </button>
          <button class="gait-action-btn action-test" onclick="runGait('test')">
            <span>🧪 3. Test All 6 Joints Sequentially</span>
            <span>▶</span>
          </button>
          <button class="gait-action-btn" onclick="runGait('stand')" style="border-color: rgba(244,63,94,0.3); color: #fca5a5;">
            <span>🛑 4. Park Neutral Stand</span>
            <span>⏹</span>
          </button>
        </div>
      </div>
    </section>

    <!-- 6-Joint Micro-Amplitude Gait Tuner -->
    <section class="card">
      <div class="card-header-bar">
        <div class="card-title"><span>🎛️</span><span>6-JOINT MICRO-AMPLITUDE GAIT TUNER</span></div>
      </div>
      <div class="tuner-grid">
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Cadence (Speed)</span><span class="tuner-val" id="val-freq">0.65 Hz</span></div>
          <input type="range" class="slider-styled" id="slider-freq" min="4" max="12" value="6.5" step="0.5" oninput="setGaitParam('freq', this.value / 10.0)">
        </div>
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Hip Stride Amp</span><span class="tuner-val" id="val-stride">4.5°</span></div>
          <input type="range" class="slider-styled" id="slider-stride" min="1.5" max="6.0" value="4.5" step="0.5" oninput="setGaitParam('stride', this.value)">
        </div>
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Knee Lift Amp</span><span class="tuner-val" id="val-knee">3.0°</span></div>
          <input type="range" class="slider-styled" id="slider-knee" min="1.0" max="4.5" value="3.0" step="0.5" oninput="setGaitParam('knee', this.value)">
        </div>
        <div class="tuner-card">
          <div class="tuner-header"><span class="tuner-label">Foot Level Amp</span><span class="tuner-val" id="val-ankle">2.0°</span></div>
          <input type="range" class="slider-styled" id="slider-ankle" min="0.5" max="4.0" value="2.0" step="0.5" oninput="setGaitParam('ankle', this.value)">
        </div>
      </div>
    </section>

    <!-- Right Leg Joint Controls -->
    <div class="leg-section-title leg-r-header">
      <span>RIGHT LEG JOINTS (SAGITTAL PITCH)</span>
      <span style="font-size: 0.75rem; font-weight: 700;">CH 0: Hip (94°) | CH 1: Knee (8°) | CH 2: Foot (94°)</span>
    </div>
    <div class="servo-grid">
      <!-- CH 0 -->
      <div class="servo-card" id="card-0">
        <div class="servo-header"><span class="servo-name" style="color: var(--accent-amber);">Right Hip (CH 0)</span><span class="servo-angle-text" id="angle-0">94°</span></div>
        <input type="range" class="slider-styled" id="slider-0" min="0" max="180" value="94" oninput="setAngle(0, this.value)" onchange="setAngle(0, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(0, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(0, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(0, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(0, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(0, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(0, 10)">+10</button>
        </div>
      </div>
      <!-- CH 1 -->
      <div class="servo-card" id="card-1">
        <div class="servo-header"><span class="servo-name" style="color: var(--accent-amber);">Right Knee (CH 1)</span><span class="servo-angle-text" id="angle-1">8°</span></div>
        <input type="range" class="slider-styled" id="slider-1" min="0" max="180" value="8" oninput="setAngle(1, this.value)" onchange="setAngle(1, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(1, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(1, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(1, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(1, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(1, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(1, 10)">+10</button>
        </div>
      </div>
      <!-- CH 2 -->
      <div class="servo-card" id="card-2">
        <div class="servo-header"><span class="servo-name" style="color: var(--accent-amber);">Right Foot (CH 2)</span><span class="servo-angle-text" id="angle-2">94°</span></div>
        <input type="range" class="slider-styled" id="slider-2" min="0" max="180" value="94" oninput="setAngle(2, this.value)" onchange="setAngle(2, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(2, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(2, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(2, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(2, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(2, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(2, 10)">+10</button>
        </div>
      </div>
    </div>

    <!-- Left Leg Joint Controls -->
    <div class="leg-section-title leg-l-header">
      <span>LEFT LEG JOINTS (SAGITTAL PITCH)</span>
      <span style="font-size: 0.75rem; font-weight: 700;">CH 4: Hip (94°) | CH 8: Knee (94°) | CH 12: Foot (94°)</span>
    </div>
    <div class="servo-grid">
      <!-- CH 4 -->
      <div class="servo-card" id="card-4">
        <div class="servo-header"><span class="servo-name" style="color: var(--accent-cyan);">Left Hip (CH 4)</span><span class="servo-angle-text" id="angle-4">94°</span></div>
        <input type="range" class="slider-styled" id="slider-4" min="0" max="180" value="94" oninput="setAngle(4, this.value)" onchange="setAngle(4, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(4, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(4, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(4, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(4, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(4, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(4, 10)">+10</button>
        </div>
      </div>
      <!-- CH 8 -->
      <div class="servo-card" id="card-8">
        <div class="servo-header"><span class="servo-name" style="color: var(--accent-cyan);">Left Knee (CH 8)</span><span class="servo-angle-text" id="angle-8">94°</span></div>
        <input type="range" class="slider-styled" id="slider-8" min="0" max="180" value="94" oninput="setAngle(8, this.value)" onchange="setAngle(8, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(8, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(8, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(8, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(8, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(8, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(8, 10)">+10</button>
        </div>
      </div>
      <!-- CH 12 -->
      <div class="servo-card" id="card-12">
        <div class="servo-header"><span class="servo-name" style="color: var(--accent-cyan);">Left Foot (CH 12)</span><span class="servo-angle-text" id="angle-12">94°</span></div>
        <input type="range" class="slider-styled" id="slider-12" min="0" max="180" value="94" oninput="setAngle(12, this.value)" onchange="setAngle(12, this.value)">
        <div class="step-btn-group">
          <button class="step-btn" onclick="stepAngle(12, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(12, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(12, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(12, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(12, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(12, 10)">+10</button>
        </div>
      </div>
    </div>

    <!-- Flash Memory & System Configuration -->
    <section class="card">
      <div class="card-header-bar">
        <div class="card-title"><span>💾</span><span>NON-VOLATILE FLASH STORAGE (EEPROM)</span></div>
      </div>
      <div class="save-actions">
        <button class="btn-save" onclick="saveOffsets()">💾 Save Current Position As Default Stand</button>
        <button class="btn-reset" onclick="resetOffsets()">🔄 Reset All Offsets To Factory Default</button>
      </div>
    </section>
  </div>

  <script>
    let sendTimers = {};
    let selectedJointChannel = 0;
    let selectedJointName = 'Right Hip';

    function sendCmd(url) {
      fetch(url).catch(()=>{});
    }

    function switchSchematic(tab) {
      const kSvg = document.getElementById('svg-kinematic');
      const cSvg = document.getElementById('svg-circuit');
      const kBtn = document.getElementById('tab-btn-kinematic');
      const cBtn = document.getElementById('tab-btn-circuit');

      if (tab === 'kinematic') {
        kSvg.style.display = 'block';
        cSvg.style.display = 'none';
        kBtn.classList.add('active');
        cBtn.classList.remove('active');
      } else {
        kSvg.style.display = 'none';
        cSvg.style.display = 'block';
        cBtn.classList.add('active');
        kBtn.classList.remove('active');
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

      // Highlight in SVG
      [0, 1, 2, 4, 8, 12].forEach(c => {
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
      sendCmd('/run_gait?gait=' + gait);
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
      } else if (param === 'ankle') {
        const el = document.getElementById('val-ankle');
        if (el) el.innerText = val.toFixed(1) + '°';
      }
      sendCmd('/set_gait_param?param=' + param + '&val=' + val);
    }

    function saveOffsets() {
      sendCmd('/save_offsets');
      alert('All 6 Joint Angles Saved Permanently to ESP32 Flash Memory!');
    }

    function resetOffsets() {
      if (confirm('Reset all 6 joint angles to factory baselines?')) {
        sendCmd('/reset_offsets');
        setTimeout(pollTelemetry, 300);
      }
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
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = (float)servos[i].homeAngle;
    setServoAngleDirect(servos[i].channel, (float)servos[i].homeAngle);
  }
}

void updateHumanLikeWalk() {
  float t = (float)(millis() - walkStartTime) / 1000.0f;

  // 1. Smooth Anti-Jerk Envelope (eases smoothly into motion)
  if (gaitRunning) {
    if (currentGaitScale < 1.0f) currentGaitScale = min(1.0f, currentGaitScale + 0.03f);
  } else {
    if (currentGaitScale > 0.0f) currentGaitScale = max(0.0f, currentGaitScale - 0.05f);
  }

  // 2. Continuous Gait Phase (0 to 2*PI)
  float phase = fmod(2.0f * (float)M_PI * walkFreq * t, 2.0f * (float)M_PI);
  if (phase < 0.0f) phase += 2.0f * (float)M_PI;

  float aHipR  = (float)servos[0].homeAngle;
  float aKneeR = (float)servos[1].homeAngle;
  float aFootR = (float)servos[2].homeAngle;
  float aHipL  = (float)servos[3].homeAngle;
  float aKneeL = (float)servos[4].homeAngle;
  float aFootL = (float)servos[5].homeAngle;

  // 3. Coordinated 6-Joint Micro-Amplitude Human Walking Gait
  // Sub-millimeter clearances & small angular distance (< 4.5°) guarantees:
  // - ZERO bracket-to-hip collisions (Right knee <= 12.5°, Left knee >= 89.5°)
  // - Dynamic natural gait: Hip swings, Knee flexes for foot clearance, Foot levels flat & pushes off
  if (phase < (float)M_PI) {
    // Phase 1: Right Leg SWING (Forward + Knee lift + Foot level), Left Leg STANCE (Push-off)
    float sw = sin(phase) * currentGaitScale;
    aHipR  = (float)servos[0].homeAngle + (strideAmp * sw);
    aKneeR = (float)servos[1].homeAngle + (kneeAmp * sw);
    aFootR = (float)servos[2].homeAngle - (ankleAmp * sw);

    aHipL  = (float)servos[3].homeAngle + (pushHip * sw);
    aKneeL = (float)servos[4].homeAngle;
    aFootL = (float)servos[5].homeAngle - (pushAnk * sw);
  } else {
    // Phase 2: Left Leg SWING (Forward + Knee lift + Foot level), Right Leg STANCE (Push-off)
    float psi = phase - (float)M_PI;
    float sw = sin(psi) * currentGaitScale;
    aHipL  = (float)servos[3].homeAngle - (strideAmp * sw);
    aKneeL = (float)servos[4].homeAngle - (kneeAmp * sw);
    aFootL = (float)servos[5].homeAngle + (ankleAmp * sw);

    aHipR  = (float)servos[0].homeAngle - (pushHip * sw);
    aKneeR = (float)servos[1].homeAngle;
    aFootR = (float)servos[2].homeAngle + (pushAnk * sw);
  }

  // 4. Strict Safety Clamping (Hard mechanical boundary protection)
  aHipR  = constrain(aHipR,  92.5f, 100.0f);
  aKneeR = constrain(aKneeR,  8.0f,  12.5f); // NEVER exceeds 12.5° -> 100% collision free!
  aFootR = constrain(aFootR, 90.0f,  97.0f);
  aHipL  = constrain(aHipL,  88.0f,  95.5f);
  aKneeL = constrain(aKneeL, 89.5f,  94.0f); // NEVER drops below 89.5° -> 100% collision free!
  aFootL = constrain(aFootL, 91.0f,  98.0f);

  // Apply directly to PWM servo driver
  servos[0].currentAngle = aHipR;  setServoAngleDirect(0,  aHipR);  // CH 0 Right Hip
  servos[1].currentAngle = aKneeR; setServoAngleDirect(1,  aKneeR); // CH 1 Right Knee
  servos[2].currentAngle = aFootR; setServoAngleDirect(2,  aFootR); // CH 2 Right Foot
  servos[3].currentAngle = aHipL;  setServoAngleDirect(4,  aHipL);  // CH 4 Left Hip
  servos[4].currentAngle = aKneeL; setServoAngleDirect(8,  aKneeL); // CH 8 Left Knee
  servos[5].currentAngle = aFootL; setServoAngleDirect(12, aFootL); // CH 12 Left Foot
}

void executeSingleHumanStride() {
  gaitRunning = true;
  activeGaitName = "step";
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

  // 1. Right Hip (CH 0): 94° -> 98.5° -> 94°
  setServoAngleDirect(0, 98.5f); servos[0].currentAngle = 98.5f; delay(1000);
  setServoAngleDirect(0, 94.0f); servos[0].currentAngle = 94.0f; delay(500);

  // 2. Right Knee (CH 1): 8° -> 11.0° -> 8° (Gentle micro-lift, zero collision)
  setServoAngleDirect(1, 11.0f); servos[1].currentAngle = 11.0f; delay(1000);
  setServoAngleDirect(1, 8.0f);  servos[1].currentAngle = 8.0f;  delay(500);

  // 3. Right Foot (CH 2): 94° -> 92.0° -> 94°
  setServoAngleDirect(2, 92.0f); servos[2].currentAngle = 92.0f; delay(1000);
  setServoAngleDirect(2, 94.0f); servos[2].currentAngle = 94.0f; delay(500);

  // 4. Left Hip (CH 4): 94° -> 89.5° -> 94°
  setServoAngleDirect(4, 89.5f); servos[3].currentAngle = 89.5f; delay(1000);
  setServoAngleDirect(4, 94.0f); servos[3].currentAngle = 94.0f; delay(500);

  // 5. Left Knee (CH 8): 94° -> 91.0° -> 94° (Gentle micro-lift, zero collision)
  setServoAngleDirect(8, 91.0f); servos[4].currentAngle = 91.0f; delay(1000);
  setServoAngleDirect(8, 94.0f); servos[4].currentAngle = 94.0f; delay(500);

  // 6. Left Foot (CH 12): 94° -> 96.0° -> 94°
  setServoAngleDirect(12, 96.0f); servos[5].currentAngle = 96.0f; delay(1000);
  setServoAngleDirect(12, 94.0f); servos[5].currentAngle = 94.0f; delay(500);

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
  json += "},\"active_gait\":\"" + activeGaitName + "\",\"gait_running\":" + (gaitRunning ? "true" : "false") + ",\"pca_found\":" + (pcaFound ? "true" : "false") + "}";
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
  if (gait == "stand" || gait == "stop") {
    parkStand();
  } else if (gait == "walk" || gait == "forward") {
    gaitRunning = true;
    activeGaitName = "walk";
    walkStartTime = millis();
  } else if (gait == "step") {
    gaitRunning = false;
    activeGaitName = "step";
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"step\"}");
    executeSingleHumanStride();
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
  else if (param == "stride") strideAmp = val;
  else if (param == "knee") kneeAmp = val;
  else if (param == "ankle") ankleAmp = val;
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
  Wire.setClock(100000); // 100kHz standard speed for stable I2C
  Wire.setTimeOut(100);

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

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifi < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("Web URL: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection timed out. Connect to AP: " + String(ap_ssid));
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

  // Auto-reconnect PCA9685 whenever it powers up or reconnects
  static unsigned long lastPcaCheck = 0;
  if (!pcaFound && (millis() - lastPcaCheck > 1500)) {
    lastPcaCheck = millis();
    if (initPCA9685()) {
      parkStand();
    }
  }

  if (gaitRunning && (activeGaitName == "walk" || activeGaitName == "forward")) {
    updateHumanLikeWalk();
    delay(15);
  }
}
