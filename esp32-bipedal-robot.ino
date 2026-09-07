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
// 2. Servo & Calibrated Offsets
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

// Calibrated Baselines: Right [CH 0: 8°, CH 1: 94°, CH 2: 94°] | Left [CH 6: 94°, CH 4: 94°, CH 5: 94°]
Joint servos[NUM_SERVOS] = {
  {0, "Right Hip",   8.0f,  8.0f, -1,  8},
  {1, "Right Knee", 94.0f, 94.0f, -1, 94},
  {2, "Right Foot", 94.0f, 94.0f, -1, 94},
  {6, "Left Hip",   94.0f, 94.0f, -1, 94},
  {4, "Left Knee",  94.0f, 94.0f, -1, 94},
  {5, "Left Foot",  94.0f, 94.0f, -1, 94}
};

String activeGaitName = "stand";
bool gaitRunning = false;

// Locomotion Parameters
float strideAmp = 12.0f;     // 12° Forward Stride Amplitude
float walkFreq = 1.25f;      // 1.25 Hz Cadence
unsigned long walkStartTime = 0;

void loadSavedOffsets() {
  preferences.begin("biped", false);
  servos[0].homeAngle = preferences.getInt("ch0", 8);
  servos[1].homeAngle = preferences.getInt("ch1", 94);
  servos[2].homeAngle = preferences.getInt("ch2", 94);
  servos[3].homeAngle = preferences.getInt("ch6", 94);
  servos[4].homeAngle = preferences.getInt("ch4", 94);
  servos[5].homeAngle = preferences.getInt("ch5", 94);
  preferences.end();

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = (float)servos[i].homeAngle;
    servos[i].targetAngle  = (float)servos[i].homeAngle;
  }
}

void saveCurrentOffsets() {
  preferences.begin("biped", false);
  preferences.putInt("ch0", (int)servos[0].currentAngle);
  preferences.putInt("ch1", (int)servos[1].currentAngle);
  preferences.putInt("ch2", (int)servos[2].currentAngle);
  preferences.putInt("ch6", (int)servos[3].currentAngle);
  preferences.putInt("ch4", (int)servos[4].currentAngle);
  preferences.putInt("ch5", (int)servos[5].currentAngle);
  preferences.end();

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].homeAngle = (int)servos[i].currentAngle;
  }
}

void resetFactoryOffsets() {
  preferences.begin("biped", false);
  preferences.clear();
  preferences.end();
  servos[0].homeAngle = 8;   // Right Hip
  servos[1].homeAngle = 94;  // Right Knee
  servos[2].homeAngle = 94;  // Right Foot
  servos[3].homeAngle = 94;  // Left Hip
  servos[4].homeAngle = 94;  // Left Knee
  servos[5].homeAngle = 94;  // Left Foot
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = (float)servos[i].homeAngle;
    servos[i].targetAngle  = (float)servos[i].homeAngle;
  }
}

// ==========================================
// 3. HTML Web Dashboard with Live Telemetry
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Biped Robot Precision Studio</title>
  <style>
    :root {
      --bg-primary: #0a0e17;
      --bg-card: #141c2e;
      --accent: #38bdf8;
      --accent-secondary: #818cf8;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --danger: #f43f5e;
      --success: #10b981;
      --warning: #f59e0b;
      --leg-r: #f59e0b;
      --leg-l: #38bdf8;
      --border-color: rgba(255, 255, 255, 0.08);
      --radius-sm: 8px;
      --radius-md: 14px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; user-select: none; }
    body { background-color: var(--bg-primary); color: var(--text-main); min-height: 100vh; padding: 12px; display: flex; flex-direction: column; align-items: center; }
    .container { width: 100%; max-width: 900px; display: flex; flex-direction: column; gap: 14px; }
    header { background: var(--bg-card); border: 1px solid var(--border-color); border-radius: var(--radius-md); padding: 12px 18px; display: flex; justify-content: space-between; align-items: center; }
    .logo-text { font-size: 1.15rem; font-weight: 800; background: linear-gradient(135deg, var(--accent), var(--accent-secondary)); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
    .status-badge { display: flex; align-items: center; gap: 6px; font-size: 0.8rem; background: rgba(0,0,0,0.25); padding: 5px 10px; border-radius: 20px; border: 1px solid var(--border-color); }
    .status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--success); }
    
    .card { background: var(--bg-card); border: 1px solid var(--border-color); border-radius: var(--radius-md); padding: 16px; display: flex; flex-direction: column; align-items: center; gap: 12px; }
    .card-title { font-size: 0.95rem; font-weight: 700; color: var(--text-muted); align-self: flex-start; }

    /* Live Telemetry Bar Grid */
    .tracker-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; width: 100%; }
    .tracker-card { background: rgba(0,0,0,0.3); border: 1px solid var(--border-color); border-radius: var(--radius-sm); padding: 10px; display: flex; flex-direction: column; align-items: center; gap: 6px; }
    .tracker-joint-name { font-size: 0.75rem; font-weight: 700; color: var(--text-muted); }
    .tracker-angle-big { font-size: 1.4rem; font-weight: 900; color: var(--accent); font-family: monospace; }
    .tracker-bar-bg { width: 100%; height: 6px; background: rgba(255,255,255,0.1); border-radius: 3px; overflow: hidden; }
    .tracker-bar-fill { height: 100%; background: linear-gradient(90deg, var(--accent), var(--accent-secondary)); width: 50%; transition: width 0.1s; }

    /* D-Pad Layout */
    .dpad-container { display: grid; grid-template-columns: repeat(3, 75px); grid-template-rows: repeat(3, 75px); gap: 10px; justify-content: center; margin: 10px 0; }
    .dpad-btn { background: rgba(255, 255, 255, 0.06); border: 1px solid var(--border-color); color: var(--text-main); font-size: 1.4rem; font-weight: 800; border-radius: var(--radius-md); display: flex; flex-direction: column; align-items: center; justify-content: center; cursor: pointer; transition: all 0.12s; }
    .dpad-btn:hover { background: rgba(56, 189, 248, 0.2); border-color: var(--accent); color: var(--accent); }
    .dpad-btn:active { transform: scale(0.92); }
    .dpad-btn span { font-size: 0.65rem; font-weight: 700; text-transform: uppercase; margin-top: 2px; }
    .btn-up { grid-column: 2; grid-row: 1; }
    .btn-left { grid-column: 1; grid-row: 2; }
    .btn-stop-center { grid-column: 2; grid-row: 2; background: rgba(244, 63, 94, 0.2); border-color: var(--danger); color: #fda4af; }
    .btn-right { grid-column: 3; grid-row: 2; }
    .btn-down { grid-column: 2; grid-row: 3; }

    .gait-actions { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 8px; width: 100%; }
    .btn { background: rgba(255,255,255,0.04); border: 1px solid var(--border-color); color: var(--text-main); padding: 12px 10px; border-radius: var(--radius-sm); font-size: 0.85rem; font-weight: 700; cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 6px; transition: all 0.15s; text-align: center; }
    .btn:hover { background: rgba(255,255,255,0.08); border-color: var(--accent); }
    .btn:active { transform: scale(0.97); }

    .save-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; width: 100%; margin-top: 6px; }
    .btn-save { background: rgba(16, 185, 129, 0.15); border-color: var(--success); color: var(--success); font-weight: 800; }
    .btn-reset { background: rgba(244, 63, 94, 0.15); border-color: var(--danger); color: #fda4af; font-weight: 800; }

    .servo-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; width: 100%; }
    .servo-card { background: rgba(0,0,0,0.2); border: 1px solid var(--border-color); border-radius: var(--radius-sm); padding: 10px; display: flex; flex-direction: column; gap: 8px; }
    .servo-header { display: flex; justify-content: space-between; align-items: center; }
    .servo-name { font-size: 0.8rem; font-weight: 700; }
    .angle-display { font-size: 0.95rem; font-weight: 800; color: var(--accent); font-family: monospace; }
    .slider-box input[type="range"] { width: 100%; height: 6px; accent-color: var(--accent); cursor: pointer; }
    .step-buttons { display: grid; grid-template-columns: repeat(6, 1fr); gap: 4px; }
    .step-btn { background: rgba(255,255,255,0.05); border: 1px solid var(--border-color); color: var(--text-muted); font-size: 0.72rem; padding: 6px 2px; border-radius: 4px; cursor: pointer; font-weight: 700; text-align: center; }
    .step-btn:hover { background: var(--accent); color: #000; }

    .leg-section-title { font-size: 0.85rem; font-weight: 800; padding: 6px 10px; border-radius: 6px; margin-top: 8px; margin-bottom: 6px; width: 100%; }
    .leg-r-title { background: rgba(245, 158, 11, 0.15); color: var(--leg-r); border-left: 4px solid var(--leg-r); }
    .leg-l-title { background: rgba(56, 189, 248, 0.15); color: var(--leg-l); border-left: 4px solid var(--leg-l); }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div class="logo-box">
        <span class="logo-text">BIPED ROBOT PRECISION STUDIO</span>
      </div>
      <div class="status-badge">
        <div class="status-dot"></div>
        <span id="status-text">Online</span>
      </div>
    </header>

    <!-- Live Telemetry -->
    <section class="card">
      <div class="card-title">📡 Real-Time Joint Telemetry</div>
      <div class="tracker-grid">
        <div class="tracker-card">
          <span class="tracker-joint-name">Right Hip (CH 0)</span>
          <span class="tracker-angle-big" id="track-0">8°</span>
          <div class="tracker-bar-bg"><div class="tracker-bar-fill" id="bar-0"></div></div>
        </div>
        <div class="tracker-card">
          <span class="tracker-joint-name">Right Knee (CH 1)</span>
          <span class="tracker-angle-big" id="track-1">94°</span>
          <div class="tracker-bar-bg"><div class="tracker-bar-fill" id="bar-1"></div></div>
        </div>
        <div class="tracker-card">
          <span class="tracker-joint-name">Right Foot (CH 2)</span>
          <span class="tracker-angle-big" id="track-2">94°</span>
          <div class="tracker-bar-bg"><div class="tracker-bar-fill" id="bar-2"></div></div>
        </div>
        <div class="tracker-card">
          <span class="tracker-joint-name">Left Hip (CH 6)</span>
          <span class="tracker-angle-big" id="track-6">94°</span>
          <div class="tracker-bar-bg"><div class="tracker-bar-fill" id="bar-6"></div></div>
        </div>
        <div class="tracker-card">
          <span class="tracker-joint-name">Left Knee (CH 4)</span>
          <span class="tracker-angle-big" id="track-4">94°</span>
          <div class="tracker-bar-bg"><div class="tracker-bar-fill" id="bar-4"></div></div>
        </div>
        <div class="tracker-card">
          <span class="tracker-joint-name">Left Foot (CH 5)</span>
          <span class="tracker-angle-big" id="track-5">94°</span>
          <div class="tracker-bar-bg"><div class="tracker-bar-fill" id="bar-5"></div></div>
        </div>
      </div>
    </section>

    <!-- Locomotion Actions -->
    <section class="card">
      <div class="card-title">🚶 Locomotion (Dual-Leg Forward Strides)</div>
      <div class="dpad-container">
        <button class="dpad-btn btn-up" onclick="runGait('walk')">▲<span>Forward</span></button>
        <button class="dpad-btn btn-left" onclick="runGait('left')">◀<span>Turn L</span></button>
        <button class="dpad-btn btn-stop-center" onclick="runGait('stand')">🛑<span>Stop</span></button>
        <button class="dpad-btn btn-right" onclick="runGait('right')">▶<span>Turn R</span></button>
        <button class="dpad-btn btn-down" onclick="runGait('backward')">▼<span>Back</span></button>
      </div>

      <div class="gait-actions">
        <button class="btn" onclick="runGait('walk')" style="border-color: var(--success); color: var(--success); font-weight: 800;">🚀 1. Forward Walk Loop</button>
        <button class="btn" onclick="runGait('step')" style="font-weight: 800;">🐾 2. Single Forward Stride</button>
        <button class="btn" onclick="runGait('stand')" style="border-color: var(--danger); color: #fda4af;">🛑 3. Stand & Silence</button>
      </div>

      <div class="save-actions">
        <button class="btn btn-save" onclick="saveOffsets()">💾 Save Current As Default Stand</button>
        <button class="btn btn-reset" onclick="resetOffsets()">🔄 Reset Factory Angles</button>
      </div>
    </section>

    <!-- Right Leg Servos -->
    <div class="leg-section-title leg-r-title">RIGHT LEG (CH 0: Hip=8°, CH 1: Knee=94°, CH 2: Foot=94°)</div>
    <div class="servo-grid">
      <div class="servo-card">
        <div class="servo-header"><span class="servo-name">Right Hip (CH 0)</span><span class="angle-display" id="angle-0">8 deg</span></div>
        <div class="slider-box"><input type="range" id="slider-0" min="0" max="180" value="8" oninput="setAngle(0, this.value)" onchange="setAngle(0, this.value)"></div>
        <div class="step-buttons">
          <button class="step-btn" onclick="stepAngle(0, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(0, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(0, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(0, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(0, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(0, 10)">+10</button>
        </div>
      </div>
      <div class="servo-card">
        <div class="servo-header"><span class="servo-name">Right Knee (CH 1)</span><span class="angle-display" id="angle-1">94 deg</span></div>
        <div class="slider-box"><input type="range" id="slider-1" min="0" max="180" value="94" oninput="setAngle(1, this.value)" onchange="setAngle(1, this.value)"></div>
        <div class="step-buttons">
          <button class="step-btn" onclick="stepAngle(1, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(1, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(1, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(1, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(1, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(1, 10)">+10</button>
        </div>
      </div>
      <div class="servo-card">
        <div class="servo-header"><span class="servo-name">Right Foot (CH 2)</span><span class="angle-display" id="angle-2">94 deg</span></div>
        <div class="slider-box"><input type="range" id="slider-2" min="0" max="180" value="94" oninput="setAngle(2, this.value)" onchange="setAngle(2, this.value)"></div>
        <div class="step-buttons">
          <button class="step-btn" onclick="stepAngle(2, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(2, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(2, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(2, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(2, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(2, 10)">+10</button>
        </div>
      </div>
    </div>

    <!-- Left Leg Servos -->
    <div class="leg-section-title leg-l-title">LEFT LEG (CH 6: Hip=94°, CH 4: Knee=94°, CH 5: Foot=94°)</div>
    <div class="servo-grid">
      <div class="servo-card">
        <div class="servo-header"><span class="servo-name">Left Hip (CH 6)</span><span class="angle-display" id="angle-6">94 deg</span></div>
        <div class="slider-box"><input type="range" id="slider-6" min="0" max="180" value="94" oninput="setAngle(6, this.value)" onchange="setAngle(6, this.value)"></div>
        <div class="step-buttons">
          <button class="step-btn" onclick="stepAngle(6, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(6, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(6, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(6, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(6, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(6, 10)">+10</button>
        </div>
      </div>
      <div class="servo-card">
        <div class="servo-header"><span class="servo-name">Left Knee (CH 4)</span><span class="angle-display" id="angle-4">94 deg</span></div>
        <div class="slider-box"><input type="range" id="slider-4" min="0" max="180" value="94" oninput="setAngle(4, this.value)" onchange="setAngle(4, this.value)"></div>
        <div class="step-buttons">
          <button class="step-btn" onclick="stepAngle(4, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(4, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(4, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(4, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(4, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(4, 10)">+10</button>
        </div>
      </div>
      <div class="servo-card">
        <div class="servo-header"><span class="servo-name">Left Foot (CH 5)</span><span class="angle-display" id="angle-5">94 deg</span></div>
        <div class="slider-box"><input type="range" id="slider-5" min="0" max="180" value="94" oninput="setAngle(5, this.value)" onchange="setAngle(5, this.value)"></div>
        <div class="step-buttons">
          <button class="step-btn" onclick="stepAngle(5, -10)">-10</button>
          <button class="step-btn" onclick="stepAngle(5, -5)">-5</button>
          <button class="step-btn" onclick="stepAngle(5, -1)">-1</button>
          <button class="step-btn" onclick="stepAngle(5, 1)">+1</button>
          <button class="step-btn" onclick="stepAngle(5, 5)">+5</button>
          <button class="step-btn" onclick="stepAngle(5, 10)">+10</button>
        </div>
      </div>
    </div>
  </div>

  <script>
    let sendTimers = {};

    function sendCmd(url) {
      fetch(url).catch(()=>{});
    }

    function setAngle(ch, val) {
      val = parseInt(val);
      updateUI(ch, val);
      
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
      if (el) el.innerText = val + ' deg';
      if (sl) sl.value = val;
      if (tr) tr.innerText = val + '°';
      if (br) br.style.width = ((val / 180) * 100) + '%';
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

    function saveOffsets() {
      sendCmd('/save_offsets');
      alert('Current Angles Saved Permanently to ESP32 Flash Memory!');
    }

    function resetOffsets() {
      if (confirm('Reset all angles to calibrated factory baselines?')) {
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
        }).catch(()=>{});
    }
    setInterval(pollTelemetry, 350);
    pollTelemetry();
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

void setServoAngleDirect(int channel, float angle) {
  if (!pcaFound) return;
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

void parkStand() {
  gaitRunning = false;
  activeGaitName = "stand";
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = (float)servos[i].homeAngle;
    setServoAngleDirect(servos[i].channel, (float)servos[i].homeAngle);
  }
}

// Alternating Dual-Leg Forward Walking Engine
void updateNonBlockingWalk() {
  float t = (float)(millis() - walkStartTime) / 1000.0f;
  float phase = fmod(2.0f * (float)M_PI * walkFreq * t, 2.0f * (float)M_PI);

  float rhBase = (float)servos[0].homeAngle; // 8°
  float lhBase = (float)servos[3].homeAngle; // 94°

  float aHipR = rhBase;
  float aHipL = lhBase;

  if (phase < (float)M_PI) {
    // Phase 1 (Right Stride): Right Hip reaches forward (8° -> 20° -> 8°), Left holds 94°
    float swingFrac = sin(phase); // 0.0 -> 1.0 -> 0.0
    aHipR = rhBase + (swingFrac * strideAmp);
    aHipL = lhBase;
  } else {
    // Phase 2 (Left Stride): Left Hip reaches forward (94° -> 106° -> 94°), Right holds 8°
    float swingFrac = sin(phase - (float)M_PI); // 0.0 -> 1.0 -> 0.0
    aHipL = lhBase + (swingFrac * strideAmp);
    aHipR = rhBase;
  }

  servos[0].currentAngle = aHipR; setServoAngleDirect(0, aHipR);
  servos[3].currentAngle = aHipL; setServoAngleDirect(6, aHipL);

  // Keep Knees and Feet locked at baselines
  setServoAngleDirect(1, (float)servos[1].homeAngle);
  setServoAngleDirect(2, (float)servos[2].homeAngle);
  setServoAngleDirect(4, (float)servos[4].homeAngle);
  setServoAngleDirect(5, (float)servos[5].homeAngle);
}

// Single Stride
void executeSingleFullStride() {
  unsigned long t0 = millis();
  float periodMs = (1000.0f / walkFreq);
  while (millis() - t0 < (unsigned long)periodMs) {
    walkStartTime = t0;
    updateNonBlockingWalk();
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
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleGetAngles() {
  String json = "{\"angles\":{";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += "\"" + String(servos[i].channel) + "\":" + String((int)servos[i].currentAngle);
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "},\"active_gait\":\"" + activeGaitName + "\",\"gait_running\":" + (gaitRunning ? "true" : "false") + "}";
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
  if (gait == "stand") {
    parkStand();
  } else if (gait == "walk" || gait == "forward") {
    gaitRunning = true;
    activeGaitName = "walk";
    walkStartTime = millis();
  } else if (gait == "step") {
    gaitRunning = false;
    activeGaitName = "forward";
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"forward\"}");
    executeSingleFullStride();
    return;
  }
  server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"" + gait + "\"}");
}

// ==========================================
// 6. Setup & Loop
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- Biped Robot Controller Starting ---");

  loadSavedOffsets();

  Wire.begin(21, 22);
  Wire.setTimeOut(100);
  
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() == 0) {
    pcaFound = true;
    Serial.println("[SUCCESS] PCA9685 found at 0x40!");
    pwm.begin();
    pwm.setPWMFreq(50);
    delay(50);
    parkStand();
  } else {
    Serial.println("[WARNING] PCA9685 not detected on I2C bus.");
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
  server.on("/save_offsets", handleSaveOffsets);
  server.on("/reset_offsets", handleResetOffsets);
  server.on("/run_gait", handleRunGait);
  server.begin();
  Serial.println("HTTP Web Server Started!");
}

void loop() {
  server.handleClient();
  if (gaitRunning && activeGaitName == "walk") {
    updateNonBlockingWalk();
    delay(15);
  }
}
