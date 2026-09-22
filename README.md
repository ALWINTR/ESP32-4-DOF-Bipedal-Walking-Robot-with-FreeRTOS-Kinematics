# 🤖 ESP32 6-DOF Bipedal Walking Robot Controller

An advanced, WiFi-enabled **6-DOF Humanoid Bipedal Robot Controller** built with **ESP32** and **PCA9685 16-Channel 12-bit PWM Servo Driver**. Featuring continuous zero-lag harmonic locomotion, S-Curve anti-shake weight shifting, soft-touchdown ground landing kinematics, and a real-time web-based calibration & locomotion studio with non-volatile NVS flash persistence.

---

## 🌟 Key Features

- **6-DOF Humanoid Kinematics**:
  - Full lateral weight shifting (Ankle Roll) enabling true mid-air leg swing without ground drag.
  - S-Curve harmonic trajectory easing ($1.0 - 0.18\cos(2\theta)$) eliminating body jitter.
  - Parabolic soft-touchdown knee trajectory with zero vertical impact velocity ($v=0$).
- **Zero-Lag Real-Time Web Studio**:
  - Embedded modern dark-mode responsive web app hosted directly on ESP32.
  - Live 6-joint telemetry bars, D-Pad locomotion, and real-time gait tuning sliders.
  - 400kHz Fast I2C communication reducing PCA9685 servo command latency by 75%.
- **Multi-Directional Locomotion**:
  - Forward Walking, Reverse Stepping, Left Pivot Turning, and Right Pivot Turning.
  - Anti-jerk dynamic startup and stop ramping over 500ms.
- **Hardware Interference Prevention**:
  - Strict software bounding and bracket collision prevention algorithms.
- **Permanent Flash Calibration**:
  - Save calibrated neutral offsets directly to ESP32 NVS Flash memory (`Preferences`).

---

## 📐 Hardware Architecture & Channel Mapping

```
                 +-----------------------------------+
                 |           ESP32 MCU               |
                 |  (WiFi AP + Station Web Server)   |
                 +-----------------+-----------------+
                                   | I2C (GPIO21 / GPIO22) @ 400kHz
                 +-----------------v-----------------+
                 |       PCA9685 16-Ch PWM Driver    |
                 |             (Address 0x40)        |
                 +-----------------+-----------------+
                                   |
         +-------------------------+-------------------------+
         |                                                   |
   [ RIGHT LEG ]                                       [ LEFT LEG ]
   - CH 0: Right Hip   (Neutral: 94°)                  - CH 4: Left Hip   (Neutral: 94°)
   - CH 1: Right Knee  (Neutral: 8°)                   - CH 8: Left Knee  (Neutral: 94°)
   - CH 2: Right Foot  (Neutral: 90°)                  - CH 12: Left Foot (Neutral: 90°)
```

### Pinout Connections

| ESP32 Pin | PCA9685 Pin | Function |
| :--- | :--- | :--- |
| `GPIO 21` | `SDA` | I2C Data Line |
| `GPIO 22` | `SCL` | I2C Clock Line |
| `3.3V` / `5V` | `VCC` | Logic Power |
| `GND` | `GND` | Common Ground |
| *External 5V-6V (3A+)* | `V+` / `GND` | High-Current Servo Power |

---

## 🎛️ Web Dashboard & API Endpoints

Once connected to your local WiFi or the ESP32 Access Point (`ESP32-Biped-Robot` / `12345678`), open `http://<ESP32_IP>` in any browser.

### REST API Reference

| Endpoint | Method | Parameters | Description |
| :--- | :--- | :--- | :--- |
| `/` | `GET` | — | Renders the full 6-DOF Web Calibration Studio |
| `/get_angles` | `GET` | — | Returns JSON telemetry of all 6 joint angles & gait state |
| `/set_servo` | `GET` | `ch=<channel>&val=<angle>` | Moves a single joint directly in real time |
| `/run_gait` | `GET` | `gait=<walk\|backward\|left\|right\|step\|stand>` | Triggers locomotion modes |
| `/set_gait_param`| `GET` | `param=<tilt\|knee\|stride\|freq>&val=<val>` | Dynamically tunes gait parameters in real time |
| `/save_offsets` | `GET` | — | Persists current angles to ESP32 Flash memory |
| `/reset_offsets`| `GET` | — | Restores factory neutral joint baselines |

---

## 🚀 Getting Started & Flashing

### Requirements
- [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli`
- ESP32 Board Package (`esp32:esp32:esp32`)
- Libraries:
  - `Adafruit PWMServoDriver`
  - `Wire`
  - `WiFi`
  - `WebServer`
  - `Preferences`

### Flashing via Arduino CLI
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --upload -p COM3 esp32-bipedal-robot
```

---

## 📄 License
This project is open-source and available under the [MIT License](LICENSE).
