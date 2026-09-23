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

## 📐 Hardware Architecture & 38-Pin ESP32 Mapping

```
                 +-----------------------------------+
                 |      ESP32 38-Pin DevKit MCU      |
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

### 38-Pin ESP32 to PCA9685 Wiring

```
   38-Pin ESP32 DevKit                       PCA9685 16-Ch Driver
+------------------------+                  +--------------------+
|                [3V3]   |                  |                    |
|                [GND] --+----------------->| [GND] (Logic GND)  |
|                [VIN] --+----------------->| [VCC] (Logic 3.3-5V|
|                [D21] --+----------------->| [SDA] (I2C Data)   |
|                [D22] --+----------------->| [SCL] (I2C Clock)  |
+------------------------+                  +--------------------+
                                                      |
 External Battery / 5V-6V (3A+) --------------------> | [V+] / [GND] (Servo Power)
```

| ESP32 38-Pin Header | Label on Board | PCA9685 Pin | Function |
| :--- | :--- | :--- | :--- |
| **Right Side (Pin 2)** | `D21` (GPIO 21) | `SDA` | I2C Data (400kHz Fast Mode) |
| **Left Side (Pin 17)** | `D22` (GPIO 22) | `SCL` | I2C Clock (400kHz Fast Mode) |
| **Left Side (Pin 1)**  | `3V3` / `VIN`   | `VCC` | Logic Power (3.3V or 5V) |
| **Left / Right**       | `GND`           | `GND` | Common Ground Reference |
| **Power Screw Terminal** | *External 5V-6V* | `V+` / `GND` | High-Current Dedicated Servo Rail |

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
