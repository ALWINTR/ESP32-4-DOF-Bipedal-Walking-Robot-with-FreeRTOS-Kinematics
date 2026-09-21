# ESP32 6-DOF / 4-DOF Bipedal Walking Robot with Zero-Lag Kinematics & Web Studio

[![GitHub Repository](https://img.shields.io/badge/GitHub-Repository-00f0ff?style=for-the-badge&logo=github&logoColor=white)](https://github.com/ALWINTR/ESP32-4-DOF-Bipedal-Walking-Robot-with-FreeRTOS-Kinematics)
[![Developer](https://img.shields.io/badge/Developer-Alwin_T_R-0284c7?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/alwintr)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

An autonomous, WiFi-enabled **Humanoid Bipedal Walking Robotic System** powered by **ESP32** and **PCA9685 16-Channel 12-Bit PWM Driver**. Featuring continuous zero-lag S-Curve harmonic locomotion, lateral weight-shift balance, soft-touchdown ground landing kinematics ($v=0$), multi-directional D-Pad navigation, and an embedded real-time web calibration studio with NVS flash memory persistence.

---

## 🌟 Key Engineering Features

- **Advanced Weight-Shift Locomotion**:
  - Full lateral weight shifting (Ankle Roll) enabling true mid-air leg swing without ground friction.
  - S-Curve harmonic trajectory easing ($1.0 - 0.18\cos(2\theta)$) eliminating body vibrations.
  - Parabolic soft-touchdown knee trajectory with zero vertical impact velocity ($v=0$).
- **Zero-Lag Real-Time Web Studio**:
  - Embedded modern dark-mode responsive web app hosted directly on the ESP32 HTTP server.
  - Live joint telemetry bars, D-Pad locomotion, and real-time gait tuning sliders.
  - 400kHz Fast I2C communication reducing PCA9685 servo command latency by 75%.
- **Multi-Directional Locomotion**:
  - Forward Walking, Reverse Stepping, Left Pivot Turning, and Right Pivot Turning.
  - Anti-jerk dynamic startup and stop ramping over 500ms.
- **Hardware Interference & Bracket Collision Prevention**:
  - Strict software bounding and clearance-safe kinematics ensuring servo brackets never jam.
- **Permanent Flash Calibration**:
  - Save calibrated neutral offsets directly to ESP32 NVS Flash memory (`Preferences`).

---

## 📐 Kinematic Architecture & Hardware Interfacing

```
                 +-----------------------------------+
                 |           ESP32 MCU               |
                 |  (WiFi AP + Station Web Server)   |
                 +-----------------+-----------------+
                                   | I2C (GPIO 21 / GPIO 22) @ 400kHz
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
   - CH 2: Right Foot  (Neutral: 90°)                  - CH 5: Left Foot  (Neutral: 90°)
```

### Hardware Specifications

| Subsystem | Component | Engineering Specification |
| :--- | :--- | :--- |
| **Main Processing Unit** | ESP32-WROOM-32 | Tensilica Xtensa Dual Core LX6 @ 240MHz, 520KB SRAM |
| **PWM Servo Driver** | PCA9685 | 16-Channel 12-Bit I2C PWM Controller (Address `0x40`) |
| **I2C Bus Speed** | Fast-Mode I2C | 400kHz Bus Clock Frequency (Low-Latency Actuation) |
| **Joint Actuators** | TowerPro MG996R / SG90 | Metal Gear High-Torque Servos |
| **Power Regulation** | 5V/6V 5A High-Power UBEC | Dedicated dual-rail power isolation with decoupling capacitor |
| **Power Source** | 2S 7.4V LiPo / External 5V 3A+ | High-discharge power supply |

### Circuit Pinout Table

| ESP32 Pin | PCA9685 Pin | Function |
| :--- | :--- | :--- |
| `GPIO 21` | `SDA` | I2C Data Line |
| `GPIO 22` | `SCL` | I2C Clock Line |
| `3.3V` / `5V` | `VCC` | Logic Power |
| `GND` | `GND` | Common Ground |
| *External 5V-6V (3A+)* | `V+` / `GND` | High-Current Servo Power Rail |

---

## 🎛️ Web Dashboard & REST API Reference

Once connected to your local WiFi or the ESP32 Access Point (`ESP32-Biped-Robot` / `12345678`), open `http://<ESP32_IP>` in any browser.

| Endpoint | Method | Parameters | Description |
| :--- | :--- | :--- | :--- |
| `/` | `GET` | — | Renders the full 6-DOF Web Calibration Studio |
| `/get_angles` | `GET` | — | Returns JSON telemetry of all joint angles & active gait state |
| `/set_servo` | `GET` | `ch=<channel>&val=<angle>` | Moves a single joint directly in real time |
| `/run_gait` | `GET` | `gait=<walk\|backward\|left\|right\|step\|stand>` | Triggers multi-directional locomotion modes |
| `/set_gait_param`| `GET` | `param=<tilt\|knee\|stride\|freq>&val=<val>` | Dynamically tunes gait parameters in real time |
| `/save_offsets` | `GET` | — | Persists current calibrated angles to ESP32 Flash memory |
| `/reset_offsets`| `GET` | — | Restores factory neutral joint baselines |

---

## 🚀 Compilation and Flashing

### Requirements
- [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli`
- ESP32 Board Package (`esp32:esp32:esp32`)
- Required Libraries:
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

## 👨‍💻 Author

**Alwin T R** - Robotics and Automation Engineer  
- LinkedIn: [linkedin.com/in/alwintr](https://www.linkedin.com/in/alwintr)  
- Portfolio: [alwintr.github.io](https://alwintr.github.io)  
- GitHub: [github.com/ALWINTR](https://github.com/ALWINTR)

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
