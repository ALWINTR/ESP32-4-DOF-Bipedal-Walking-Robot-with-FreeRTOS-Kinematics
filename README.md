# 🦾 ESP32 Bipedal Walking Robot (4-DOF Kinematics & FreeRTOS)

[![GitHub Repository](https://img.shields.io/badge/GitHub-Repository-00f0ff?style=for-the-badge&logo=github&logoColor=white)](https://github.com/ALWINTR/esp32-bipedal-robot)
[![Developer](https://img.shields.io/badge/Developer-Alwin_T_R-0284c7?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/alwintr)
[![Platform](https://img.shields.io/badge/Platform-ESP32_Dual--Core_FreeRTOS-38bdf8?style=for-the-badge&logo=espressif&logoColor=white)](https://github.com/ALWINTR)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

A four-degree-of-freedom (4-DOF) autonomous bipedal walking robotic mechanism featuring synchronized hip and ankle inverse kinematics, multi-channel 16-bit hardware PWM timer coordination, dual-core FreeRTOS task scheduling, and real-time serial telemetry streaming.

---

## 📌 Kinematic Architecture & DOF Distribution

The robot features 4 motorized joints symmetrically distributed across two planar leg assemblies:
1. **Left Hip Pitch (Joint 1)**: Controls sagittal plane forward/backward swing trajectory of the left leg.
2. **Left Ankle Roll (Joint 2)**: Stabilizes lateral center-of-mass (CoM) during single-support stance phase.
3. **Right Hip Pitch (Joint 3)**: Provides complementary phase swing trajectory for the right leg.
4. **Right Ankle Roll (Joint 4)**: Executes lateral tilt and ground contact shock dampening.

```
                  ┌──────────────────────────────┐
                  │    ESP32 Dual-Core Engine    │
                  │   Core 0: Serial Telemetry   │
                  │   Core 1: Kinematic Gait FSM │
                  └──────────────┬───────────────┘
                                 │ 16-Bit LEDC PWM (50Hz)
            ┌────────────────────┼────────────────────┐
            ▼                    ▼                    ▼
     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
     │ Left Hip     │     │ Left Ankle   │     │ Right Hip/   │
     │ MG996R Servo │     │ MG996R Servo │     │ Ankle Servos │
     └──────────────┘     └──────────────┘     └──────────────┘
```

---

## ⚙️ Hardware Specifications & Power Delivery

| Subsystem | Component | Engineering Specification |
| :--- | :--- | :--- |
| **Main Processing Unit** | ESP32-WROOM-32 | Tensilica Xtensa Dual-Core LX6 @ 240MHz, 520KB SRAM |
| **Joint Actuators** | 4x TowerPro MG996R | Metal Gear High-Torque Servos (11 kg-cm @ 6.0V, 0.16s/60°) |
| **PWM Resolution** | ESP32 LEDC Timers | 16-Bit Timer Resolution (~0.3µs pulse precision @ 50Hz) |
| **Power Regulation** | 5V/6V 5A High-Power UBEC | Independent dual-rail power isolation with 1000µF filter |
| **Power Source** | 2S 7.4V LiPo Battery | High-discharge 25C lithium-polymer cell |

---

## 🔌 Pinout & Hardware Interface

| Joint Actuator | ESP32 GPIO Pin | LEDC Channel | PWM Frequency | Duty Range |
| :--- | :--- | :--- | :--- | :--- |
| **Left Hip Pitch** | GPIO 18 | Channel 0 | 50Hz | 500µs - 2500µs |
| **Left Ankle Roll** | GPIO 19 | Channel 1 | 50Hz | 500µs - 2500µs |
| **Right Hip Pitch** | GPIO 21 | Channel 2 | 50Hz | 500µs - 2500µs |
| **Right Ankle Roll** | GPIO 22 | Channel 3 | 50Hz | 500µs - 2500µs |
| **Serial Telemetry TX** | GPIO 1 (TX0) | UART0 | 115200 Baud | Angle & phase stream |

---

## 🧠 FreeRTOS Firmware & Gait State Machine

The firmware (`esp32-bipedal-robot.ino`) utilizes FreeRTOS multitasking across both processor cores:
- **Core 1 — High-Priority Kinematics Task (`kinematicsTask`)**: Computes smooth sinusoidal joint trajectory equations at a 20ms update rate, updating LEDC PWM duty cycles without jitter.
- **Core 0 — Telemetry & Communications Task (`telemetryTask`)**: Streams JSON/CSV angle logs over UART at 115200 baud for real-time monitoring on PC.
- **Gait Cycle Phases**:
  1. *Right Stance / Left Swing*: Body tilts right (Ankles -15°), Left Hip advances +25°.
  2. *Double Support Transfer*: Both feet contact ground, CoM shifts toward center.
  3. *Left Stance / Right Swing*: Body tilts left (Ankles +15°), Right Hip advances +25°.
  4. *Push-Off & Forward Motion*: Stance hips push backward, advancing the torso forward.

---

## 📊 Live PowerShell Serial Telemetry

This repository includes live telemetry capture scripts in PowerShell:
- `read_serial.ps1`: Logs raw joint angles and phase states to CSV.
- `read_serial_live.ps1`: Interactive terminal HUD displaying live hip and ankle angles with graphical ASCII progress bars.

```powershell
# Run the live serial telemetry HUD
powershell -ExecutionPolicy Bypass -File read_serial_live.ps1 -Port COM3 -Baud 115200
```

---

## 🚀 Getting Started

1. Clone repository:
   ```bash
   git clone https://github.com/ALWINTR/esp32-bipedal-robot.git
   ```
2. Open `esp32-bipedal-robot.ino` in Arduino IDE or VS Code PlatformIO.
3. Select board **ESP32 Dev Module**.
4. Set CPU Frequency to **240MHz**.
5. Upload firmware and open Serial Monitor at **115200 baud**.

---

## 👨‍💻 Author

**Alwin T R** — Robotics & Automation Engineer  
- 💼 LinkedIn: [linkedin.com/in/alwintr](https://www.linkedin.com/in/alwintr)  
- 🌌 Portfolio: [alwintr.github.io](https://alwintr.github.io)  
- 💻 GitHub: [github.com/ALWINTR](https://github.com/ALWINTR)

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.
