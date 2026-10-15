# ESP32 4 DOF Bipedal Walking Robot with FreeRTOS Kinematics

<div align="center">

[![GitHub Repository](https://img.shields.io/badge/GitHub-Repository-00f0ff?style=for-the-badge&logo=github&logoColor=white)](https://github.com/ALWINTR/ESP32-4-DOF-Bipedal-Walking-Robot-with-FreeRTOS-Kinematics)
[![Developer](https://img.shields.io/badge/Developer-Alwin_T_R-0284c7?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/alwintr)
[![Portfolio](https://img.shields.io/badge/Portfolio-alwintr.github.io-00f0ff?style=for-the-badge&logo=googlechrome&logoColor=black)](https://alwintr.github.io)
[![License](https://img.shields.io/badge/License-MIT-10b981?style=for-the-badge)](LICENSE)

</div>

> Four-DOF bipedal walking robot featuring synchronized hip and ankle inverse kinematics, multi-channel 16-bit hardware PWM timers, and real-time serial telemetry running on FreeRTOS Core 1.

---

## 📌 Executive Overview

Four-DOF bipedal walking robot featuring synchronized hip and ankle inverse kinematics, multi-channel 16-bit hardware PWM timers, and real-time serial telemetry running on FreeRTOS Core 1. Designed and validated on physical hardware, this robotics architecture implements multi-joint forward and inverse kinematics, 16-bit hardware PWM pulse generation, real-time serial telemetry streams, and alternating knee gait phases to maintain dynamic stability and prevent joint strain.

- **Primary Domain**: Robotics
- **Microcontroller Core**: ESP32 / FreeRTOS
- **Kinematics Engine**: Inverse Kinematics (IK) & Synchronized Joint Phase Planning
- **Live Interactive Specs**: [alwintr.github.io](https://alwintr.github.io)

---

## 📐 System Architecture & Signal Flow

```
+--------------------------------------------------------------+
|                ESP32 Dual-Core SoC (240MHz)                  |
|  [Core 0: Telemetry & Comm]   |   [Core 1: FreeRTOS Gait Task]|
+--------------------------------------------------------------+
                                |
                                v Sinusoidal Gait Trajectory Generator
        +-----------------------+-----------------------+
        |                       |                       |
        v                       v                       v
  [LEDC Channel 0]        [LEDC Channel 1]        [LEDC Channels 2,3]
        |                       |                       |
        v                       v                       v
 [Right Hip MG996R]      [Right Foot MG996R]     [Left Hip & Foot MG996R]
```

---

## 📋 Hardware Bill of Materials (BOM)

| Component Name | Operating Voltage | Interface / Functionality |
| :--- | :--- | :--- |
| **ESP32-WROOM-32D** | `3.3V / 5V` | Dual Xtensa 240MHz, FreeRTOS |
| **4x TowerPro MG996R Servos** | `6.0V / 2.5A peak` | Metal gear, 13 kg-cm torque |
| **External UBEC 6V / 5A Buck** | `7.4V 2S LiPo -> 6V` | Isolated actuator power |
| **MPU-6050 6-Axis IMU** | `3.3V I2C` | Real-time posture feedback |

---

## 🔌 Circuit Pinout & Microcontroller Interconnect

| MCU Pin / Channel | Peripheral Pin | Signal Function / Bus |
| :--- | :--- | :--- |
| `ESP32 GPIO 18` | `Left Hip Servo PWM` | LEDC Timer 0, Channel 0 |
| `ESP32 GPIO 19` | `Left Foot Servo PWM` | LEDC Timer 0, Channel 1 |
| `ESP32 GPIO 22` | `Right Hip Servo PWM` | LEDC Timer 1, Channel 2 |
| `ESP32 GPIO 23` | `Right Foot Servo PWM` | LEDC Timer 1, Channel 3 |
| `ESP32 GPIO 21 / 22` | `MPU-6050 SDA / SCL` | I2C Bus at 400kHz |

---

## ⚙️ Firmware Architecture & Locomotion Kinematics

Dual-core FreeRTOS firmware dedicating Core 1 to high-priority sinusoidal inverse kinematics and gait trajectory generation, while Core 0 handles serial telemetry streaming.

### Locomotion & Gait Features:
- **Strictly Alternating Knee Locomotion**: Knee actuation phases are synchronized to alternate strictly so both knees never flex simultaneously, preserving center-of-mass equilibrium.
- **Dynamic Hip-Knee Proportional Scaling**: Dynamically maps delta hip trajectory change to knee pitch adjustment (`dKnee == dHip`) while maintaining flat foot alignment.
- **Dual-Core FreeRTOS Task Distribution**: Core 0 manages high-speed wireless telemetry while Core 1 executes deterministic 16-bit microsecond PWM timing.

---

## 📁 Repository Structure

```
ESP32-4-DOF-Bipedal-Walking-Robot-with-FreeRTOS-Kinematics/
├── .gitignore
├── connect_wifi.ps1
├── include
├── LICENSE
├── platformio.ini
├── read_serial.ps1
├── read_serial_live.ps1
├── src
├── test_hip_knee.py
└── README.md
```

---

## 🚀 Build & Deployment Instructions

### 1. Hardware Power Distribution
- Supply high-torque digital servos with a dedicated 5V–6V 3A+ BEC power rail.
- Establish a shared common ground between the external servo power supply and the ESP32 ground pin.

### 2. Compilation & Flashing
```bash
# Clone the repository
git clone https://github.com/ALWINTR/ESP32-4-DOF-Bipedal-Walking-Robot-with-FreeRTOS-Kinematics.git

# Navigate into directory
cd ESP32-4-DOF-Bipedal-Walking-Robot-with-FreeRTOS-Kinematics
```
- Open `esp32-bipedal-robot.ino` in **Arduino IDE** or **VS Code with PlatformIO**.
- Select `ESP32 Dev Module` under Board Manager.
- Compile and flash to the ESP32 microcontroller over USB-UART.

---

## 👨‍💻 Author

**Alwin T R** — Robotics & Automation Engineer  
- 💼 **LinkedIn**: [linkedin.com/in/alwintr](https://www.linkedin.com/in/alwintr)  
- 🌐 **Interactive Portfolio**: [alwintr.github.io](https://alwintr.github.io)  
- 🐙 **GitHub**: [github.com/ALWINTR](https://github.com/ALWINTR)  
- ✉️ **Direct Email**: [alwintr2003@gmail.com](mailto:alwintr2003@gmail.com)

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.
