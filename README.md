# ESP32 4 DOF Bipedal Walking Robot with FreeRTOS Kinematics

[![GitHub Repository](https://img.shields.io/badge/GitHub-Repository-00f0ff?style=for-the-badge&logo=github&logoColor=white)](https://github.com/ALWINTR/esp32-bipedal-robot)
[![Developer](https://img.shields.io/badge/Developer-Alwin_T_R-0284c7?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/alwintr)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

A four-degree-of-freedom (4-DOF) autonomous bipedal walking robotic mechanism featuring synchronized hip and ankle inverse kinematics, multi-channel 16-bit hardware PWM timer coordination, dual-core FreeRTOS task scheduling, and real-time serial telemetry streaming.

---

## Kinematic Architecture and Degrees of Freedom

The robot features 4 motorized joints symmetrically distributed across two planar leg assemblies:
1. **Left Hip Pitch (Joint 1)**: Controls sagittal plane forward and backward swing trajectory of the left leg.
2. **Left Ankle Roll (Joint 2)**: Stabilizes lateral center-of-mass (CoM) during single-support stance phase.
3. **Right Hip Pitch (Joint 3)**: Provides complementary phase swing trajectory for the right leg.
4. **Right Ankle Roll (Joint 4)**: Executes lateral tilt and ground contact shock dampening.

```
                  +------------------------------+
                  |    ESP32 Dual Core Engine    |
                  |   Core 0: Serial Telemetry   |
                  |   Core 1: Kinematic Gait FSM |
                  +--------------+---------------+
                                 | 16-Bit LEDC PWM (50Hz)
            +--------------------+--------------------+
            v                    v                    v
     +--------------+     +--------------+     +--------------+
     | Left Hip     |     | Left Ankle   |     | Right Hip/   |
     | MG996R Servo |     | MG996R Servo |     | Ankle Servos |
     +--------------+     +--------------+     +--------------+
```

---

## Hardware Specifications

| Subsystem | Component | Engineering Specification |
| :--- | :--- | :--- |
| **Main Processing Unit** | ESP32-WROOM-32 | Tensilica Xtensa Dual Core LX6 @ 240MHz, 520KB SRAM |
| **Joint Actuators** | 4x TowerPro MG996R | Metal Gear High-Torque Servos (11 kg-cm @ 6.0V) |
| **PWM Resolution** | ESP32 LEDC Timers | 16-Bit Timer Resolution (~0.3 microsecond precision @ 50Hz) |
| **Power Regulation** | 5V/6V 5A High-Power UBEC | Dedicated dual-rail power isolation with 1000 microfarad capacitor |
| **Power Source** | 2S 7.4V LiPo Battery | High-discharge 25C lithium-polymer cell |

---

## Circuit Pinout Table

| Joint Actuator | ESP32 GPIO Pin | LEDC Channel | PWM Frequency | Duty Range |
| :--- | :--- | :--- | :--- | :--- |
| **Left Hip Pitch** | GPIO 18 | Channel 0 | 50Hz | 500 to 2500 microseconds |
| **Left Ankle Roll** | GPIO 19 | Channel 1 | 50Hz | 500 to 2500 microseconds |
| **Right Hip Pitch** | GPIO 21 | Channel 2 | 50Hz | 500 to 2500 microseconds |
| **Right Ankle Roll** | GPIO 22 | Channel 3 | 50Hz | 500 to 2500 microseconds |
| **Serial Telemetry** | GPIO 1 (TX0) | UART0 | 115200 Baud | Real-time angle telemetry |

---

## FreeRTOS Firmware Implementation

The firmware (`esp32-bipedal-robot.ino`) utilizes FreeRTOS multitasking across both processor cores:
- **Core 1 - High-Priority Kinematics Task**: Computes smooth sinusoidal joint trajectory equations at a 20ms update rate, updating LEDC PWM duty cycles without jitter.
- **Core 0 - Telemetry Task**: Streams joint angle status and gait cycle phases over UART at 115200 baud for real-time monitoring.
- **Gait Cycle Phases**:
  1. *Right Stance / Left Swing*: Lateral tilt to the right, advancing the left leg.
  2. *Double Support Transfer*: Both feet on ground, shifting center-of-mass forward.
  3. *Left Stance / Right Swing*: Lateral tilt to the left, advancing the right leg.
  4. *Push-Off*: Stance hips push backward, propelling torso forward.

---

## Author

**Alwin T R** - Robotics and Automation Engineer  
- LinkedIn: [linkedin.com/in/alwintr](https://www.linkedin.com/in/alwintr)  
- Portfolio: [alwintr.github.io](https://alwintr.github.io)  
- GitHub: [github.com/ALWINTR](https://github.com/ALWINTR)

---

## License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
