# 🚀 Drone Control System – Master Project

**Test stand for evaluating drone control systems with 4DOF**  
_Master’s Thesis Project by [Michał Rogulski](https://github.com/MRogul)_

---

## 📘 Project Summary

This repository contains the full implementation of a 4-DOF quadrotor control system, including:

• custom-built experimental test stand,

• STM32 firmware for sensor acquisition, control algorithms and motor mixing,

• ESP8266-based web interface for real-time tuning,

• MATLAB/Simulink models used for controller prototyping and simulation,

• a complete validation pipeline comparing the mathematical model with real measurements.

The project was developed as part of a Master’s Thesis in Control Engineering.
---

## 📸 Real Test Stand

| Final Stand | Initial Concept |
|:------------:|:----------------:|
| ![4DOF Stand](images/stanowisko_irl.jpg) | ![Concept](images/DroneTestRig.png) |
| **4DOF Research Stand (Final version)** | **Initial design of the test rig** |

---

## ⚙️ Project Highlights

✨ **Embedded Control System (STM32)**

• Four PID loops: roll, pitch, yaw, altitude (Z)

• Derivative-on-measurement (reduces derivative kick)

• Anti-windup (conditional integration)

• Mixer for quadcopter X-configuration

• Digital ESC control via DShot300

🧠 **Sensor Fusion Algorithms**

• Complementary filter (custom implementation)

• Kalman filter (custom implementation)

• BNO055 onboard fusion (BSX) as reference

🌐 **Real-Time Tuning via ESP8266**

• Web UI hosted directly on ESP8266 (AP mode)

• Live PID tuning

• Real-time telemetry

• No need to reflash STM32 to change parameters

🛠️ **Simulation & Validation**

• 4DOF Simulink model (Plant + Sensors + Mixer + PID)

• Comparison against real test stand

• Step-response evaluation for all controlled axes

• Analysis of estimator influence on closed-loop behaviour

---

## 🗂 Repository Structure

Drone_Control_System_Master_Project/

│

├── Drone_Control/          # STM32 code (IMU, sonar, PID, mixer, DShot)

├── ESP8266/                # Web interface for real-time tuning

├── MATLAB/                 # 4DOF simulation model

├── CAD files/              # CAD files of the test stand

├── Library/BNO055/         # BNO055 Library

├── images/                 # Photos and diagrams

├── Datasheets/             # Helpful materials

├── Results/                # Responses and comparisons

└── README.md

---

## 🧠 Technologies Used

| Category | Tools |
|-----------|-------|
| Microcontroller | STM32L432KC |
| IDE | STM32CubeMX |
| Communication | ESP8266 (Wi-Fi, Web Interface) |
| Simulation | MATLAB / Simulink |
| Visualization | STM32CubeMonitor |
| Mechanical Design | Fusion 360 / Bambu Studio |

---

## 🧪 Experimental Results

Tests performed on the 4DOF stand include:

• step responses for roll, pitch, yaw and vertical altitude (Z)

• comparison of Complementary, Kalman and BNO055 fusion

• PID tuning experiments in simulation and on hardware

• validation of control structure and estimator performance

All implemented algorithms (PID + sensor fusion) behave correctly and consistently across simulation and physical testing.

---

## 🪶 About the Project

This platform demonstrates how a complete UAV stabilization system can be built using accessible components, providing:

• safe indoor flight testing,

• repeatable laboratory experiments,

• a foundation for research on estimation & control,

• a practical educational tool for UAV control engineering.


---

## 📄 License

This project is licensed under the **MIT License**.  
You are free to use and modify it for academic or research purposes.

---

**Author:** [Michał Rogulski](https://github.com/MRogul)  
🎓 Warsaw University of Technology  
📘 [Drone_Control_System_Master_Project](https://github.com/MRogul/Drone_Control_System_Master_Project)
