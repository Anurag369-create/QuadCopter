# QuadCopter

# Experimental Quadcopter – Teensy + FS-i6 (PPM Mode)

This project is an **experimental quadcopter flight controller** built around a **Teensy microcontroller** and the **FlySky FS-i6 transmitter/receiver system**.  
The goal is to design, test, and document a custom control stack for multirotors, focusing on **lightweight firmware**, **sensor fusion**, and **PPM-based receiver decoding**.

---

## 🛠 Hardware

- **MCU**: Teensy (e.g., Teensy 4.0 / 4.1, ARM Cortex-M7)  
- **Transmitter/Receiver**: FlySky FS-i6 + FS-iA6B in **PPM mode**  
- **IMU Sensor**: MPU6050 / MPU9250 (for gyro + accelerometer data)  
- **Motors**: Brushless DC motors with ESCs  
- **Power**: LiPo battery (3S/4S depending on design)  
- **Frame**: 250–450mm experimental quadcopter frame  

---

## 🎛 Features (Planned / In Progress)

- ✅ **PPM input decoding** from FS-i6 receiver (all channels over a single pin)  
- ✅ **Kalman filter** for sensor fusion (accelerometer + gyroscope)  
- ✅ **Angle mode** (basic self-leveling)  
- 🚧 **Rate mode** (manual acro control)  
- 🚧 **PID tuning interface**  
- 🚧 **Failsafe / arming logic**  
- 🚧 **Experimental autonomous features**  
---

## 📜 License
This project is licensed under the [MIT License](LICENSE).

---
