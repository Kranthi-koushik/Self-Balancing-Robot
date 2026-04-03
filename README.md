# Self-Balancing Robot

A two-wheel self-balancing robot built using PID control and IMU sensor for real-time stabilization. This project was developed as part of a Control Systems semester project.

## 🚀 Overview
The robot maintains balance by continuously adjusting motor speed based on tilt angle feedback, similar to systems like Segway.

## ⚙️ Hardware Used
- Arduino (Uno)
- MPU6050 IMU (Accelerometer + Gyroscope)
- L298N Motor Driver
- DC Geared Motors
- Li-ion Battery
- Custom Chassis

## 🧠 Working Principle
- IMU measures tilt angle
- Error calculated from vertical position
- PID controller computes correction
- Motors adjust speed to maintain balance

## 🔧 Key Concepts
- Closed-loop control system
- PID control
- Sensor fusion
- Real-time feedback systems

## 📈 Challenges
- Sensor noise and drift
- PID tuning (overshoot, oscillations)
- Mechanical balancing issues

## 📚 Learnings
- Practical control systems implementation
- Real-time system debugging
- Hardware-software integration

## 👥 Team Members
- M. Anvesh Kumar Reddy  
- M. Shanmukha Vinayak  
- A. Rakesh  
- A. Gowtham  
*(Developed collaboratively as part of a Control Systems semester project)*
