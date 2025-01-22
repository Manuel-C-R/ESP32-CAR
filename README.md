# Design of a Low-Cost Robotic Platform

This repository contains the design and implementation details of a robotic platform: a four-wheeled robot car powered by DC motors. The project focuses on developing a versatile and functional robotic system capable of real-time operations.

This project has been created for the subject Industrial Informatics 2024/25, of the B.S. Industrial Electronics Engineering at the University of Malaga.

<div style="display: flex; justify-content: center; align-items: center; gap: 50px;">
  <img src="images/FrontView.jpg" alt="Front view" width="300">
  <img src="images/SideView.jpg" alt="Side view" width="300">
</div>

## Features

- **Omnidirectional Movement**: The robot can be controlled to move in any direction.
- **Distance Measurement**: Equipped with sensors to measure distances to various points in its environment.
- **Real-Time Image Transmission**: Capable of capturing and transmitting images in real time.
- **Communication Protocols**:
  - Wi-Fi and MQTT for data transmission.
  - Bluetooth integration for control via an Android application.

<img src="images/Radar.png" alt="Radar." width="500">

## Communication Architecture

1. **Wi-Fi & MQTT**: Handles data exchange between the robot and a server or other devices.
2. **Bluetooth**: Facilitates direct control via a custom Android application.

<img src="images/Architecture.png" alt="Architecture." width="500">

## Android Application

The project includes a dedicated Android application to:

- Control the robot's movement.
- Visualize real-time data and images.

<div style="display: flex; justify-content: center; align-items: center; gap: 20px;">
  <img src="images/app1.jpg" alt="App Screenshot 1" width="200">
  <img src="images/app2.jpg" alt="App Screenshot 2" width="200">
  <img src="images/app3.jpg" alt="App Screenshot 3" width="200">
  <img src="images/app4.jpg" alt="App Screenshot 4" width="200">
</div>
