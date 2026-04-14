# Computer vision demo

## Overview


This project implements a real-time robotic arm/gripper simulation controlled by hand tracking input. The system maps human hand motion to a robot end-effector using inverse kinematics and visualizes the result in a 3D environment.

The project is designed as both a simulation and a foundation for real hardware control via Arduino.

The robot model used in this project is based on the ready-made Moveo arm/gripper URDF from:

`https://github.com/jesseweisberg/moveo_ros`

This repository is used as the base robot description for the simulation and can be further modified to match the project needs.

---

## Features

- Real-time hand tracking (Kinect)
- Inverse kinematics using Drake
- Smooth trajectory filtering
- 3D visualization using Meshcat
- Optional Arduino integration for physical robot control
- Modular architecture for easy extension

---

## Tech Stack

- numpy
- opencv-python
- pykinect2
- drake
- pyyaml
- scipy
- pyserial
- matplotlib
- filterpy
- pyQt5
- PyOpenGL
- pyqtgraph

---


## How It Works

1. Hand Tracking  
   The system captures hand position using MediaPipe or Kinect.

2. Coordinate Mapping  
   The hand position is transformed into the robot coordinate system and constrained to workspace limits.

3. Filtering  
   The signal is smoothed using exponential moving average and outlier rejection.

4. Inverse Kinematics  
   Drake computes joint angles required to reach the target position.

5. Simulation  
   The robot is updated and visualized in Meshcat.

6. Hardware (Optional)  
   Joint angles are sent via serial to Arduino for real robot control.

---

## Installation

### Requirements

- Python 3.10+
- Virtual environment recommended (WSL)

### Setup

```bash
git clone <your-repo-url>
cd computer-vision-demo

python3 -m venv .venv
source .venv/bin/activate  # Linux / WSL
