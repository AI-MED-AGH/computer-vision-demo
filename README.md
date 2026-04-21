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

## Hardware Requirements
To run this project, you will need the following hardware:

- Sensor: Microsoft Kinect for Windows v2.
- Adapter: Kinect Adapter for Windows.


## System Architecture (Hybrid Setup)
Due to driver limitations, the system operates across two environments:
1.  **Windows:** Responsible for Kinect V2 sensor data processing and hand tracking via a C# bridge.
2.  **Linux (WSL/VM):** Runs the Drake simulation, Inverse Kinematics solver, and Meshcat visualization.
3.  **Communication:** Data is sent from Windows to Linux via **UDP protocol**.

---

## How to Run

### 1. Linux Setup (Simulation)
- Python 3.10+

1.  **Prepare the environment:**
    ```bash
    git clone <your-repo-url>
    cd computer-vision-demo
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    ```
2.  **Identify your IP address:**
    Run the following command to find the IP needed for the C# bridge:
    ```bash
    hostname -I
    ```
3.  **Launch the simulation:**
    ```bash
    python src/main.py
    ```

### 2. Windows Setup (Hand Tracking Bridge)
1.  **Drivers:** Install *Kinect for Windows SDK 2.0*.
2.  **C# Script:** Use the provided Console Application (see code below) to send coordinates to Linux.
3.  **Configuration:** Set the `targetIp` variable in the C# script to the IP obtained in the previous step.
4.  **Run:** Compile and start the script to begin streaming hand coordinates.

---

## C# Bridge Script (Windows Side)
This script captures hand data (placeholder for Kinect SDK) and sends it to the Linux simulation via UDP.

The `script.cs` file provided in this repository is a standalone C# script. 
It initializes the Kinect V2 sensor, tracks the user's hand position, and packages the $(X, Y, Z)$ 
coordinates into a UDP packet.
