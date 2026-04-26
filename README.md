<div align="center">

```
███╗   ██╗██╗██╗  ██╗██╗██████╗  ██████╗ 
████╗  ██║██║██║ ██╔╝██║██╔══██╗██╔═══██╗
██╔██╗ ██║██║█████╔╝ ██║██████╔╝██║   ██║
██║╚██╗██║██║██╔═██╗ ██║██╔══██╗██║   ██║
██║ ╚████║██║██║  ██╗██║██║  ██║╚██████╔╝
╚═╝  ╚═══╝╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═╝ ╚═════╝ 
```

### **Autonomous Mobile Robot Simulation Framework**
*ROS 2 · Gazebo · Nav2 · SLAM · MyCobot Arm*

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge)](https://classic.gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-Stack-green?style=for-the-badge)](https://nav2.ros.org/)
[![OpenCV](https://img.shields.io/badge/Open-CV-red?style=for-the-badge)](https://opencv.org/)


</div>

---

## ⚡ What is Nikiro?

**Nikiro** is a full-stack ROS 2 simulation framework for an **Autonomous Mobile Robot (AMR)** with a **MyCobot 6-DOF robotic arm**. Built for indoor navigation using a differential drive system, it bundles everything — from physics simulation to path planning — into one modular, extensible package.

> 🔬 Designed for research, development, and testing without ever needing physical hardware.

---

## 🎯 Core Capabilities

| Module | Description |
|--------|-------------|
| 🗺️ **SLAM** | Real-time map building via `slam_toolbox` |
| 📍 **Localization** | AMCL + Nav2 lifecycle nodes |
| 🧭 **Path Planning** | Full Nav2 stack — obstacle avoidance, goal reaching |
| 🏭 **Conveyor System** | IR sensor-based automation + multi-object spawning |
| 🦾 **MyCobot Arm** | 6-DOF manipulation with pick-and-place |
| 👁️ **Object Detection** | Integrated visual detection support |
| 🤖 **Multi-Robot** | Scalable architecture for multiple simultaneous robots |
| 📡 **RViz2** | Real-time visualization of robot state, sensors & maps |

---

## 🚀 Getting Started

### Installation

```bash
mkdir -p nikiro_simulation/src
cd nikiro_simulation/src
git clone https://github.com/logesh1516/Nikiro_simulation.git
cd ..
colcon build
```

### Setup Environment

```bash
cd nikiro_simulation
source install/setup.bash
```

---

## 🤖 AMR Simulation

Launch the robot in Gazebo:
```bash
ros2 launch amr_description gazebo.launch.py
```

Build a map of the environment (SLAM):
```bash
ros2 launch amr_description mapping.launch.py
```

Autonomous navigation with Nav2:
```bash
ros2 launch amr_description nav2.launch.py
```

---

## 🏭 Conveyor Belt Simulation

### Basic Launch
```bash
cd ~/colcon_ws && source install/setup.bash
ros2 launch conveyor_description conveyor_model.launch.py
```

### Spawning Objects

**Single object** — choose your cube color:
```bash
# 🔴 Red
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyor_description" --urdf "redcube.urdf" --name "redcube" --x -0.2 --y 0 --z 1

# 🔵 Blue
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyor_description" --urdf "bluecube.urdf" --name "bluecube" --x -0.2 --y 0 --z 1

# 🟢 Green
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyor_description" --urdf "greencube.urdf" --name "greencube" --x -0.2 --y 0 --z 1
```

**Multiple objects** — run the summoner script:
```bash
cd ~/src/Conveyor_simulation/ros2_conveyorbelt/python
python3 summoner.py
```
> ⚠️ Each object type can only be spawned once per session.

### Controlling Belt Speed

**Manual power control** — set speed from 0 to 100:
```bash
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: <VALUE>}"
```

**Automated mode** — IR sensor-driven automation:
```bash
ros2 run conveyor_automation conveyor_automation_node
```
> Two IR sensors at start/end positions detect object presence and trigger belt control automatically.

---

## 🦾 MyCobot Arm Simulation

Launch the arm in Gazebo:
```bash
ros2 launch mycobot_gazebo mycobotjn_control.launch.py
```

Run pick-and-place demo:
```bash
ros2 run mycobot_gazebo pick_and_place
```

---

## 🏗️ Architecture Overview

```
nikiro_simulation/
├── amr_description/          # AMR URDF, launch files, Nav2 config
├── conveyor_description/     # Conveyor belt model & object URDFs
├── ros2_conveyorbelt/        # Belt control service & spawner scripts
├── conveyor_automation/      # IR sensor automation node
└── mycobot_gazebo/           # MyCobot arm model & pick-and-place
```

---

## 📸 Simulation Gallery

<div align="center">

| Mycobot Moveit2 | Conveyor |
|:-:|:-:|
| ![mycobot](https://github.com/user-attachments/assets/aef01611-3f4d-450a-b539-3f6b1ba52c6d) | ![conveyor](https://github.com/user-attachments/assets/dd4af00a-3e8d-4e7c-a134-7c5d580e141b) |

| Full Environment | AMR Navigatio(SLAM)|
|:-:|:-:|
| ![environment](https://github.com/user-attachments/assets/f7070e00-396c-4c26-96d4-45eea8033aa4) | ![Slam](https://github.com/user-attachments/assets/56a8ea1b-e843-4ad6-9a2e-66fd26833575) |

| Object Detection | Pick & Place |
|:-:|:-:|
| ![Obj](https://github.com/user-attachments/assets/253ebebc-489d-4ee7-89aa-68d6d321e1ea) | ![p&p](https://github.com/user-attachments/assets/b163a8eb-8576-47a0-bdef-fbba2a5f7504) |

</div>

---

## 🛠️ Tech Stack

- **ROS 2** (Humble) — Robot middleware & communication
- **Gazebo Classic** — Physics-based 3D simulation
- **Nav2** — Navigation, planning & behavior trees
- **slam_toolbox** — Online/offline SLAM
- **AMCL** — Adaptive Monte Carlo Localization
- **RViz2** — Real-time visualization
- **MyCobot API** — 6-DOF arm control

---

## 🤝 Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss what you'd like to change.

---
