![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/python-3.10-blue?logo=python&logoColor=white)
![Gazebo](https://img.shields.io/badge/simulation-ignition-orange?logo=gazebo&logoColor=white)
![MoveIt](https://img.shields.io/badge/planning-moveit2-green)
![License](https://img.shields.io/badge/license-MIT-yellow)

# Autonomous Manipulation Suite with Panda (ROS 2 & MoveIt 2)

**A robotic manipulation framework for Table Cleaning using the Franka Emika Panda in Ignition Gazebo.**

## Table of Contents
1. [Project Overview](#project-overview)
2. [Tech Stack](#tech-stack)
3. [System Architecture](#system-architecture)
4. [Directory Structure](#directory-structure)
5. [Installation](#installation)
6. [How to Run](#how-to-run)

## Project Overview
This repository implements a complete autonomous manipulation framework developed on **ROS 2 Humble**. It integrates computer vision (OpenCV) and motion planning (MoveIt 2) to control a 7-DOF **Franka Emika Panda** robot in a physics-based simulation (Ignition Gazebo).

* **Objective:** Maintain a clean workspace by autonomously removing randomly spawned cubes.
* **Behavior:** A spawner continuously drops cubes at random position every 30 seconds. The robot detects cubes anywhere on the table, plans a path, and clears it into a dustbin.

## Tech Stack

| Component | Technology |
| :--- | :--- |
| **Operating System** | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| **Middleware** | ROS 2 Humble Hawksbill |
| **Simulation** | Ignition Gazebo Fortress |
| **Motion Planning** | MoveIt 2 |
| **Computer Vision** | OpenCV |
| **Programming** | Python 3.10 |
| **Hardware** | Franka Emika Panda (Simulated) |

## System Architecture

The system follows a modular pipeline structure, separating Perception, Logic, and Actuation.


<img width="1024" height="572" alt="image-2" src="https://github.com/user-attachments/assets/4b1d516f-e5fa-472c-9423-ca458d4a4b9b" />

The image above illustrates the overall architecture and data flow of our ROS 2 robotic system. It is composed of three main subsystems:

**1. Ignition Gazebo Physics (Simulation)**
* **Simulated World (Camera & Motors):** This component represents the simulated environment, including the robot, its sensors (like the camera), and the physics engine that governs their interactions. It generates raw image data from the camera and accepts motor commands to actuate the robot.
* **ros_gz_bridge:** This is a crucial bridge that facilitates communication between the Ignition Gazebo simulation and the ROS 2 ecosystem. It translates data formats, allowing ROS 2 nodes to receive sensor data (like `Raw Image`) and send commands to the simulation.

**2. ROS 2 Software Stack (Perception & Logic)**
* **tf_detector (Perception):** This node subscribes to the `/camera/image_raw` topic, receiving image data from the simulation. It processes these images to detect and locate blocks, publishing their coordinates to the `/block_coords` topic.
* **automatic_pnp:** This node acts as the high-level decision-maker. It receives block coordinates from `tf_detector` and determines the sequence of actions required for a pick-and-place task. It then sends the desired `Target Pose` to the motion planning subsystem.

**3. Motion Planning (Path Planning & Control)**
* **MoveIt 2 (Path Planning):** MoveIt 2 receives the `Target Pose` from the `automatic_pnp` node. It calculates a collision-free path for the robot to reach the target pose and generates a trajectory, which is a sequence of `Trajectory Point`s.
* **JointTrajectoryController (Driver):** This controller receives the trajectory from MoveIt 2 and converts it into low-level `Motor Command`s. These commands are then sent back to the `Simulated World` in Ignition Gazebo to physically move the robot's joints.



## Directory Structure

```text
src
├── automatic_pnp
│   └── automatic_pnp
│       ├── clear_table.py          # Table cleaning logic node
│       └── __init__.py
├── cube_spawner
│   ├── cube_spawner
│   │   ├── __init__.py
│   │   └── spawner.py              # Base spawning utility
│   └── launch
│       └── bringup.launch.py
├── franka_moveit_config
│   └── (World, Arm Config, Controllers, Launch Files)
├── panda_description
│   └── (Visual meshes, Collision models, URDF)
└── tf_detector
    ├── launch
    │   └── vision_system.launch.py # Starts Camera & Detector
    └── tf_detector
        ├── detector.py             # Generic Object Detection Node
        └── __init__.py
```
## Installation

### 1. Prerequisites

Ensure you have the following core components installed on your system. Click the links for official setup guides:

* **[ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)** (Desktop Install recommended)
* **[Ignition Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu)** (Simulated physics engine)
* **[MoveIt 2](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)** (Motion planning framework)

**to install opencv using pip:**
```bash
pip install opencv
```
### 2. Setup Workspace

Once the prerequisites are ready, set up your ROS 2 workspace:

```bash
# 1. Create a workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone this repository
git clone https://github.com/roboholic-harsh/table_cleaning
```

### 3. Install Dependencies

Use `rosdep` to automatically install all missing system dependencies (controllers, bridges, etc.):

```bash
cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Project
Compile the packages using `colcon`:

```bash
colcon build --symlink-install
```
## How to Run

Open **2 separate terminal tabs**. In *every* tab, make sure to source your workspace first:


```bash
source install/setup.bash
```

*by running this the robot continuously clears objects that spawn every 30 seconds.*

| Terminal | Component | Command |
| --- | --- | --- |
| **1** | **Vision** | `ros2 launch tf_detector vision_system.launch.py` |
| **2** | **Brain** | `ros2 run automatic_pnp clear_table` |

---

The framework can be extended to different objects, tasks, and robotic platforms with minimal modifications.


