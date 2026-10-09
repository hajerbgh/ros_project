# 🤖 TP5 — Autonomous Navigation with Obstacle Avoidance
**ROS2 Humble | TurtleBot3 Waffle | Gazebo Classic**

> Autonomous navigation system for a TurtleBot3 Waffle robot with two implementations:
> a Finite State Machine (FSM) and a Behavior Tree (BT), both featuring intelligent
> obstacle avoidance inspired by Roomba vacuum robots.

---

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Build](#build)
- [Usage](#usage)
- [Architecture](#architecture)
- [Improvements over base assignment](#improvements-over-base-assignment)
- [Project Structure](#project-structure)

---

## Overview

This project implements autonomous navigation with obstacle avoidance for a TurtleBot3
Waffle robot simulated in Gazebo. Two versions are provided:

| Version | File | Description |
|---------|------|-------------|
| FSM | `bump_go_fsm.cpp` | Finite State Machine with 7 states including wall-following and escape |
| BT  | `bump_go_bt.cpp`  | Behavior Tree using BehaviorTree.CPP v3 |

The robot uses its 360° Lidar sensor (`/scan`) to detect obstacles and publishes
velocity commands on `/cmd_vel`.

---

## Features

### FSM Version
- **Multi-zone Lidar analysis** — 3 angular zones instead of a single center point
- **Intelligent rotation direction** — memorizes the clearest side before backing up
- **Wall Following** — proportional controller (P-regulator) to follow walls at constant distance
- **Anti-stuck ESCAPE** — detects repeated collisions and triggers a forced escape sequence
  with alternating directions

### BT Version
- **Modular Behavior Tree** — easy to extend without modifying existing nodes
- **Clean XML definition** — tree structure defined in XML, readable and maintainable
- **Shared ROS2 context** — all BT nodes share the same node and publisher

---

## Requirements

- Ubuntu 22.04 LTS (native or WSL2)
- ROS2 Humble Hawksbill
- Gazebo Classic 11
- TurtleBot3 packages
- BehaviorTree.CPP v3

---

## Installation

### 1. Install ROS2 Humble
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install TurtleBot3 and BehaviorTree
```bash
sudo apt install -y \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-simulations \
  ros-humble-behaviortree-cpp-v3
```

### 3. Clone and setup workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/hajerbgh/bump_go_tp5.git
```

---

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bump_go_tp5
source install/setup.bash
```

---

## Usage

### Terminal 1 — Launch Gazebo simulation
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Wait for: `Spawn status: Successfully spawned entity [waffle]`

### Terminal 2 — Run FSM version
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run bump_go_tp5 bump_go_fsm --ros-args -p use_sim_time:=true
```

### Terminal 2 — Run BT version (alternative)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run bump_go_tp5 bump_go_bt --ros-args -p use_sim_time:=true
```

> ⚠️ Run only ONE version at a time — both publish on `/cmd_vel`.

### Terminal 3 — Monitor (optional)
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /cmd_vel      # watch velocity commands
ros2 topic hz /scan           # check Lidar frequency (~4 Hz)
ros2 node list                # list active nodes
```

---

## Architecture

### ROS2 Computation Graph
```
[Gazebo]
    │
    ├──▶ /scan  (sensor_msgs/LaserScan, ~4Hz, BestEffort QoS)
    │         │
    │         ▼
    │   [bump_go_fsm] OR [bump_go_bt]
    │         │
    │         ▼
    └──▶ /cmd_vel  (geometry_msgs/Twist, 20Hz)
              │
              ▼
         [TurtleBot3 in Gazebo]
```

### FSM State Diagram
```
                    ┌─────────────────────────────────────────┐
                    │                                         │
              scan ok                                   scan ok
                    │                                         │
         ┌──────────▼──────────┐              ┌──────────────▼──────┐
         │       FORWARD        │──obstacle──▶ │        BACK         │
         │    v = 0.2 m/s       │              │    v = -0.2 m/s     │
         └──────────────────────┘              └──────────────┬──────┘
                    │                                         │
              timeout scan                               2s elapsed
                    │                                         │
         ┌──────────▼──────────┐              ┌──────────────▼──────┐
         │        STOP          │              │        TURN         │
         │      v = w = 0       │              │   w = ±0.4 rad/s   │
         └──────────────────────┘              └──────────────┬──────┘
                                                              │
                                                         3s elapsed
                                                              │
         ┌──────────────────────┐                            │
         │     WALL_FOLLOW *    │◀── wall < 0.875m ──────────┘
         │  P-controller        │
         └──────────────────────┘
                    │
             5 consecutive obstacles
                    │
         ┌──────────▼──────────┐    3s    ┌──────────────────────┐
         │    ESCAPE_BACK *     │─────────▶│    ESCAPE_TURN *     │
         │   v = -0.24 m/s      │          │  w = ±0.6 rad/s 5s  │
         └──────────────────────┘          └──────────────────────┘

* = added states (not in base assignment)
```

### BT Structure
```
Selector (root)
├── Sequence [obstacle_avoidance]   ← high priority
│     ├── IsObstacleAhead           (Condition: obstacle < 0.5m?)
│     ├── BackAction                (Action: back up 2s)
│     └── TurnAction                (Action: turn 2s)
├── Sequence [normal_nav]
│     ├── HasScan                   (Condition: recent scan < 1s?)
│     └── MoveForward               (Action: v=0.3 m/s)
└── StopAction                      (Action: fallback stop)
```

---

## Improvements over base assignment

| Feature | Base assignment | Our version |
|---------|----------------|-------------|
| Obstacle detection | Single center point `ranges[N/2]` | 3 angular zones ±30° front + lateral |
| Rotation direction | Always left | Memorized clearest side before backing |
| Wall behavior | Brutal back-up | P-controller wall-following at 0.35m |
| Anti-stuck | None | ESCAPE counter + alternating direction |
| Obstacle threshold | 1.0m | 0.5m (calibrated for 28cm robot width) |
| Rear collision | Not checked | Monitored during ESCAPE_BACK |

### Key parameters

```cpp
OBSTACLE_DISTANCE = 0.5f   // obstacle detection threshold (m)
WALL_DISTANCE     = 0.35f  // wall-following target distance (m)
SPEED_LINEAR      = 0.2f   // forward speed (m/s)
SPEED_ANGULAR     = 0.4f   // rotation speed (rad/s)
BACKING_TIME      = 2s     // back-up duration
TURNING_TIME      = 3s     // turn duration
ESCAPE_BACK_TIME  = 3s     // escape back-up duration
ESCAPE_TURN_TIME  = 5s     // escape turn duration
obstacle_count    = 5      // collisions before ESCAPE triggers
```

---

## Project Structure

```
bump_go_tp5/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS2 package metadata
├── README.md               # This file
└── src/
    ├── bump_go_fsm.cpp     # Version 1: FSM implementation
    └── bump_go_bt.cpp      # Version 2: Behavior Tree implementation
```

---

## Known Limitations

- The Lidar cannot distinguish between a wall and a pole — it only measures distances.
  A future improvement would analyze scan continuity to differentiate extended vs. punctual obstacles.
- During ESCAPE_BACK, the robot may hit a rear obstacle despite the rear-zone check,
  as the Lidar frequency (~4 Hz) may miss fast approach.

---

## Author

**Hajer Benghazi** — ICE4, Faculté des Sciences de Tunis  
Supervisor: Chiheb Ameur ABID  
Academic year: 2025–2026
