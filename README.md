# Revel x LycheeAI x NVIDIA Isaac Sim Hackathon Project
### Panda Robot Pick-and-Place with MoveIt2 & NVIDIA Isaac Sim 

A complete ROS2 manipulation pipeline demonstrating autonomous pick-and-place of an RTX 3080 graphics card using the Franka Emika Panda robot with collision-aware motion planning in NVIDIA Isaac Sim 4.5.

**Team:** RTXriders  

---

## 🎥 Demo
https://drive.google.com/file/d/1A_-KIXTbWYIuZcsXhohWAbOqj7ww5Lom/view?usp=drive_link
---

## 🎯 Project Overview

This project implements a complete robotic manipulation system that autonomously picks up an RTX 3080 graphics card from one location and places it at another while avoiding obstacles (IKEA tables) in the environment. The system integrates:

- **NVIDIA Isaac Sim 4.5** for high-fidelity physics simulation
- **MoveIt2** for collision-aware motion planning
- **ROS2 Humble** for robot control and communication
- **Custom perception pipeline** for pick-and-place execution

### Key Capabilities

✅ **Collision-Aware Planning** - Dynamically avoids IKEA tables in the environment  
✅ **Precise Manipulation** - Grasps delicate electronics (RTX 3080) safely  
✅ **Multi-Step Execution** - Pre-grasp → Grasp → Lift → Transport → Place sequence  
✅ **Gripper Control** - Adaptive gripper control for object handling  
✅ **Scene Management** - Real-time collision object updates in planning scene  

---

## 🏗️ Technical Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    NVIDIA Isaac Sim 4.5                     │
│         (3080_submission.usd + Physics Engine)              │
└────────────────┬────────────────────────────────────────────┘
                 │ ROS2 Bridge (Topic-based Control)
                 ▼
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 Humble                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │   MoveIt2    │  │ Perception   │  │ ROS2 Control │     │
│  │   Planning   │◄─┤  Pipeline    │◄─┤ + Controllers│     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

### Component Breakdown

**Isaac Sim Integration**
- Custom scene: `3080_submission.usd` with RTX 3080 and IKEA tables
- ROS2 Humble bridge with topic-based control interface
- Physics-accurate simulation of Franka Emika Panda robot

**MoveIt2 Motion Planning**
- OMPL, CHOMP, STOMP, Pilz, and BioIK planning pipelines
- Real-time collision checking with environment obstacles
- Joint-space and Cartesian trajectory planning

**Custom Perception Pipeline**
- Scene setup and collision object management
- Sequential motion execution (10-step pick-and-place workflow)
- Object attachment/detachment for grasp handling

---

## 📁 Workspace Structure

```
revel_lycheeai_nvidia_isaac_sim_pick_and_place_hackathon/
│
├── src/
│   ├── panda_description/           # Robot URDF, meshes, visuals
│   ├── panda_moveit_config/         # MoveIt2 configuration
│   └── perception_pipeline/         # Pick-and-place implementation
│       ├── src/
│       │   └── pick_place_with_grasp.cpp    # Main executable
│       ├── launch/
│       │   ├── pick_and_place.launch.py
│       │   └── perception_pipeline_demo.launch.py
│       └── config/
│           └── sensors_3d.yaml      # Sensor configuration
│
├── 3080_submission.usd              # Isaac Sim scene file
└── README.md
```

---

## 🚀 Getting Started

### Prerequisites

**Required Dependencies:**
- **ROS2 Humble**
- **NVIDIA Isaac Sim 4.5**
- **MoveIt2 Humble**
- **ROS2 Control**
- **ROS2 Controllers**
- **Topic-based ROS2 Control**
- **colcon** build tool

### Installation

**1. Clone the repository:**
```bash
# Create a new workspace
mkdir -p ~/rtx_ws/src
cd ~/rtx_ws/src

# Clone the repo
git clone https://github.com/vaishman/revel_lycheeai_nvidia_isaac_sim_pick_and_place_hackathon.git

# Move src contents to workspace
cd ~/rtx_ws
```

**2. Install dependencies:**
```bash
cd ~/rtx_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the workspace:**
```bash
colcon build
source install/setup.bash
```

---

## 🎮 Running the Demo

### Step-by-Step Execution

**Step 1: Launch Isaac Sim**
```bash
# Open NVIDIA Isaac Sim 4.5
# Load the scene file: 3080_submission.usd
# Enable ROS2 Humble Bridge
# Click Play ▶️
```

**Step 2: Launch Perception Pipeline** (Terminal 1)
```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/rtx_ws/install/setup.bash

# Launch perception pipeline
ros2 launch perception_pipeline perception_pipeline_demo.launch.py
```

**Step 3: Run Pick-and-Place** (Terminal 2)
```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/rtx_ws/install/setup.bash

# Execute pick-and-place
ros2 launch perception_pipeline pick_and_place.launch.py
```

### Quick Start (One Command)
```bash
# After Isaac Sim is running, in separate terminals:

# Terminal 1
ros2 launch perception_pipeline perception_pipeline_demo.launch.py

# Terminal 2
ros2 launch perception_pipeline pick_and_place.launch.py
```

---

## 🔧 Implementation Details

### Pick-and-Place Workflow

The system executes a 10-step manipulation sequence:

1. **Scene Setup** - Add RTX 3080 and IKEA tables to planning scene
2. **Pre-Grasp Approach** - Move arm above target object
3. **Open Gripper** - Prepare for grasping
4. **Lower to Grasp** - Descend to grasp pose
5. **Close Gripper** - Grasp the RTX 3080
6. **Attach Object** - Link object to end-effector in planning scene
7. **Lift** - Raise object clear of surface
8. **Transport** - Navigate to drop location (avoiding IKEA tables)
9. **Release** - Open gripper to place object
10. **Detach & Update** - Remove attachment and update scene

### Collision Avoidance

The MoveIt2 planner actively avoids:
- **IKEA tables** in the workspace
- **Self-collision** of robot links
- **Dynamic obstacles** in the scene

Collision checking uses:
- Primitive shape representations (boxes, cylinders)
- Mesh-based collision models for accurate planning
- Real-time updates from Isaac Sim environment


