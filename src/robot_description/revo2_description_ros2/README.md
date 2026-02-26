<div align="right">

[English](README.md)|[简体中文](README_CN.md)

</div>

# Revo2 Description

> 👉 Want the ROS 2 version? Visit [revo2_description (ROS2)](https://github.com/BrainCoTech/revo2_description)

## Overview

The Revo2 Description repository offers detailed URDF models for BrainCo Revo2 dexterous hands. Additionally, the repository provides Docker support for easy visualization and simulation setup.

## Features

- **Detailed 3D Models**: High-fidelity 3D models of Revo2 left and right hands for accurate simulation and visualization.
- **RViz Visualization**: Built-in launch files for visualizing robot models in RViz.
- **Gazebo Simulation**: Launch files for simulating robot models in Gazebo.
- **Docker Support**: Pre-configured Docker environment for easy setup and deployment.

## Prerequisites

### For Docker Usage
- Docker

### For Native ROS1 Usage
- Ubuntu 20.04
- ROS Noetic
- Additional packages: `ros-noetic-xacro`, `ros-noetic-joint-state-publisher-gui`, `ros-noetic-gazebo-ros-pkgs`

## Installation and Setup

### Option 1: Docker Setup (Recommended)

To use the Docker-based visualization and simulation:

#### Automatic Setup (Recommended)
The scripts will automatically build the Docker image on first use:

```bash
# Visualize left hand (default) - image will be built automatically on first run
./scripts/visualize_revo2.sh left

# Visualize right hand
./scripts/visualize_revo2.sh right

# Simulate left hand (default)
./scripts/simulate_revo2.sh left

# Simulate right hand
./scripts/simulate_revo2.sh right
```

#### Manual Setup (Optional)
You can also build the Docker image manually beforehand:

```bash
# Build the Docker image once
docker build -t revo2_description_ros2 .docker

# Then use the scripts (they will skip building and use the existing image)
./scripts/visualize_revo2.sh left
```

### Option 2: Native ROS1 Setup

For native ROS1 installation:

```bash
# Install ROS Noetic (if not already installed)
# Follow instructions at: http://wiki.ros.org/noetic/Installation/Ubuntu

# Install additional dependencies
sudo apt-get update
sudo apt-get install ros-noetic-xacro ros-noetic-joint-state-publisher-gui ros-noetic-gazebo-ros-pkgs

# Create a catkin workspace
mkdir -p ~/revo2_ws/src
cd ~/revo2_ws/src

# Copy or clone the revo2_description package
cp -r /path/to/revo2_description .

# Build the workspace
cd ~/revo2_ws
catkin_make
source devel/setup.bash
```

## Visualization

### RViz Visualization

#### Using Docker (Recommended)

```bash
# Visualize left hand (default)
./scripts/visualize_revo2.sh left

# Visualize right hand
./scripts/visualize_revo2.sh right
```

#### Using Native ROS1

After setting up the workspace:

```bash
# Source the workspace
source ~/revo2_ws/devel/setup.bash

# Visualize left hand
roslaunch revo2_description display_revo2_left_hand.launch

# Visualize right hand
roslaunch revo2_description display_revo2_right_hand.launch
```

### Gazebo Simulation

#### Using Docker (Recommended)

```bash
# Simulate left hand (default)
./scripts/simulate_revo2.sh left

# Simulate right hand
./scripts/simulate_revo2.sh right
```

#### Using Native ROS1

After setting up the workspace:

```bash
# Source the workspace
source ~/revo2_ws/devel/setup.bash

# Simulate left hand
roslaunch revo2_description gazebo_revo2_left_hand.launch

# Simulate right hand
roslaunch revo2_description gazebo_revo2_right_hand.launch
```

## Joint Information

### Left Hand Joints

| Joint Name | Description | Range (degrees) | Range (radians) |
|------------|-------------|-----------------|-----------------|
| left_thumb_flex_joint | Thumb flexion | 0 ~ 59 | 0 ~ 1.03 |
| left_thumb_abduct_joint | Thumb abduction | 0 ~ 90 | 0 ~ 1.57 |
| left_index_joint | Index finger | 0 ~ 81 | 0 ~ 1.41 |
| left_middle_joint | Middle finger | 0 ~ 81 | 0 ~ 1.41 |
| left_ring_joint | Ring finger | 0 ~ 81 | 0 ~ 1.41 |
| left_pinky_joint | Pinky finger | 0 ~ 81 | 0 ~ 1.41 |

### Right Hand Joints

| Joint Name | Description | Range (degrees) | Range (radians) |
|------------|-------------|-----------------|-----------------|
| right_thumb_flex_joint | Thumb flexion | 0 ~ 59 | 0 ~ 1.03 |
| right_thumb_abduct_joint | Thumb abduction | 0 ~ 90 | 0 ~ 1.57 |
| right_index_joint | Index finger | 0 ~ 81 | 0 ~ 1.41 |
| right_middle_joint | Middle finger | 0 ~ 81 | 0 ~ 1.41 |
| right_ring_joint | Ring finger | 0 ~ 81 | 0 ~ 1.41 |
| right_pinky_joint | Pinky finger | 0 ~ 81 | 0 ~ 1.41 |

## Package Structure

```
revo2_description/
├── launch/                 # ROS launch files
├── meshes/                 # 3D mesh files (.STL)
│   ├── revo2_left_hand/    # Left hand meshes
│   └── revo2_right_hand/   # Right hand meshes
├── scripts/                # Docker utility scripts
├── urdf/                   # URDF model files
├── rviz/                   # RViz configuration files
└── .docker/               # Docker support files
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
