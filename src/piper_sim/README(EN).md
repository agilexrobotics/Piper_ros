# piper_sim

[中文](README.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 Gazebo Simulation

### 0 Environment Setup

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```

### 1.1 Piper Gazebo Simulation (With Gripper)

Run the Gazebo simulation:

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

### 1.2 Piper Gazebo Simulation (Without Gripper)

```bash
ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py
```

**Note:** If controlling via MoveIt, you must start Gazebo first, then MoveIt. Also, use `piper_moveit.launch.py` instead of `demo.launch.py`.

## 2 Mujoco Simulation

### 2.1 Environment Setup

Use `uv` to automatically configure the MuJoCo Python environment:

```bash
cd piper_ros
uv sync
source .venv/bin/activate
sudo apt install libosmesa6-dev
sudo apt install patchelf
```

**Note:** The project is configured with the latest MuJoCo Python bindings. No need to manually install mujoco210 or mujoco-py.

### 2.2 Piper Mujoco Simulation (With Gripper)

Run the Mujoco simulation:

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_mujoco_ctrl.py
```

**Note:** If you are unable to control the Mujoco robot arm, restart Mujoco.

To control the robot arm with the `rviz_gui` (run in a new terminal):

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_urdf.launch.py
```

### 2.3 Piper Mujoco Simulation (Without Gripper)

Run the Mujoco simulation:

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_no_gripper_mujoco_ctrl.py
```

To control the robot arm with `rviz_gui` (run in a new terminal):

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_no_gripper_urdf.launch.py
```

**Note:** If you cannot control the robot arm, run Mujoco after launching `rviz_gui`.

### Control Parameter Introduction

[Control parameters for the gripper version](../piper_description/mujoco_model/piper_description.xml)

[Control parameters for the non-gripper version](../piper_description/mujoco_model/piper_no_gripper_description.xml)

- `damping="100"` → Adjust joint damping
- `kp="10000"` → Adjust joint control gain
- `forcerange="-100 100"` → Adjust joint control torque range
