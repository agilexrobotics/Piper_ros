# piper_sim

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 gazebo仿真

### 0 环境配置

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```

### 1.1 piper gazebo仿真(有夹爪)

gazebo仿真运行

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

### 1.2 piper gazebo仿真(无夹爪)

```bash
ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py
```

注：**若通过moveit控制时需要先启动gazebo，再启动moveit，并且使用piper_moveit.launch.py而不是demo.launch.py**

## 2 mujoco仿真

### 2.1 环境配置

使用 `uv` 自动配置 MuJoCo Python 环境：

```bash
cd piper_ros
uv sync
source .venv/bin/activate
sudo apt install libosmesa6-dev
sudo apt install patchelf
```

注：**项目已配置最新的 MuJoCo Python 绑定，无需手动安装 mujoco210 或 mujoco-py**

### 2.2 piper mujoco仿真(有夹爪)

mujoco仿真运行

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_mujoco_ctrl.py
```

注：**若出现不能控制mujoco机械臂，请重新启动mujoco**

通过rviz_gui控制有夹爪机械臂(新终端运行)

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_urdf.launch.py
```

#### 2.3 piper mujoco仿真(无夹爪)

mujoco仿真运行

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_no_gripper_mujoco_ctrl.py
```

通过rviz_gui控制无夹爪机械臂(新终端运行)

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_no_gripper_urdf.launch.py
```

注：**如果不能控制，请在运行rviz_gui后运行mujoco**

#### 控制参数介绍

[有夹爪控制参数](../piper_description/mujoco_model/piper_description.xml)

[无夹爪控制参数](../piper_description/mujoco_model/piper_no_gripper_description.xml)

- damping="100 更改关节阻尼

- kp="10000" 更改关节控制增益

- forcerange="-100 100" 更改关节控制力矩
