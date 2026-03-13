# Piper Pinocchio Inverse Kinematics

## Conda Environment Setup

The code has only been tested with **pinocchio=3.6.0=py39hfd66edf_0**.

```shell
conda create -n tv python=3.10
conda activate tv
conda install pinocchio=3.6.0 -c conda-forge
pip install meshcat
pip install casadi
```

## Run

Run the following in a separate terminal.

This launches the **Piper single-arm control node**.  
(This step can also be skipped if you only want to run the inverse kinematics. This step is used to control the **real robotic arm**.)

```bash
source /{your_path}/piper_ros/install/setup.sh
ros2 launch piper start_single_piper.launch.py
```

Run the script directly using the Python from the conda environment:

```bash
source /{your_path}/piper_ros/install/setup.sh
conda activate tv
python /{your_path}/piper_ros/src/piper/scripts/piper_pinocchio/piper_pinocchio.py
```

After running successfully, a **browser visualization page will automatically open**, and the node will create topics.  
Here we only display the topics created by the `piper_pinocchio` node.

```bash
ros2 topic list
```

```bash
/joint_states #pub
/pin_pos_cmd #sub
```

View details of `/pin_pos_cmd`:

```bash
ros2 topic info /pin_pos_cmd
```

You can see that this message is a **custom message type** subscribed to by this node.  
To control the robotic arm, you must **publish messages to this topic**.

```bash
Type: piper_msgs/msg/PosCmd

Publisher count: 0

Subscription count: 1
```

Publish end-effector commands:

```bash
source /{your_path}/piper_ros/install/setup.sh
# Send zero position
ros2 topic pub /pin_pos_cmd piper_msgs/PosCmd "{x: 0.056218, y: 0.0, z: 0.213266, roll: 0.0, pitch: 1.48352986, yaw: 0.0, gripper: 0.0, mode1: 0, mode2: 0}"

# Raise the z-axis by 0.1 m
ros2 topic pub /pin_pos_cmd piper_msgs/PosCmd "{x: 0.056218, y: 0.0, z: 0.313266, roll: 0.0, pitch: 1.48352986, yaw: 0.0, gripper: 0.0, mode1: 0, mode2: 0}"
```

## Q&A

### 1

If you installed `pinocchio` but encounter the error:

```bash
ImportError: cannot import name 'casadi' from 'pinocchio'
```

First check your Pinocchio version:

```bash
# Activate environment
conda activate tv

# Check package
conda list | grep pinocchio

# Or
conda env export | grep pinocchio
```

This issue occurs because **casadi is not included inside pinocchio** in some versions.

According to the project's changelog, versions **after 3.0.0 include casadi**, but **version 3.7.0 dropped support for Ubuntu 20.04**.

https://github.com/stack-of-tasks/pinocchio/blob/devel/CHANGELOG.md#300---2024-05-27

Use the following command to check which Pinocchio versions are available:

```bash
conda search pinocchio -c conda-forge
```

Then check whether the version recommended in this document is available.
