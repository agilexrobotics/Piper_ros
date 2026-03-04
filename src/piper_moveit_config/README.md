# Piper MoveIt Configuration
## WARNING: THE VAST MAJORITY OF THIS CODE WAS DEVELOPED WITH THE HELP OF GENERATIVE AI.
This package was meant to only create a simulation environment in ROS2 Foxy MoveIt.
It has NOT been debugged to use with a real robot.

## TL;DR
`ros2 launch piper_moveit_config piper_bringup.launch.py`


## Overview
This package provides MoveIt2 configuration for the Piper robot arm in ROS2 Foxy.

The package includes:
- **SRDF**: Semantic robot description with planning groups, end effectors, and predefined poses
- **Planning Groups**:
  - `arm`: 6-DOF manipulator (joint1-joint6) with end effector at gripper_base
  - `gripper`: 2-finger parallel gripper (joint7-joint8)
- **Kinematics**: KDL solver configuration
- **Motion Planning**: OMPL with multiple planning algorithms (RRTConnect default)
- **Controllers**: Fake controllers for simulation

## Installation

### Prerequisites

Make sure you have the following ROS2 packages installed:

```bash
sudo apt update
sudo apt install ros-foxy-moveit ros-foxy-moveit-ros-visualization \
  ros-foxy-moveit-planners-ompl ros-foxy-moveit-simple-controller-manager
```

### Build

```bash
cd ~/catkin_ws/src
colcon build --packages-select piper_moveit_config
source install/setup.bash
```

Note: You mgiht run into errors when building. I manually copied a header file into the include folder.

## Usage

### Launch MoveIt with RViz

To start the complete MoveIt interface with RViz for motion planning:

```bash
source ~/catkin_ws/src/install/setup.bash
ros2 launch piper_moveit_config piper_bringup.launch.py
```

This will:
1. Load robot description (XACRO)     
2. Load configurations (controllers, kinematics, planning)  
3. [Skip serial/comms - simulation]
4. Start hardware interface (fake_hardware_interface)  
5. Start robot state publisher  
6. Start MoveIt move_group  
7. Start RViz  

### Using the Interface

Once RViz opens with the MoveIt plugin:

1. **Select Planning Group**: In the "MotionPlanning" panel, select "arm" or "gripper" from the dropdown

2. **Set Goal State**:
   - **Interactive Markers**: Drag the interactive marker (orange ball) to set a goal pose
   - **Named States**: Select predefined poses like "home" or "ready" from the dropdown
   - **Random Valid**: Click to generate a random valid goal configuration

3. **Plan Motion**:
   - Click "Plan" button to compute a trajectory
   - The planned path will be visualized in RViz

4. **Execute Motion**:
   - After planning, click "Execute" to run the trajectory
   - In simulation mode, the robot will move smoothly following the planned path

### Named States

The following predefined joint configurations are available:

**Arm Group:**
- `home`: All joints at 0° (straight up configuration)
- `ready`: Standard ready position

**Gripper Group:**
- `gripper_open`: Fully open (0.035m)
- `gripper_closed`: Fully closed (0m)

### Configuration Files

All configuration files are in the `config/` directory:

- `piper.srdf`: Semantic robot description (planning groups, collision matrix)
- `kinematics.yaml`: Inverse kinematics solver settings
- `joint_limits.yaml`: Velocity and acceleration limits
- `ompl_planning.yaml`: Motion planning algorithm configuration
- `moveit_controllers.yaml`: Controller definitions for MoveIt
- `ros_controllers.yaml`: ROS2 controller configuration

## Customization

### Modify Joint Limits

Edit `config/joint_limits.yaml` to adjust:
- Velocity scaling: `default_velocity_scaling_factor`
- Acceleration scaling: `default_acceleration_scaling_factor`
- Per-joint limits

### Change Planning Algorithm

Edit `config/ompl_planning.yaml` to set different default planners:
- RRTConnect (fast, default)
- RRTstar (optimal)
- PRM (probabilistic roadmap)
- Others: KPIECE, EST, SBL, TRRT

### Add Custom Poses

Edit `config/piper.srdf` to add new `<group_state>` entries with desired joint values.

## Integration with Real Robot

To use MoveIt with the real robot instead of fake controllers:

1. Modify `launch/demo.launch.py`:
   - Set `moveit_manage_controllers: True` in trajectory_execution
   - Remove or modify the `joint_state_publisher` node

2. Ensure your robot hardware interface publishes:
   - `/joint_states` topic
   - Action server for `/follow_joint_trajectory`

3. Update `config/moveit_controllers.yaml` with your actual controller names

## Troubleshooting

**Issue**: RViz doesn't show the robot
- Check that `robot_description` topic is being published
- Verify URDF/XACRO files are correctly loaded

**Issue**: Planning fails
- Check joint limits in `config/joint_limits.yaml`
- Verify goal pose is reachable and collision-free
- Try different planning algorithms

**Issue**: Move Group node crashes
- Check MoveIt dependencies are installed
- Verify SRDF file is correctly formatted
- Check ROS2 logs: `ros2 launch piper_moveit_config demo.launch.py --screen`

## Files Structure

```
piper_moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── piper.srdf                    # Semantic description
│   ├── kinematics.yaml               # IK solver config
│   ├── joint_limits.yaml             # Joint constraints
│   ├── ompl_planning.yaml            # Planning algorithms
│   ├── moveit_controllers.yaml       # MoveIt controllers
│   └── ros_controllers.yaml          # ROS2 controllers
├── launch/
│   ├── piper_bringup.launch.py       # MAIN launch file
│   ├── demo.launch.py                # Previous launch file
│   └── move_group.launch.py          # Move group only
├── scripts/
│   ├── fake_hardware_interface.py    # Set up action server to execute fake trajectories
│   └── move_group.launch.py          # Move group only
└── rviz/
    └── moveit.rviz                   # RViz configuration
```

## Next Steps

1. **Test Planning**: Try planning to different poses and verify smooth motion
2. **Tune Parameters**: Adjust velocity/acceleration limits for your needs
3. **Add Obstacles**: Use the "Scene Objects" panel in RViz to add collision objects
4. **Python/C++ Interface**: Write code to send motion planning requests programmatically

## Resources

- [MoveIt2 Documentation](https://moveit.picknik.ai/foxy/index.html)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/foxy/doc/tutorials/tutorials.html)
- [OMPL Planners](https://ompl.kavrakilab.org/planners.html)
