import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
from launch.actions import TimerAction, OpaqueFunction


def load_yaml(package_name, file_path):
    """Load a YAML file from a ROS package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

# ==========================================================================
# Launch Arguments
# ==========================================================================
debug_arg = DeclareLaunchArgument(
    'debug',
    default_value='false',
    description='Enable debug output'
)

gripper_arg = DeclareLaunchArgument(
    'use_gripper',
    default_value='true',
    description='Whether to include gripper in the MoveIt configuration'
)

launch_args = [debug_arg, gripper_arg]

def launch_setup(context):
    """
    Minimal launch file for Piper robot with MoveIt in simulation.
    
    Equivalent structure to HDT robot's launch file:
    1. Load robot description (XACRO)
    2. Load configurations (controllers, kinematics, etc.)
    3. Start hardware interface (fake)
    4. Start robot state publisher
    5. Start MoveIt move_group
    6. Start RViz
    """
    
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    piper_moveit_config_path = get_package_share_directory('piper_moveit_config')

    # ==========================================================================
    # Load Robot Description (from XACRO)
    # ==========================================================================
    piper_description_path = get_package_share_directory('piper_description')
    # if use_gripper is false, load the no_gripper version of the xacro and configs
    use_gripper = LaunchConfiguration('use_gripper').perform(context)
    if use_gripper.lower() == 'false':
        urdf_file = os.path.join(piper_moveit_config_path, 'config/no_gripper', 'piper.urdf.xacro')
        srdf_file = os.path.join(piper_moveit_config_path, 'config/no_gripper', 'piper.srdf')
        # Controller configuration
        moveit_controllers_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/moveit_controllers.yaml') # REAL controllers
        controllers_yaml_path = os.path.join(piper_moveit_config_path, "config/no_gripper", "ros2_controllers.yaml")
        # Kinematics configuration
        kinematics_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/kinematics.yaml')
        joint_limits_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/joint_limits.yaml')
        # Planning configuration
        ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/ompl_planning.yaml')
    else:
        urdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro')
        srdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.srdf')
        # Controller configuration
        moveit_controllers_yaml = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml') # REAL controllers
        controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")
        # Kinematics configuration
        kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
        joint_limits_yaml = load_yaml('piper_moveit_config', 'config/joint_limits.yaml')
        # Planning configuration
        ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/ompl_planning.yaml')
    
    # urdf_file = os.path.join(piper_description_path, 'urdf', 'piper_description.xacro')
    robot_description_content = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    robot_description = {'robot_description': robot_description_content}
    robot_description = {"robot_description": Command(["xacro ", urdf_file])}
    
    
    # Semantic description (SRDF)
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    
    # ==========================================================================
    # Hardware Interface Node (Fake execution for simulation)
    # ==========================================================================
    # fake_hardware_node = Node(
    #     package='piper_moveit_config',
    #     executable='fake_hardware_interface.py',
    #     name='fake_hardware_interface',
    #     output='screen'
    # )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml_path
        ],
        output="screen",
    )
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
                ],
            )
        ]
    )

    # velocity_controller_spawner = TimerAction(
    #     period=4.0,
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner.py",
    #             arguments=["joint_group_velocity_controller", "--controller-manager", "/controller_manager"],
    #         )
    #     ]
    # )
    arm_controller_spawner = TimerAction(
        period=5.0,
        actions=[Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"], # for position control
    )])

    arm_controller_velocity_spawner = TimerAction(
        period=5.0,
        actions=[Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_velocity_controller", "--controller-manager", "/controller_manager"], # for velocity control
        condition=LaunchConfigurationEquals('use_gripper', 'true')
    )])

    gripper_controller_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
            condition=LaunchConfigurationEquals('use_gripper', 'true')
        )]
    )
    
    # ==========================================================================
    # Robot State Publisher (Broadcasts TF from /joint_states)
    # ==========================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # ==========================================================================
    # MoveIt Move Group Node
    # ==========================================================================
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            ompl_planning_pipeline_config,
            trajectory_execution,
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_simple_controller_manager': moveit_controllers_yaml.get('moveit_simple_controller_manager', {})},
            planning_scene_monitor,
        ]
    )
    
    # ==========================================================================
    # RViz Visualization
    # ==========================================================================
    rviz_config_file = os.path.join(piper_moveit_config_path, 'rviz', 'moveit.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
        ]
    )
    
    # ==========================================================================
    # Launch Description
    # ==========================================================================
    return [
        debug_arg,
        # fake_hardware_node,
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        arm_controller_velocity_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld