import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    """Load a YAML file from a ROS package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
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
    # Launch Arguments
    # ==========================================================================
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )
    
    # ==========================================================================
    # Load Robot Description (from XACRO)
    # ==========================================================================
    piper_description_path = get_package_share_directory('piper_description')
    urdf_file = os.path.join(piper_description_path, 'urdf', 'piper_description.xacro')
    robot_description_content = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    robot_description = {'robot_description': robot_description_content}
    
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    piper_moveit_config_path = get_package_share_directory('piper_moveit_config')
    
    # Semantic description (SRDF)
    srdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Controller configuration
    # moveit_controllers_yaml = load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml') # REAL controllers
    moveit_controllers_yaml = load_yaml('piper_moveit_config', 'config/fake_moveit_controllers.yaml')
    
    # Kinematics configuration
    kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
    
    # Planning configuration
    joint_limits_yaml = load_yaml('piper_moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/ompl_planning.yaml')
    
    # ==========================================================================
    # Hardware Interface Node (Fake execution for simulation)
    # ==========================================================================
    fake_hardware_node = Node(
        package='piper_moveit_config',
        executable='fake_hardware_interface.py',
        name='fake_hardware_interface',
        output='screen'
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
    return LaunchDescription([
        debug_arg,
        fake_hardware_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
