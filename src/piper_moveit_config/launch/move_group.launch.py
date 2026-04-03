import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    
    # Planning configuration
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    ompl_planning_yaml = load_yaml('piper_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    
    # Trajectory execution configuration
    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml('piper_moveit_config', 'config/moveit_controllers.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Start the move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': '/joint_states',
                }
            },
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
    
    return LaunchDescription([
        move_group_node
    ])
