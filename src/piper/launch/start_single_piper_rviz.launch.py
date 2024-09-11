# piper_launch.py

from launch import LaunchDescription
# from launch_ros.actions import Node, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription  # 正确的导入方式
from launch_ros.actions import Node  # 保持不变
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # 获取piper_description包的路径
    piper_description_path = os.path.join(
        get_package_share_directory('piper_description'),
        'launch',
        'display_xacro.launch.py'
    )

    # 定义launch参数
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port for the robot arm'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Enable robot arm automatically'
    )

    # 包含 display_xacro.launch.py
    display_xacro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_description_path)
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='true',
        description='Start rviz flag.'
    )
    
    girpper_exist_arg = DeclareLaunchArgument(
        'girpper_exist',
        default_value='true',
        description='gripper'
    )
    
    # 定义机械臂节点
    piper_ctrl_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        parameters=[
            {'can_port': LaunchConfiguration('can_port')},
            {'auto_enable': LaunchConfiguration('auto_enable')},
            {'rviz_ctrl_flag':  LaunchConfiguration('rviz_ctrl_flag')},
            {'girpper_exist':  LaunchConfiguration('girpper_exist')}
        ],
        remappings=[
            ('joint_ctrl_single', '/joint_states')
        ]
    )

    # 返回LaunchDescription对象，包含以上所有元素
    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        display_xacro_launch,
        rviz_ctrl_flag_arg,
        girpper_exist_arg,
        piper_ctrl_node
    ])
