"""
Teleoperation Launch File for MIRO v2

Launches joystick-based teleoperation for manual robot control.

Components:
- Joy Node: Joystick hardware interface (optional, may already be running)
- Teleop Twist Joy: Converts joystick axes to cmd_vel messages

Usage:
    ros2 launch miro_control teleop.launch.py
    ros2 launch miro_control teleop.launch.py launch_joy:=false
    ros2 launch miro_control teleop.launch.py joy_device:=/dev/input/js1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for teleoperation."""

    pkg_name = 'miro_control'
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'joystick.yaml'
    )

    # Launch arguments
    launch_joy_arg = DeclareLaunchArgument(
        'launch_joy',
        default_value='true',
        description='Launch joy_node for joystick input (set to false if already running)'
    )

    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # 1. Joy Node (조이스틱 하드웨어 읽기)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_device'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('launch_joy'))
    )

    # 2. Teleop Twist Joy (Joy -> cmd_vel 변환)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        # Launch arguments
        launch_joy_arg,
        joy_device_arg,

        # Nodes
        joy_node,
        teleop_node,
    ])
