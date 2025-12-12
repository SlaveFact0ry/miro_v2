"""
Calibration System Launch File for MIRO v2

Launches the calibration system components including:
- Speed Preset Manager: Fixed speed control via joystick buttons
- Joy Node: Joystick hardware interface (optional, may already be running)

Usage:
    ros2 launch miro_calibration calibration.launch.py
    ros2 launch miro_calibration calibration.launch.py launch_joy:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for calibration system."""

    # Package directories
    calibration_pkg = get_package_share_directory('miro_calibration')

    # Configuration file paths
    speed_presets_config = PathJoinSubstitution([
        FindPackageShare('miro_calibration'),
        'config',
        'speed_presets.yaml'
    ])

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

    # Joy node (joystick hardware interface)
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

    # Speed Preset Manager node
    speed_preset_manager_node = Node(
        package='miro_calibration',
        executable='speed_preset_manager',
        name='speed_preset_manager',
        output='screen',
        parameters=[speed_presets_config]
    )

    return LaunchDescription([
        # Launch arguments
        launch_joy_arg,
        joy_device_arg,

        # Nodes
        joy_node,
        speed_preset_manager_node,
    ])
