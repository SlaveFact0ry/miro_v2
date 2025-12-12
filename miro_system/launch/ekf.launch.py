"""
EKF Sensor Fusion Launch Configuration for Autonomous Pool Cleaning Robot

This launch file starts the robot_localization EKF node to fuse sensor data
for accurate odometry estimation on water surfaces.

Sensor Inputs:
- /odom_rf2o: Laser-based odometry from rf2o_laser_odometry (position, velocity)
- /imu/data: IMU orientation and angular velocity from ESP32 micro-ROS

Output:
- /odometry/filtered: Fused odometry estimate
- TF: odom -> base_link transform

Usage:
    # Launch EKF alone
    ros2 launch miro_system ekf.launch.py

    # Launch with custom config
    ros2 launch miro_system ekf.launch.py config_file:=/path/to/ekf.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for EKF sensor fusion."""

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('miro_system'),
            'config',
            'ekf.yaml'
        ]),
        description='Path to EKF configuration file'
    )

    # EKF node from robot_localization package
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
            ('/odometry/filtered', '/odometry/filtered')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        config_file_arg,
        ekf_node
    ])
