"""
Static TF Transforms Launch for Autonomous Pool Cleaning Robot

This launch file publishes all static transforms in the robot's TF tree.
These transforms define the geometric relationships between coordinate frames
that don't change during operation.

Transform tree structure:
    base_link (robot center)
    ├── imu_link (IMU sensor on ESP32)
    ├── laser (RPLIDAR A1)
    ├── sonar_left_link (Left ultrasonic sensor)
    ├── sonar_center_link (Center ultrasonic sensor)
    └── sonar_right_link (Right ultrasonic sensor)

Note: The odom -> base_link transform is published by robot_localization EKF node

Usage:
    ros2 launch miro_system static_transforms.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate static transform publishers for robot frames."""

    # Static transform: base_link -> imu_link
    # IMU (MPU9250) is mounted on ESP32 at robot center, slightly elevated
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '--x', '0.0',      # 0cm forward from base_link
            '--y', '0.0',      # 0cm left from base_link
            '--z', '0.05',     # 5cm up from base_link (ESP32 height)
            '--roll', '0.0',   # No rotation
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ]
    )

    # Static transform: base_link -> laser
    # RPLIDAR A1 is mounted at the front-center of the robot
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.15',     # 15cm forward from base_link
            '--y', '0.0',      # Centered
            '--z', '0.12',     # 12cm up from base_link
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ]
    )

    # Static transform: base_link -> sonar_left_link
    # Left ultrasonic sensor (HC-SR04)
    base_to_sonar_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_sonar_left_tf',
        arguments=[
            '--x', '0.20',     # 20cm forward from base_link
            '--y', '0.10',     # 10cm left from base_link
            '--z', '0.08',     # 8cm up from base_link
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.524',  # 30 degrees left (0.524 radians)
            '--frame-id', 'base_link',
            '--child-frame-id', 'sonar_left_link'
        ]
    )

    # Static transform: base_link -> sonar_center_link
    # Center ultrasonic sensor (HC-SR04)
    base_to_sonar_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_sonar_center_tf',
        arguments=[
            '--x', '0.22',     # 22cm forward from base_link (front)
            '--y', '0.0',      # Centered
            '--z', '0.08',     # 8cm up from base_link
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',    # Pointing straight forward
            '--frame-id', 'base_link',
            '--child-frame-id', 'sonar_center_link'
        ]
    )

    # Static transform: base_link -> sonar_right_link
    # Right ultrasonic sensor (HC-SR04)
    base_to_sonar_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_sonar_right_tf',
        arguments=[
            '--x', '0.20',     # 20cm forward from base_link
            '--y', '-0.10',    # 10cm right from base_link
            '--z', '0.08',     # 8cm up from base_link
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '-0.524', # 30 degrees right (-0.524 radians)
            '--frame-id', 'base_link',
            '--child-frame-id', 'sonar_right_link'
        ]
    )

    return LaunchDescription([
        base_to_imu,
        base_to_laser,
        base_to_sonar_left,
        base_to_sonar_center,
        base_to_sonar_right,
    ])
