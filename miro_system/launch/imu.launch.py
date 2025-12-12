"""
IMU Driver Launch Configuration for Autonomous Pool Cleaning Robot

This launch file provides a generic IMU driver configuration that can be adapted
to different IMU hardware models commonly used in robotics applications.

Supported IMU types:
- mpu6050: MPU6050/MPU9250 (I2C)
- bno055: Bosch BNO055 (I2C)
- xsens_mti: Xsens MTi series (USB/UART)
- imu_filter: Generic IMU with complementary filter

Usage:
    ros2 launch miro_system imu.launch.py imu_type:=mpu6050
    ros2 launch miro_system imu.launch.py imu_type:=bno055
    ros2 launch miro_system imu.launch.py imu_type:=xsens_mti
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_imu_nodes(context, *args, **kwargs):
    """Generate IMU nodes based on the selected IMU type."""

    imu_type = LaunchConfiguration('imu_type').perform(context)
    use_filter = LaunchConfiguration('use_filter').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)

    nodes = []

    # Static transform: base_link -> imu_link
    # IMU is typically mounted on the robot's center, slightly above the base
    # Adjust these values based on your actual IMU mounting position
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '--x', '0.0',      # 0cm forward from base_link
            '--y', '0.0',      # 0cm left from base_link
            '--z', '0.05',     # 5cm up from base_link
            '--roll', '0.0',   # No rotation
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ]
    )
    nodes.append(static_transform)

    # IMU driver node based on type
    if imu_type == 'mpu6050':
        # MPU6050/MPU9250 driver
        imu_driver = Node(
            package='mpu6050_driver',
            executable='mpu6050_node',
            name='imu_driver',
            parameters=[config_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
        nodes.append(imu_driver)

    elif imu_type == 'bno055':
        # Bosch BNO055 driver
        imu_driver = Node(
            package='bno055',
            executable='bno055_node',
            name='imu_driver',
            parameters=[config_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
        nodes.append(imu_driver)

    elif imu_type == 'xsens_mti':
        # Xsens MTi driver
        imu_driver = Node(
            package='xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            parameters=[config_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
        nodes.append(imu_driver)

    elif imu_type == 'imu_filter':
        # Generic raw IMU data node (placeholder for custom driver)
        # Replace this with your actual IMU driver node
        imu_driver = Node(
            package='miro_system',
            executable='imu_raw_publisher.py',
            name='imu_raw_publisher',
            parameters=[config_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            condition=UnlessCondition(use_filter)
        )
        nodes.append(imu_driver)

    # IMU Complementary Filter (optional, for sensor fusion)
    # Use this if your IMU doesn't provide fused orientation
    if use_filter == 'true':
        imu_filter = Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='imu_filter',
            parameters=[{
                'use_mag': True,
                'do_bias_estimation': True,
                'do_adaptive_gain': True,
                'gain_acc': 0.01,
                'gain_mag': 0.01,
                'publish_tf': False,
                'fixed_frame': 'odom',
                'publish_debug_topics': True
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/data', 'imu/data'),
                ('imu/mag', 'imu/mag')
            ],
            output='screen'
        )
        nodes.append(imu_filter)

    # IMU data monitor (publishes diagnostics)
    imu_monitor = Node(
        package='miro_system',
        executable='imu_monitor.py',
        name='imu_monitor',
        parameters=[{
            'expected_frequency': 50.0,
            'frequency_tolerance': 5.0,
            'orientation_tolerance': 0.035  # 2 degrees in radians
        }],
        output='screen'
    )
    # Note: imu_monitor.py needs to be created separately
    # nodes.append(imu_monitor)

    return nodes


def generate_launch_description():
    """Generate launch description for IMU driver."""

    # Declare launch arguments
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='imu_filter',
        description='IMU driver type: mpu6050, bno055, xsens_mti, or imu_filter'
    )

    use_filter_arg = DeclareLaunchArgument(
        'use_filter',
        default_value='false',
        description='Use complementary filter for sensor fusion (true/false)'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('miro_system'),
            'config',
            'imu.yaml'
        ]),
        description='Path to IMU configuration file'
    )

    # OpaqueFunction to generate nodes based on launch arguments
    imu_nodes = OpaqueFunction(function=generate_imu_nodes)

    return LaunchDescription([
        imu_type_arg,
        use_filter_arg,
        config_file_arg,
        imu_nodes
    ])
