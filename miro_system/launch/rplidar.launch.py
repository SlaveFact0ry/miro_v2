"""
RPLIDAR Launch File
Configures and launches the RPLIDAR sensor for autonomous pool cleaning robot.

This launch file configures the RPLIDAR to provide 360-degree laser scan data
at 10Hz for SLAM and navigation in pool environments (0-6m range).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for configurability
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='Serial port for RPLIDAR device'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for laser scan data (matches static_transforms.launch.py)'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode: Standard or Sensitivity'
    )

    # RPLIDAR node configuration
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            # Serial port configuration
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 115200,

            # Frame ID for TF integration
            'frame_id': LaunchConfiguration('frame_id'),

            # Scan configuration for pool environment
            'scan_mode': LaunchConfiguration('scan_mode'),
            'scan_frequency': 10.0,  # 10Hz scan rate

            # Angle range for 360-degree coverage
            'angle_min': 0.0,        # 0 radians
            'angle_max': 6.28318531, # 2*pi radians (360 degrees)

            # Range limits for pool environment
            'range_min': 0.2,        # 0.2 meters minimum
            'range_max': 6.0,        # 6.0 meters maximum

            # Quality and performance settings
            'inverted': False,        # Normal mounting orientation
            'angle_compensate': True, # Enable angle compensation for motor speed variations

            # Auto-reconnect on connection loss
            'auto_reconnect': True,
        }]
    )

    # Note: Static transform base_link -> laser is published by static_transforms.launch.py
    # This launch file only starts the RPLIDAR driver node

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        frame_id_arg,
        scan_mode_arg,

        # Nodes
        rplidar_node,
    ])
