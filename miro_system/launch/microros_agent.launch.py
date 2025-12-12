"""
micro-ROS Agent Launch Configuration for Autonomous Pool Cleaning Robot

This launch file starts the micro-ROS agent to bridge communication between
ESP32 micro-ROS node and the ROS2 network on Raspberry Pi.

The ESP32 (miro_mcu_node) publishes:
- /imu/data (sensor_msgs/Imu) - IMU data from MPU9250 at 20Hz
- /ultrasonic/left, /ultrasonic/center, /ultrasonic/right (sensor_msgs/Range)

The ESP32 subscribes to:
- /cmd_vel (geometry_msgs/Twist) - Motor control commands

Transport: Serial USB connection (ESP32 typically appears as /dev/ttyACM0)

Usage:
    # Default serial connection
    ros2 launch miro_system microros_agent.launch.py

    # Custom device
    ros2 launch miro_system microros_agent.launch.py device:=/dev/ttyACM1

    # UDP WiFi connection (if ESP32 configured for WiFi)
    ros2 launch miro_system microros_agent.launch.py transport:=udp4 port:=8888
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for micro-ROS agent."""

    # Declare launch arguments
    transport_arg = DeclareLaunchArgument(
        'transport',
        default_value='serial',
        description='Transport type: serial (USB) or udp4 (WiFi)',
        choices=['serial', 'udp4']
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyACM0',
        description='Serial device path for ESP32 (typically /dev/ttyACM0)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate matching ESP32 firmware'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8888',
        description='UDP port (for udp4 transport)'
    )

    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='4',
        description='Verbosity level: 1-6 (1=FATAL, 2=ERROR, 3=WARN, 4=INFO, 5=DEBUG, 6=VERBOSE)'
    )

    # Get launch configurations
    transport = LaunchConfiguration('transport')
    device = LaunchConfiguration('device')
    baudrate = LaunchConfiguration('baudrate')
    port = LaunchConfiguration('port')
    verbosity = LaunchConfiguration('verbosity')

    # micro-ROS agent node - Serial transport
    microros_agent_serial = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=[
            'serial',
            '--dev', device,
            '-b', baudrate,
            '-v', verbosity
        ],
        condition=IfCondition(
            PythonExpression(["'", transport, "' == 'serial'"])
        )
    )

    # micro-ROS agent node - UDP transport
    microros_agent_udp = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=[
            'udp4',
            '--port', port,
            '-v', verbosity
        ],
        condition=IfCondition(
            PythonExpression(["'", transport, "' == 'udp4'"])
        )
    )

    # Log startup information
    log_serial = LogInfo(
        msg=['Starting micro-ROS agent with Serial transport on device: ', device, ' @ ', baudrate, ' baud'],
        condition=IfCondition(
            PythonExpression(["'", transport, "' == 'serial'"])
        )
    )

    log_udp = LogInfo(
        msg=['Starting micro-ROS agent with UDP transport on port: ', port],
        condition=IfCondition(
            PythonExpression(["'", transport, "' == 'udp4'"])
        )
    )

    return LaunchDescription([
        # Declare arguments
        transport_arg,
        device_arg,
        baudrate_arg,
        port_arg,
        verbosity_arg,

        # Log information
        log_serial,
        log_udp,

        # Launch nodes
        microros_agent_serial,
        microros_agent_udp,
    ])
