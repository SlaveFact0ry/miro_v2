import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'miro_control'
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'joystick.yaml'
    )

    return LaunchDescription([
        # 1. Joy Node (조이스틱 하드웨어 읽기)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # 2. Teleop Twist Joy (Joy -> cmd_vel 변환)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_file],
            remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])
