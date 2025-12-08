import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            parameters=[{
                'speed': 0.4, # 초기 속도 (m/s)
                'turn': 1.0   # 초기 회전 속도 (rad/s)
            }]
        )
    ])
