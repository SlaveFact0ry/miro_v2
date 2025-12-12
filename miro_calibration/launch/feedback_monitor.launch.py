#!/usr/bin/env python3
"""
Launch file for Feedback Monitor node.

Starts the velocity feedback monitor with configuration from feedback_monitor.yaml.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for feedback monitor."""

    # Get package directory
    pkg_dir = get_package_share_directory('miro_calibration')
    config_file = os.path.join(pkg_dir, 'config', 'feedback_monitor.yaml')

    # Feedback Monitor Node
    feedback_monitor_node = Node(
        package='miro_calibration',
        executable='feedback_monitor',
        name='feedback_monitor',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,  # Enable terminal color output
    )

    return LaunchDescription([
        feedback_monitor_node,
    ])
