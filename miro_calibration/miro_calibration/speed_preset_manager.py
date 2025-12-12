#!/usr/bin/env python3
"""
Speed Preset Manager Node for MIRO v2 Calibration System

Subscribes to joystick input and maps button presses to fixed speed presets
for consistent calibration testing.

Publishes:
    /calibration/speed_mode (miro_msgs/SpeedMode): Current speed preset

Subscribes:
    /joy (sensor_msgs/Joy): Joystick input

Parameters:
    button_slow (int): Joystick button index for slow preset (default: 0)
    button_medium (int): Joystick button index for medium preset (default: 1)
    button_fast (int): Joystick button index for fast preset (default: 2)
    speed_slow (float): Linear speed for slow preset m/s (default: 0.1)
    speed_medium (float): Linear speed for medium preset m/s (default: 0.2)
    speed_fast (float): Linear speed for fast preset m/s (default: 0.3)
    angular_slow (float): Angular speed for slow preset rad/s (default: 0.2)
    angular_medium (float): Angular speed for medium preset rad/s (default: 0.4)
    angular_fast (float): Angular speed for fast preset rad/s (default: 0.6)
    default_preset (str): Default preset on startup (default: "medium")
    debounce_time (float): Debounce time in seconds (default: 0.3)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from miro_msgs.msg import SpeedMode
import time


class SpeedPresetManager(Node):
    """Manages fixed speed presets for calibration via joystick buttons."""

    def __init__(self):
        super().__init__('speed_preset_manager')

        # Declare parameters
        self.declare_parameter('button_slow', 0)
        self.declare_parameter('button_medium', 1)
        self.declare_parameter('button_fast', 2)
        self.declare_parameter('speed_slow', 0.1)
        self.declare_parameter('speed_medium', 0.2)
        self.declare_parameter('speed_fast', 0.3)
        self.declare_parameter('angular_slow', 0.2)
        self.declare_parameter('angular_medium', 0.4)
        self.declare_parameter('angular_fast', 0.6)
        self.declare_parameter('default_preset', 'medium')
        self.declare_parameter('debounce_time', 0.3)

        # Get parameters
        self.button_slow = self.get_parameter('button_slow').value
        self.button_medium = self.get_parameter('button_medium').value
        self.button_fast = self.get_parameter('button_fast').value

        self.speed_slow = self.get_parameter('speed_slow').value
        self.speed_medium = self.get_parameter('speed_medium').value
        self.speed_fast = self.get_parameter('speed_fast').value

        self.angular_slow = self.get_parameter('angular_slow').value
        self.angular_medium = self.get_parameter('angular_medium').value
        self.angular_fast = self.get_parameter('angular_fast').value

        default_preset = self.get_parameter('default_preset').value
        self.debounce_time = self.get_parameter('debounce_time').value

        # Speed presets configuration
        self.presets = {
            SpeedMode.SLOW: {
                'name': 'SLOW',
                'linear': self.speed_slow,
                'angular': self.angular_slow,
                'button': self.button_slow
            },
            SpeedMode.MEDIUM: {
                'name': 'MEDIUM',
                'linear': self.speed_medium,
                'angular': self.angular_medium,
                'button': self.button_medium
            },
            SpeedMode.FAST: {
                'name': 'FAST',
                'linear': self.speed_fast,
                'angular': self.angular_fast,
                'button': self.button_fast
            }
        }

        # Set default preset
        if default_preset.upper() == 'SLOW':
            self.current_mode = SpeedMode.SLOW
        elif default_preset.upper() == 'FAST':
            self.current_mode = SpeedMode.FAST
        else:
            self.current_mode = SpeedMode.MEDIUM

        # Debouncing state
        self.last_button_time = {
            self.button_slow: 0.0,
            self.button_medium: 0.0,
            self.button_fast: 0.0
        }
        self.last_button_state = {
            self.button_slow: 0,
            self.button_medium: 0,
            self.button_fast: 0
        }

        # Publishers
        self.speed_mode_pub = self.create_publisher(
            SpeedMode,
            '/calibration/speed_mode',
            10
        )

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publish initial speed mode
        self.publish_speed_mode()

        self.get_logger().info(
            f'Speed Preset Manager initialized with default: '
            f'{self.presets[self.current_mode]["name"]} '
            f'({self.presets[self.current_mode]["linear"]} m/s, '
            f'{self.presets[self.current_mode]["angular"]} rad/s)'
        )
        self.get_logger().info(
            f'Button mapping: B{self.button_slow}=SLOW, '
            f'B{self.button_medium}=MEDIUM, B{self.button_fast}=FAST'
        )

    def joy_callback(self, msg):
        """
        Process joystick input and detect button presses.

        Args:
            msg (sensor_msgs/Joy): Joystick message
        """
        current_time = time.time()

        # Check each preset button
        for mode, config in self.presets.items():
            button_idx = config['button']

            # Check if button index is valid
            if button_idx >= len(msg.buttons):
                continue

            button_state = msg.buttons[button_idx]

            # Detect button press (rising edge with debouncing)
            if (button_state == 1 and
                self.last_button_state[button_idx] == 0 and
                current_time - self.last_button_time[button_idx] > self.debounce_time):

                # Button pressed - change mode
                if mode != self.current_mode:
                    self.current_mode = mode
                    self.publish_speed_mode()

                    self.get_logger().info(
                        f'Speed preset changed to {config["name"]}: '
                        f'{config["linear"]} m/s, {config["angular"]} rad/s'
                    )

                self.last_button_time[button_idx] = current_time

            # Update button state
            self.last_button_state[button_idx] = button_state

    def publish_speed_mode(self):
        """Publish current speed mode to /calibration/speed_mode topic."""
        msg = SpeedMode()
        msg.mode = self.current_mode
        msg.linear_speed = self.presets[self.current_mode]['linear']
        msg.angular_speed = self.presets[self.current_mode]['angular']
        msg.timestamp = self.get_clock().now().to_msg()

        self.speed_mode_pub.publish(msg)


def main(args=None):
    """Main entry point for speed preset manager node."""
    rclpy.init(args=args)

    try:
        node = SpeedPresetManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
