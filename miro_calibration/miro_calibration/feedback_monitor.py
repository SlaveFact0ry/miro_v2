#!/usr/bin/env python3
"""
Feedback Monitor Node for MIRO v2 Calibration System

Monitors commanded vs actual velocity in real-time at 10Hz with color-coded
terminal feedback and publishes statistics at 1Hz.

Requirements: REQ-2 (Real-time Velocity Feedback Display)
"""

import math
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from miro_msgs.msg import VelocityFeedback, VelocityStats, SpeedMode


class FeedbackMonitor(Node):
    """
    Real-time velocity feedback monitor for calibration assessment.

    Subscribes to commanded (/cmd_vel) and actual (/odometry/filtered) velocities,
    calculates error metrics, and publishes feedback with color-coded status.
    """

    # ANSI color codes for terminal output
    COLOR_GREEN = '\033[92m'
    COLOR_YELLOW = '\033[93m'
    COLOR_RED = '\033[91m'
    COLOR_RESET = '\033[0m'
    COLOR_BOLD = '\033[1m'

    def __init__(self):
        super().__init__('feedback_monitor')

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        self.feedback_rate_hz = self.get_parameter('feedback_rate_hz').value
        self.stats_rate_hz = self.get_parameter('stats_rate_hz').value
        self.error_threshold_green = self.get_parameter('error_threshold_green').value
        self.error_threshold_yellow = self.get_parameter('error_threshold_yellow').value
        self.stats_window_sec = self.get_parameter('stats_window_sec').value
        self.enable_terminal_output = self.get_parameter('enable_terminal_output').value
        self.use_ansi_colors = self.get_parameter('use_ansi_colors').value
        self.min_velocity_threshold = self.get_parameter('min_velocity_threshold').value

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            qos_profile
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_callback,
            qos_profile
        )

        self.speed_mode_sub = self.create_subscription(
            SpeedMode,
            '/calibration/speed_mode',
            self._speed_mode_callback,
            qos_profile
        )

        # Publishers
        self.feedback_pub = self.create_publisher(
            VelocityFeedback,
            '/calibration/velocity_feedback',
            qos_profile
        )

        self.stats_pub = self.create_publisher(
            VelocityStats,
            '/calibration/velocity_stats',
            qos_profile
        )

        # Timers
        self.feedback_timer = self.create_timer(
            1.0 / self.feedback_rate_hz,
            self._feedback_timer_callback
        )

        self.stats_timer = self.create_timer(
            1.0 / self.stats_rate_hz,
            self._stats_timer_callback
        )

        # State variables (latest values)
        self.cmd_linear_x: float = 0.0
        self.cmd_angular_z: float = 0.0
        self.actual_linear_x: float = 0.0
        self.actual_angular_z: float = 0.0
        self.current_speed_mode: str = "UNKNOWN"

        # Pre-allocated message objects to avoid allocations in hot path
        self.feedback_msg = VelocityFeedback()

        # Statistics tracking using sliding window
        self.stats_window_size = int(self.stats_window_sec * self.feedback_rate_hz)
        self.linear_error_window = deque(maxlen=self.stats_window_size)
        self.angular_error_window = deque(maxlen=self.stats_window_size)
        self.linear_percent_window = deque(maxlen=self.stats_window_size)
        self.angular_percent_window = deque(maxlen=self.stats_window_size)

        # Welford's algorithm state for online variance calculation
        self.linear_error_count = 0
        self.linear_error_mean = 0.0
        self.linear_error_m2 = 0.0

        self.angular_error_count = 0
        self.angular_error_mean = 0.0
        self.angular_error_m2 = 0.0

        self.get_logger().info(
            f'Feedback Monitor initialized: '
            f'feedback_rate={self.feedback_rate_hz}Hz, '
            f'stats_rate={self.stats_rate_hz}Hz, '
            f'window={self.stats_window_sec}s'
        )

    def _declare_parameters(self):
        """Declare all node parameters with default values."""
        self.declare_parameter('feedback_rate_hz', 10.0)
        self.declare_parameter('stats_rate_hz', 1.0)
        self.declare_parameter('error_threshold_green', 0.05)  # 5%
        self.declare_parameter('error_threshold_yellow', 0.10)  # 10%
        self.declare_parameter('stats_window_sec', 10.0)
        self.declare_parameter('enable_terminal_output', True)
        self.declare_parameter('use_ansi_colors', True)
        self.declare_parameter('min_velocity_threshold', 0.01)  # Minimum velocity to calculate percentage

    def _cmd_vel_callback(self, msg: Twist):
        """Store commanded velocity from /cmd_vel."""
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

    def _odom_callback(self, msg: Odometry):
        """Store actual velocity from /odometry/filtered."""
        self.actual_linear_x = msg.twist.twist.linear.x
        self.actual_angular_z = msg.twist.twist.angular.z

    def _speed_mode_callback(self, msg: SpeedMode):
        """Store current speed mode for context."""
        self.current_speed_mode = msg.mode_name

    def _calculate_error(
        self,
        cmd_value: float,
        actual_value: float
    ) -> tuple[float, float]:
        """
        Calculate absolute and percentage error.

        Args:
            cmd_value: Commanded velocity
            actual_value: Actual velocity

        Returns:
            Tuple of (absolute_error, percentage_error)
            Percentage is 0.0 if commanded velocity is below threshold.
        """
        absolute_error = abs(actual_value - cmd_value)

        # Avoid division by zero and percentage calculation for very small velocities
        if abs(cmd_value) < self.min_velocity_threshold:
            percentage_error = 0.0
        else:
            percentage_error = (absolute_error / abs(cmd_value)) * 100.0

        return absolute_error, percentage_error

    def _determine_status(
        self,
        linear_percent: float,
        angular_percent: float
    ) -> tuple[int, str]:
        """
        Determine color-coded status based on error thresholds.

        Args:
            linear_percent: Linear velocity error percentage
            angular_percent: Angular velocity error percentage

        Returns:
            Tuple of (status_code, status_message)
        """
        # Use the maximum error percentage for status determination
        max_error_percent = max(linear_percent, angular_percent)

        if max_error_percent < self.error_threshold_green * 100:
            status = VelocityFeedback.GREEN
            status_msg = "EXCELLENT"
        elif max_error_percent < self.error_threshold_yellow * 100:
            status = VelocityFeedback.YELLOW
            status_msg = "ACCEPTABLE"
        else:
            status = VelocityFeedback.RED
            status_msg = "NEEDS TUNING"

        return status, status_msg

    def _update_welford_stats(
        self,
        linear_error: float,
        angular_error: float
    ):
        """
        Update online statistics using Welford's algorithm.

        This algorithm computes variance in a single pass with numerical stability.

        Args:
            linear_error: New linear error sample
            angular_error: New angular error sample
        """
        # Update linear error statistics
        self.linear_error_count += 1
        delta = linear_error - self.linear_error_mean
        self.linear_error_mean += delta / self.linear_error_count
        delta2 = linear_error - self.linear_error_mean
        self.linear_error_m2 += delta * delta2

        # Update angular error statistics
        self.angular_error_count += 1
        delta = angular_error - self.angular_error_mean
        self.angular_error_mean += delta / self.angular_error_count
        delta2 = angular_error - self.angular_error_mean
        self.angular_error_m2 += delta * delta2

    def _format_terminal_output(
        self,
        feedback: VelocityFeedback
    ) -> str:
        """
        Format terminal output with optional ANSI colors.

        Args:
            feedback: VelocityFeedback message

        Returns:
            Formatted string for terminal display
        """
        # Select color based on status
        if self.use_ansi_colors:
            if feedback.status == VelocityFeedback.GREEN:
                color = self.COLOR_GREEN
            elif feedback.status == VelocityFeedback.YELLOW:
                color = self.COLOR_YELLOW
            else:
                color = self.COLOR_RED
            bold = self.COLOR_BOLD
            reset = self.COLOR_RESET
        else:
            color = bold = reset = ''

        # Format output string
        output = (
            f"{bold}[{self.current_speed_mode}]{reset} "
            f"Linear: {feedback.cmd_linear_x:+.3f} → {feedback.actual_linear_x:+.3f} m/s "
            f"({color}{feedback.linear_error_percent:5.1f}%{reset}) | "
            f"Angular: {feedback.cmd_angular_z:+.3f} → {feedback.actual_angular_z:+.3f} rad/s "
            f"({color}{feedback.angular_error_percent:5.1f}%{reset}) | "
            f"{color}{feedback.status_message}{reset}"
        )

        return output

    def _feedback_timer_callback(self):
        """
        Publish velocity feedback at 10Hz rate.

        Calculates error metrics, determines status, updates statistics,
        and publishes feedback message.
        """
        # Calculate errors
        linear_error_abs, linear_error_percent = self._calculate_error(
            self.cmd_linear_x,
            self.actual_linear_x
        )

        angular_error_abs, angular_error_percent = self._calculate_error(
            self.cmd_angular_z,
            self.actual_angular_z
        )

        # Determine status
        status, status_message = self._determine_status(
            linear_error_percent,
            angular_error_percent
        )

        # Update statistics windows
        self.linear_error_window.append(linear_error_abs)
        self.angular_error_window.append(angular_error_abs)
        self.linear_percent_window.append(linear_error_percent)
        self.angular_percent_window.append(angular_error_percent)

        # Update Welford statistics
        self._update_welford_stats(linear_error_abs, angular_error_abs)

        # Populate feedback message (reuse pre-allocated object)
        self.feedback_msg.header.stamp = self.get_clock().now().to_msg()
        self.feedback_msg.header.frame_id = 'base_link'

        self.feedback_msg.cmd_linear_x = self.cmd_linear_x
        self.feedback_msg.cmd_angular_z = self.cmd_angular_z
        self.feedback_msg.actual_linear_x = self.actual_linear_x
        self.feedback_msg.actual_angular_z = self.actual_angular_z

        self.feedback_msg.linear_error_abs = linear_error_abs
        self.feedback_msg.linear_error_percent = linear_error_percent
        self.feedback_msg.angular_error_abs = angular_error_abs
        self.feedback_msg.angular_error_percent = angular_error_percent

        self.feedback_msg.status = status
        self.feedback_msg.status_message = status_message

        # Publish feedback
        self.feedback_pub.publish(self.feedback_msg)

        # Terminal output
        if self.enable_terminal_output:
            output_str = self._format_terminal_output(self.feedback_msg)
            print(output_str)

    def _stats_timer_callback(self):
        """
        Publish velocity statistics at 1Hz rate.

        Computes statistics from sliding window and publishes to
        /calibration/velocity_stats topic.
        """
        if len(self.linear_error_window) == 0:
            return  # No data yet

        # Create statistics message
        stats_msg = VelocityStats()
        stats_msg.header.stamp = self.get_clock().now().to_msg()
        stats_msg.header.frame_id = 'base_link'

        stats_msg.window_duration_sec = self.stats_window_sec
        stats_msg.sample_count = len(self.linear_error_window)

        # Linear error statistics
        linear_errors = list(self.linear_error_window)
        stats_msg.linear_error_mean = sum(linear_errors) / len(linear_errors)
        stats_msg.linear_error_min = min(linear_errors)
        stats_msg.linear_error_max = max(linear_errors)

        # Calculate standard deviation from Welford's algorithm
        if self.linear_error_count > 1:
            variance = self.linear_error_m2 / (self.linear_error_count - 1)
            stats_msg.linear_error_std_dev = math.sqrt(variance)
        else:
            stats_msg.linear_error_std_dev = 0.0

        # Angular error statistics
        angular_errors = list(self.angular_error_window)
        stats_msg.angular_error_mean = sum(angular_errors) / len(angular_errors)
        stats_msg.angular_error_min = min(angular_errors)
        stats_msg.angular_error_max = max(angular_errors)

        # Calculate standard deviation from Welford's algorithm
        if self.angular_error_count > 1:
            variance = self.angular_error_m2 / (self.angular_error_count - 1)
            stats_msg.angular_error_std_dev = math.sqrt(variance)
        else:
            stats_msg.angular_error_std_dev = 0.0

        # Percentage statistics
        linear_percents = list(self.linear_percent_window)
        if linear_percents:
            stats_msg.linear_percent_mean = sum(linear_percents) / len(linear_percents)

        angular_percents = list(self.angular_percent_window)
        if angular_percents:
            stats_msg.angular_percent_mean = sum(angular_percents) / len(angular_percents)

        # Publish statistics
        self.stats_pub.publish(stats_msg)


def main(args=None):
    """Main entry point for feedback monitor node."""
    rclpy.init(args=args)

    node = FeedbackMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
