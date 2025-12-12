#!/usr/bin/env python3
"""
Unit tests for Feedback Monitor node.

Tests error calculation, status determination, and statistics tracking.
"""

import unittest
import math
from unittest.mock import MagicMock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import node class
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from miro_calibration.feedback_monitor import FeedbackMonitor


class TestErrorCalculation(unittest.TestCase):
    """Test error calculation logic."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Create node instance for each test."""
        self.node = FeedbackMonitor()
        self.node.min_velocity_threshold = 0.01

    def tearDown(self):
        """Destroy node after each test."""
        self.node.destroy_node()

    def test_calculate_error_normal_velocity(self):
        """Test error calculation with normal velocities."""
        cmd_value = 0.2
        actual_value = 0.19

        abs_error, percent_error = self.node._calculate_error(cmd_value, actual_value)

        self.assertAlmostEqual(abs_error, 0.01, places=5)
        self.assertAlmostEqual(percent_error, 5.0, places=1)

    def test_calculate_error_zero_commanded(self):
        """Test error calculation when commanded velocity is zero."""
        cmd_value = 0.0
        actual_value = 0.05

        abs_error, percent_error = self.node._calculate_error(cmd_value, actual_value)

        self.assertAlmostEqual(abs_error, 0.05, places=5)
        self.assertEqual(percent_error, 0.0)  # Should avoid division by zero

    def test_calculate_error_below_threshold(self):
        """Test error calculation when velocity is below threshold."""
        cmd_value = 0.005  # Below 0.01 threshold
        actual_value = 0.004

        abs_error, percent_error = self.node._calculate_error(cmd_value, actual_value)

        self.assertAlmostEqual(abs_error, 0.001, places=5)
        self.assertEqual(percent_error, 0.0)  # Should return 0 for small velocities

    def test_calculate_error_negative_velocities(self):
        """Test error calculation with negative (reverse) velocities."""
        cmd_value = -0.2
        actual_value = -0.18

        abs_error, percent_error = self.node._calculate_error(cmd_value, actual_value)

        self.assertAlmostEqual(abs_error, 0.02, places=5)
        self.assertAlmostEqual(percent_error, 10.0, places=1)

    def test_calculate_error_large_error(self):
        """Test error calculation with large tracking error."""
        cmd_value = 0.3
        actual_value = 0.15

        abs_error, percent_error = self.node._calculate_error(cmd_value, actual_value)

        self.assertAlmostEqual(abs_error, 0.15, places=5)
        self.assertAlmostEqual(percent_error, 50.0, places=1)

    def test_calculate_error_perfect_tracking(self):
        """Test error calculation with perfect velocity tracking."""
        cmd_value = 0.2
        actual_value = 0.2

        abs_error, percent_error = self.node._calculate_error(cmd_value, actual_value)

        self.assertAlmostEqual(abs_error, 0.0, places=5)
        self.assertAlmostEqual(percent_error, 0.0, places=1)


class TestStatusDetermination(unittest.TestCase):
    """Test status determination logic."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for all tests."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Create node instance for each test."""
        self.node = FeedbackMonitor()
        self.node.error_threshold_green = 0.05  # 5%
        self.node.error_threshold_yellow = 0.10  # 10%

    def tearDown(self):
        """Destroy node after each test."""
        self.node.destroy_node()

    def test_status_green(self):
        """Test GREEN status determination (error < 5%)."""
        from miro_msgs.msg import VelocityFeedback

        linear_percent = 3.0
        angular_percent = 2.5

        status, status_msg = self.node._determine_status(linear_percent, angular_percent)

        self.assertEqual(status, VelocityFeedback.GREEN)
        self.assertEqual(status_msg, "EXCELLENT")

    def test_status_yellow(self):
        """Test YELLOW status determination (5% <= error < 10%)."""
        from miro_msgs.msg import VelocityFeedback

        linear_percent = 7.0
        angular_percent = 3.0

        status, status_msg = self.node._determine_status(linear_percent, angular_percent)

        self.assertEqual(status, VelocityFeedback.YELLOW)
        self.assertEqual(status_msg, "ACCEPTABLE")

    def test_status_red(self):
        """Test RED status determination (error >= 10%)."""
        from miro_msgs.msg import VelocityFeedback

        linear_percent = 15.0
        angular_percent = 5.0

        status, status_msg = self.node._determine_status(linear_percent, angular_percent)

        self.assertEqual(status, VelocityFeedback.RED)
        self.assertEqual(status_msg, "NEEDS TUNING")

    def test_status_boundary_green_yellow(self):
        """Test status at boundary between GREEN and YELLOW."""
        from miro_msgs.msg import VelocityFeedback

        linear_percent = 4.99
        angular_percent = 0.0

        status, _ = self.node._determine_status(linear_percent, angular_percent)
        self.assertEqual(status, VelocityFeedback.GREEN)

        linear_percent = 5.01
        status, _ = self.node._determine_status(linear_percent, angular_percent)
        self.assertEqual(status, VelocityFeedback.YELLOW)

    def test_status_boundary_yellow_red(self):
        """Test status at boundary between YELLOW and RED."""
        from miro_msgs.msg import VelocityFeedback

        linear_percent = 9.99
        angular_percent = 0.0

        status, _ = self.node._determine_status(linear_percent, angular_percent)
        self.assertEqual(status, VelocityFeedback.YELLOW)

        linear_percent = 10.01
        status, _ = self.node._determine_status(linear_percent, angular_percent)
        self.assertEqual(status, VelocityFeedback.RED)

    def test_status_uses_max_error(self):
        """Test that status uses maximum error from linear and angular."""
        from miro_msgs.msg import VelocityFeedback

        # Angular error is higher and determines status
        linear_percent = 3.0
        angular_percent = 12.0

        status, _ = self.node._determine_status(linear_percent, angular_percent)
        self.assertEqual(status, VelocityFeedback.RED)


class TestWelfordStatistics(unittest.TestCase):
    """Test Welford's algorithm for online variance calculation."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for all tests."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Create node instance for each test."""
        self.node = FeedbackMonitor()

    def tearDown(self):
        """Destroy node after each test."""
        self.node.destroy_node()

    def test_welford_single_sample(self):
        """Test Welford statistics with single sample."""
        self.node._update_welford_stats(0.05, 0.02)

        self.assertEqual(self.node.linear_error_count, 1)
        self.assertAlmostEqual(self.node.linear_error_mean, 0.05, places=5)

        # Variance calculation requires n > 1
        variance = self.node.linear_error_m2 / (self.node.linear_error_count - 1) if self.node.linear_error_count > 1 else 0.0
        self.assertEqual(variance, 0.0)

    def test_welford_multiple_samples(self):
        """Test Welford statistics with multiple samples."""
        samples = [0.01, 0.02, 0.03, 0.04, 0.05]

        for sample in samples:
            self.node._update_welford_stats(sample, sample)

        # Calculate expected mean and std dev
        expected_mean = sum(samples) / len(samples)
        expected_variance = sum((x - expected_mean) ** 2 for x in samples) / (len(samples) - 1)
        expected_std_dev = math.sqrt(expected_variance)

        self.assertEqual(self.node.linear_error_count, len(samples))
        self.assertAlmostEqual(self.node.linear_error_mean, expected_mean, places=5)

        calculated_variance = self.node.linear_error_m2 / (self.node.linear_error_count - 1)
        calculated_std_dev = math.sqrt(calculated_variance)

        self.assertAlmostEqual(calculated_std_dev, expected_std_dev, places=5)

    def test_welford_zero_variance(self):
        """Test Welford statistics with constant values (zero variance)."""
        constant_value = 0.15

        for _ in range(10):
            self.node._update_welford_stats(constant_value, constant_value)

        self.assertEqual(self.node.linear_error_count, 10)
        self.assertAlmostEqual(self.node.linear_error_mean, constant_value, places=5)

        # Variance should be very close to zero
        variance = self.node.linear_error_m2 / (self.node.linear_error_count - 1)
        self.assertAlmostEqual(variance, 0.0, places=10)

    def test_welford_large_variance(self):
        """Test Welford statistics with high variance samples."""
        samples = [0.01, 0.50, 0.02, 0.45, 0.03]

        for sample in samples:
            self.node._update_welford_stats(sample, sample)

        expected_mean = sum(samples) / len(samples)
        expected_variance = sum((x - expected_mean) ** 2 for x in samples) / (len(samples) - 1)

        calculated_variance = self.node.linear_error_m2 / (self.node.linear_error_count - 1)

        self.assertAlmostEqual(calculated_variance, expected_variance, places=5)


class TestTerminalFormatting(unittest.TestCase):
    """Test terminal output formatting."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for all tests."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Create node instance for each test."""
        self.node = FeedbackMonitor()
        from miro_msgs.msg import VelocityFeedback
        self.feedback_msg = VelocityFeedback()
        self.feedback_msg.cmd_linear_x = 0.2
        self.feedback_msg.cmd_angular_z = 0.0
        self.feedback_msg.actual_linear_x = 0.19
        self.feedback_msg.actual_angular_z = 0.0
        self.feedback_msg.linear_error_percent = 5.0
        self.feedback_msg.angular_error_percent = 0.0
        self.feedback_msg.status = VelocityFeedback.GREEN
        self.feedback_msg.status_message = "EXCELLENT"

    def tearDown(self):
        """Destroy node after each test."""
        self.node.destroy_node()

    def test_format_with_ansi_colors(self):
        """Test terminal formatting with ANSI colors enabled."""
        self.node.use_ansi_colors = True
        self.node.current_speed_mode = "MEDIUM"

        output = self.node._format_terminal_output(self.feedback_msg)

        # Should contain ANSI escape codes
        self.assertIn('\033[', output)
        self.assertIn('MEDIUM', output)
        self.assertIn('EXCELLENT', output)

    def test_format_without_ansi_colors(self):
        """Test terminal formatting with ANSI colors disabled."""
        self.node.use_ansi_colors = False
        self.node.current_speed_mode = "SLOW"

        output = self.node._format_terminal_output(self.feedback_msg)

        # Should not contain ANSI escape codes
        self.assertNotIn('\033[', output)
        self.assertIn('SLOW', output)
        self.assertIn('EXCELLENT', output)

    def test_format_contains_all_data(self):
        """Test that formatted output contains all required data."""
        self.node.use_ansi_colors = False
        self.node.current_speed_mode = "FAST"

        output = self.node._format_terminal_output(self.feedback_msg)

        # Check for velocity values
        self.assertIn('0.200', output)
        self.assertIn('0.190', output)

        # Check for percentage
        self.assertIn('5.0%', output)

        # Check for status
        self.assertIn('EXCELLENT', output)


if __name__ == '__main__':
    unittest.main()
