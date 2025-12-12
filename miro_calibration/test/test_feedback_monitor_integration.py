#!/usr/bin/env python3
"""
Integration tests for Feedback Monitor node.

Tests real-time monitoring with published messages from cmd_vel and odometry topics.
"""

import unittest
import time
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# Import generated messages (will be available after build)
try:
    from miro_msgs.msg import VelocityFeedback, VelocityStats
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False

# Import node class
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from miro_calibration.feedback_monitor import FeedbackMonitor


class TestPublisher(Node):
    """Helper node to publish test data."""

    def __init__(self):
        super().__init__('test_publisher')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        """Publish commanded velocity."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_odometry(self, linear_x: float, angular_z: float):
        """Publish odometry with actual velocity."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = linear_x
        msg.twist.twist.angular.z = angular_z
        self.odom_pub.publish(msg)


class TestSubscriber(Node):
    """Helper node to receive feedback messages."""

    def __init__(self):
        super().__init__('test_subscriber')

        self.feedback_received = Event()
        self.stats_received = Event()
        self.latest_feedback = None
        self.latest_stats = None
        self.feedback_count = 0
        self.stats_count = 0

        if MSGS_AVAILABLE:
            self.feedback_sub = self.create_subscription(
                VelocityFeedback,
                '/calibration/velocity_feedback',
                self._feedback_callback,
                10
            )

            self.stats_sub = self.create_subscription(
                VelocityStats,
                '/calibration/velocity_stats',
                self._stats_callback,
                10
            )

    def _feedback_callback(self, msg):
        """Store received feedback message."""
        self.latest_feedback = msg
        self.feedback_count += 1
        self.feedback_received.set()

    def _stats_callback(self, msg):
        """Store received statistics message."""
        self.latest_stats = msg
        self.stats_count += 1
        self.stats_received.set()

    def reset(self):
        """Reset event flags."""
        self.feedback_received.clear()
        self.stats_received.clear()


@unittest.skipIf(not MSGS_AVAILABLE, "Message definitions not built yet")
class TestFeedbackMonitorIntegration(unittest.TestCase):
    """Integration tests for feedback monitor with real message passing."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create executor."""
        rclpy.init()
        cls.executor = MultiThreadedExecutor()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2."""
        rclpy.shutdown()

    def setUp(self):
        """Create nodes for each test."""
        self.monitor_node = FeedbackMonitor()
        self.publisher_node = TestPublisher()
        self.subscriber_node = TestSubscriber()

        # Add nodes to executor
        self.executor.add_node(self.monitor_node)
        self.executor.add_node(self.publisher_node)
        self.executor.add_node(self.subscriber_node)

        # Spin in background
        import threading
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # Allow time for node initialization
        time.sleep(0.5)

    def tearDown(self):
        """Destroy nodes after each test."""
        self.executor.remove_node(self.monitor_node)
        self.executor.remove_node(self.publisher_node)
        self.executor.remove_node(self.subscriber_node)

        self.monitor_node.destroy_node()
        self.publisher_node.destroy_node()
        self.subscriber_node.destroy_node()

    def test_feedback_publication_rate(self):
        """Test that feedback is published at 10Hz."""
        # Publish some velocity data
        self.publisher_node.publish_cmd_vel(0.2, 0.0)
        self.publisher_node.publish_odometry(0.19, 0.0)

        # Wait for multiple feedback messages
        time.sleep(1.5)  # Allow 1.5 seconds for feedback messages

        # Should receive approximately 15 messages (10Hz * 1.5s)
        # Allow some tolerance for timing variations
        self.assertGreater(self.subscriber_node.feedback_count, 12)
        self.assertLess(self.subscriber_node.feedback_count, 18)

    def test_feedback_message_content(self):
        """Test that feedback message contains correct error calculations."""
        # Publish specific velocities
        cmd_linear = 0.2
        actual_linear = 0.18
        cmd_angular = 0.5
        actual_angular = 0.45

        self.publisher_node.publish_cmd_vel(cmd_linear, cmd_angular)
        self.publisher_node.publish_odometry(actual_linear, actual_angular)

        # Wait for feedback
        self.subscriber_node.reset()
        success = self.subscriber_node.feedback_received.wait(timeout=0.5)
        self.assertTrue(success, "No feedback message received")

        feedback = self.subscriber_node.latest_feedback

        # Verify commanded and actual values
        self.assertAlmostEqual(feedback.cmd_linear_x, cmd_linear, places=5)
        self.assertAlmostEqual(feedback.actual_linear_x, actual_linear, places=5)
        self.assertAlmostEqual(feedback.cmd_angular_z, cmd_angular, places=5)
        self.assertAlmostEqual(feedback.actual_angular_z, actual_angular, places=5)

        # Verify error calculations
        expected_linear_error = abs(actual_linear - cmd_linear)
        expected_linear_percent = (expected_linear_error / cmd_linear) * 100

        self.assertAlmostEqual(feedback.linear_error_abs, expected_linear_error, places=5)
        self.assertAlmostEqual(feedback.linear_error_percent, expected_linear_percent, places=1)

    def test_feedback_status_green(self):
        """Test that GREEN status is set for low error."""
        # Publish velocities with < 5% error
        self.publisher_node.publish_cmd_vel(0.2, 0.0)
        self.publisher_node.publish_odometry(0.198, 0.0)  # 1% error

        # Wait for feedback
        self.subscriber_node.reset()
        success = self.subscriber_node.feedback_received.wait(timeout=0.5)
        self.assertTrue(success)

        feedback = self.subscriber_node.latest_feedback
        self.assertEqual(feedback.status, VelocityFeedback.GREEN)
        self.assertEqual(feedback.status_message, "EXCELLENT")

    def test_feedback_status_yellow(self):
        """Test that YELLOW status is set for medium error."""
        # Publish velocities with 5-10% error
        self.publisher_node.publish_cmd_vel(0.2, 0.0)
        self.publisher_node.publish_odometry(0.185, 0.0)  # 7.5% error

        # Wait for feedback
        self.subscriber_node.reset()
        success = self.subscriber_node.feedback_received.wait(timeout=0.5)
        self.assertTrue(success)

        feedback = self.subscriber_node.latest_feedback
        self.assertEqual(feedback.status, VelocityFeedback.YELLOW)
        self.assertEqual(feedback.status_message, "ACCEPTABLE")

    def test_feedback_status_red(self):
        """Test that RED status is set for high error."""
        # Publish velocities with > 10% error
        self.publisher_node.publish_cmd_vel(0.2, 0.0)
        self.publisher_node.publish_odometry(0.15, 0.0)  # 25% error

        # Wait for feedback
        self.subscriber_node.reset()
        success = self.subscriber_node.feedback_received.wait(timeout=0.5)
        self.assertTrue(success)

        feedback = self.subscriber_node.latest_feedback
        self.assertEqual(feedback.status, VelocityFeedback.RED)
        self.assertEqual(feedback.status_message, "NEEDS TUNING")

    def test_statistics_publication_rate(self):
        """Test that statistics are published at 1Hz."""
        # Publish some velocity data
        self.publisher_node.publish_cmd_vel(0.2, 0.0)
        self.publisher_node.publish_odometry(0.19, 0.0)

        # Wait for statistics messages
        time.sleep(2.5)  # Allow 2.5 seconds for stats messages

        # Should receive approximately 2-3 messages (1Hz * 2.5s)
        self.assertGreaterEqual(self.subscriber_node.stats_count, 2)
        self.assertLessEqual(self.subscriber_node.stats_count, 4)

    def test_statistics_content(self):
        """Test that statistics message contains valid data."""
        # Publish varying velocity data to generate statistics
        for i in range(20):
            self.publisher_node.publish_cmd_vel(0.2, 0.0)
            # Vary actual velocity slightly
            actual = 0.2 + (i % 3 - 1) * 0.01
            self.publisher_node.publish_odometry(actual, 0.0)
            time.sleep(0.05)  # Small delay between publishes

        # Wait for statistics
        self.subscriber_node.reset()
        success = self.subscriber_node.stats_received.wait(timeout=2.0)
        self.assertTrue(success, "No statistics message received")

        stats = self.subscriber_node.latest_stats

        # Verify statistics fields are populated
        self.assertGreater(stats.sample_count, 0)
        self.assertGreaterEqual(stats.linear_error_mean, 0.0)
        self.assertGreaterEqual(stats.linear_error_std_dev, 0.0)
        self.assertGreaterEqual(stats.linear_error_max, stats.linear_error_min)

    def test_zero_velocity_handling(self):
        """Test handling of zero commanded velocity."""
        # Publish zero commanded velocity
        self.publisher_node.publish_cmd_vel(0.0, 0.0)
        self.publisher_node.publish_odometry(0.01, 0.0)

        # Wait for feedback
        self.subscriber_node.reset()
        success = self.subscriber_node.feedback_received.wait(timeout=0.5)
        self.assertTrue(success)

        feedback = self.subscriber_node.latest_feedback

        # Should not crash and percentage should be 0
        self.assertEqual(feedback.linear_error_percent, 0.0)

    def test_continuous_monitoring(self):
        """Test continuous monitoring over multiple velocity changes."""
        test_velocities = [
            (0.1, 0.0, 0.095, 0.0),
            (0.2, 0.0, 0.19, 0.0),
            (0.3, 0.0, 0.27, 0.0),
            (0.2, 0.5, 0.19, 0.48),
        ]

        for cmd_lin, cmd_ang, act_lin, act_ang in test_velocities:
            self.subscriber_node.reset()

            self.publisher_node.publish_cmd_vel(cmd_lin, cmd_ang)
            self.publisher_node.publish_odometry(act_lin, act_ang)

            # Wait for feedback
            success = self.subscriber_node.feedback_received.wait(timeout=0.5)
            self.assertTrue(success, f"No feedback for cmd=({cmd_lin}, {cmd_ang})")

            feedback = self.subscriber_node.latest_feedback
            self.assertIsNotNone(feedback)

            # Verify values are updated
            self.assertAlmostEqual(feedback.cmd_linear_x, cmd_lin, places=5)
            self.assertAlmostEqual(feedback.actual_linear_x, act_lin, places=5)


if __name__ == '__main__':
    unittest.main()
