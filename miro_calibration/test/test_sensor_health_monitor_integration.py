#!/usr/bin/env python3
"""
Integration tests for Sensor Health Monitor Node

Tests the full node behavior including subscription, publishing,
and real-time monitoring with simulated sensor data.
"""

import unittest
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Vector3
from miro_msgs.msg import SensorStatus, RPLIDARStats, IMUStats

from miro_calibration.sensor_health_monitor import SensorHealthMonitor


class TestSensorHealthMonitorIntegration(unittest.TestCase):
    """Integration tests for sensor health monitor."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Set up test node and executor."""
        self.node = SensorHealthMonitor()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Create test publishers
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.node.create_publisher(Imu, '/imu/data', 10)

        # Create test subscribers
        self.status_received = None
        self.rplidar_stats_received = None
        self.imu_stats_received = None

        self.status_sub = self.node.create_subscription(
            SensorStatus,
            '/calibration/sensor_status',
            self._status_callback,
            10
        )

        self.rplidar_stats_sub = self.node.create_subscription(
            RPLIDARStats,
            '/calibration/rplidar_stats',
            self._rplidar_stats_callback,
            10
        )

        self.imu_stats_sub = self.node.create_subscription(
            IMUStats,
            '/calibration/imu_stats',
            self._imu_stats_callback,
            10
        )

        # Allow time for subscriptions to establish
        time.sleep(0.2)

    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()

    def _status_callback(self, msg):
        """Callback for sensor status messages."""
        self.status_received = msg

    def _rplidar_stats_callback(self, msg):
        """Callback for RPLIDAR stats messages."""
        self.rplidar_stats_received = msg

    def _imu_stats_callback(self, msg):
        """Callback for IMU stats messages."""
        self.imu_stats_received = msg

    def _spin_for_duration(self, duration_sec):
        """Spin the executor for a specified duration."""
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            self.executor.spin_once(timeout_sec=0.1)

    def test_node_initialization(self):
        """Test that node initializes correctly."""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'sensor_health_monitor')

    def test_publish_status_with_no_sensor_data(self):
        """Test status publishing when no sensor data received."""
        # Reset received messages
        self.status_received = None

        # Spin to allow timer callback to execute
        self._spin_for_duration(1.5)

        # Should have received status message
        self.assertIsNotNone(self.status_received, "Should receive status message")
        self.assertFalse(self.status_received.rplidar_connected,
                        "RPLIDAR should be disconnected")
        self.assertFalse(self.status_received.imu_connected,
                        "IMU should be disconnected")
        self.assertGreater(len(self.status_received.warnings), 0,
                          "Should have warnings")

    def test_rplidar_monitoring(self):
        """Test RPLIDAR monitoring with simulated scan data."""
        # Reset received messages
        self.rplidar_stats_received = None

        # Publish simulated RPLIDAR scans at 10 Hz
        for i in range(15):
            scan = LaserScan()
            scan.header.stamp = self.node.get_clock().now().to_msg()
            scan.header.frame_id = 'laser'
            scan.angle_min = 0.0
            scan.angle_max = 2 * math.pi
            scan.angle_increment = 0.01
            scan.range_min = 0.15
            scan.range_max = 12.0
            scan.ranges = [1.0 + 0.1 * math.sin(i * 0.1)] * 200  # 200 points with slight variation

            self.scan_pub.publish(scan)
            time.sleep(0.1)  # 10 Hz
            self.executor.spin_once(timeout_sec=0.01)

        # Wait for status to be published
        self._spin_for_duration(1.5)

        # Check RPLIDAR stats
        self.assertIsNotNone(self.rplidar_stats_received, "Should receive RPLIDAR stats")
        self.assertTrue(self.rplidar_stats_received.connected,
                       "RPLIDAR should be connected")
        self.assertGreater(self.rplidar_stats_received.rate_hz, 8.0,
                          "Rate should be approximately 10 Hz")
        self.assertLess(self.rplidar_stats_received.rate_hz, 12.0,
                       "Rate should be approximately 10 Hz")
        self.assertGreaterEqual(self.rplidar_stats_received.point_count, 200,
                               "Point count should be 200")

    def test_imu_monitoring(self):
        """Test IMU monitoring with simulated IMU data."""
        # Reset received messages
        self.imu_stats_received = None

        # Publish simulated IMU data at 20 Hz
        for i in range(30):
            imu = Imu()
            imu.header.stamp = self.node.get_clock().now().to_msg()
            imu.header.frame_id = 'imu_link'
            imu.angular_velocity.x = 0.0
            imu.angular_velocity.y = 0.0
            imu.angular_velocity.z = 0.001  # Small bias
            imu.linear_acceleration.x = 0.0
            imu.linear_acceleration.y = 0.0
            imu.linear_acceleration.z = 9.81

            self.imu_pub.publish(imu)
            time.sleep(0.05)  # 20 Hz
            self.executor.spin_once(timeout_sec=0.01)

        # Wait for status to be published
        self._spin_for_duration(1.5)

        # Check IMU stats
        self.assertIsNotNone(self.imu_stats_received, "Should receive IMU stats")
        self.assertTrue(self.imu_stats_received.connected,
                       "IMU should be connected")
        self.assertGreater(self.imu_stats_received.rate_hz, 15.0,
                          "Rate should be approximately 20 Hz")
        self.assertLess(self.imu_stats_received.rate_hz, 25.0,
                       "Rate should be approximately 20 Hz")
        self.assertAlmostEqual(self.imu_stats_received.accel_magnitude, 9.81, delta=0.1,
                              msg="Acceleration magnitude should be ~9.81 m/s²")

    def test_sensor_timeout_detection(self):
        """Test sensor timeout detection."""
        # Send initial data
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.ranges = [1.0] * 100
        self.scan_pub.publish(scan)

        imu = Imu()
        imu.header.stamp = self.node.get_clock().now().to_msg()
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        # Spin to process messages
        self._spin_for_duration(0.5)

        # Wait for timeout (1 second default)
        time.sleep(1.2)

        # Reset received messages
        self.status_received = None

        # Spin to get new status
        self._spin_for_duration(1.5)

        # Both sensors should be timed out
        self.assertIsNotNone(self.status_received, "Should receive status")
        self.assertFalse(self.status_received.rplidar_connected,
                        "RPLIDAR should timeout")
        self.assertFalse(self.status_received.imu_connected,
                        "IMU should timeout")
        self.assertGreater(len(self.status_received.warnings), 0,
                          "Should have timeout warnings")

    def test_warning_generation_low_point_count(self):
        """Test warning generation for low RPLIDAR point count."""
        # Reset received messages
        self.status_received = None

        # Publish scans with low point count
        for i in range(15):
            scan = LaserScan()
            scan.header.stamp = self.node.get_clock().now().to_msg()
            scan.ranges = [1.0] * 50  # Only 50 points (below min_points=100)

            self.scan_pub.publish(scan)
            time.sleep(0.1)
            self.executor.spin_once(timeout_sec=0.01)

        # Wait for status
        self._spin_for_duration(1.5)

        # Should have warning about low point count
        self.assertIsNotNone(self.status_received, "Should receive status")
        low_point_warnings = [w for w in self.status_received.warnings
                             if "Low point count" in w]
        self.assertGreater(len(low_point_warnings), 0,
                          "Should warn about low point count")

    def test_warning_generation_high_gyro_bias(self):
        """Test warning generation for high gyro bias."""
        # Reset received messages
        self.status_received = None

        # Publish IMU data with high gyro bias
        for i in range(30):
            imu = Imu()
            imu.header.stamp = self.node.get_clock().now().to_msg()
            imu.angular_velocity.z = 0.05  # High bias (above max_gyro_bias=0.02)
            imu.linear_acceleration.z = 9.81

            self.imu_pub.publish(imu)
            time.sleep(0.05)
            self.executor.spin_once(timeout_sec=0.01)

        # Wait for status
        self._spin_for_duration(1.5)

        # Should have warning about gyro bias
        self.assertIsNotNone(self.status_received, "Should receive status")
        gyro_warnings = [w for w in self.status_received.warnings
                        if "High gyro bias" in w]
        self.assertGreater(len(gyro_warnings), 0,
                          "Should warn about high gyro bias")

    def test_healthy_sensors(self):
        """Test that healthy sensors report as healthy."""
        # Publish healthy RPLIDAR data
        for i in range(15):
            scan = LaserScan()
            scan.header.stamp = self.node.get_clock().now().to_msg()
            scan.ranges = [1.0 + 0.05 * math.sin(i * 0.1)] * 200

            self.scan_pub.publish(scan)
            time.sleep(0.1)
            self.executor.spin_once(timeout_sec=0.01)

        # Publish healthy IMU data
        for i in range(30):
            imu = Imu()
            imu.header.stamp = self.node.get_clock().now().to_msg()
            imu.angular_velocity.z = 0.001  # Low bias
            imu.linear_acceleration.z = 9.81

            self.imu_pub.publish(imu)
            time.sleep(0.05)
            self.executor.spin_once(timeout_sec=0.01)

        # Wait for stats
        self._spin_for_duration(1.5)

        # Both sensors should be healthy
        self.assertIsNotNone(self.rplidar_stats_received, "Should receive RPLIDAR stats")
        self.assertIsNotNone(self.imu_stats_received, "Should receive IMU stats")
        self.assertTrue(self.rplidar_stats_received.healthy,
                       "RPLIDAR should be healthy")
        self.assertTrue(self.imu_stats_received.healthy,
                       "IMU should be healthy")


if __name__ == '__main__':
    unittest.main()
