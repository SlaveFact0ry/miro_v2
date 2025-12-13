#!/usr/bin/env python3
"""
Unit tests for Sensor Health Monitor Node

Tests frequency calculation, variance calculation, timeout detection,
and warning generation logic.
"""

import unittest
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Vector3
from miro_msgs.msg import SensorStatus, RPLIDARStats, IMUStats

from miro_calibration.sensor_health_monitor import SensorHealthMonitor


class TestFrequencyCalculation(unittest.TestCase):
    """Test frequency calculation logic."""

    def setUp(self):
        """Initialize ROS2 for testing."""
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        """Cleanup after tests."""
        pass

    def test_frequency_calculation_empty(self):
        """Test frequency calculation with empty window."""
        node = SensorHealthMonitor()
        timestamps = deque(maxlen=10)

        freq = node._calculate_frequency(timestamps)

        self.assertEqual(freq, 0.0, "Empty window should return 0.0 Hz")
        node.destroy_node()

    def test_frequency_calculation_single_sample(self):
        """Test frequency calculation with single sample."""
        node = SensorHealthMonitor()
        timestamps = deque(maxlen=10)
        timestamps.append(1.0)

        freq = node._calculate_frequency(timestamps)

        self.assertEqual(freq, 0.0, "Single sample should return 0.0 Hz")
        node.destroy_node()

    def test_frequency_calculation_10hz(self):
        """Test frequency calculation for 10 Hz signal."""
        node = SensorHealthMonitor()
        timestamps = deque(maxlen=100)

        # Simulate 10 Hz signal over 5 seconds (50 samples)
        for i in range(50):
            timestamps.append(i * 0.1)  # 10 Hz = 0.1 second interval

        freq = node._calculate_frequency(timestamps)

        # Allow 1% tolerance
        self.assertAlmostEqual(freq, 10.0, delta=0.1, msg="Should calculate ~10 Hz")
        node.destroy_node()

    def test_frequency_calculation_20hz(self):
        """Test frequency calculation for 20 Hz signal."""
        node = SensorHealthMonitor()
        timestamps = deque(maxlen=100)

        # Simulate 20 Hz signal over 5 seconds (100 samples)
        for i in range(100):
            timestamps.append(i * 0.05)  # 20 Hz = 0.05 second interval

        freq = node._calculate_frequency(timestamps)

        # Allow 1% tolerance
        self.assertAlmostEqual(freq, 20.0, delta=0.2, msg="Should calculate ~20 Hz")
        node.destroy_node()

    def test_frequency_calculation_irregular(self):
        """Test frequency calculation with irregular timing."""
        node = SensorHealthMonitor()
        timestamps = deque(maxlen=100)

        # Simulate irregular timing
        timestamps.extend([0.0, 0.1, 0.15, 0.3, 0.35, 0.5])

        freq = node._calculate_frequency(timestamps)

        # (6 samples - 1 intervals) / (0.5 - 0.0) = 5 / 0.5 = 10 Hz average
        self.assertAlmostEqual(freq, 10.0, delta=0.1, msg="Should calculate average frequency")
        node.destroy_node()


class TestRangeVarianceCalculation(unittest.TestCase):
    """Test range variance calculation."""

    def setUp(self):
        """Initialize ROS2 for testing."""
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        """Cleanup after tests."""
        pass

    def test_scan_callback_variance_uniform(self):
        """Test variance calculation for uniform ranges."""
        node = SensorHealthMonitor()

        # Create scan with uniform ranges
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [1.0] * 100  # All points at 1 meter

        node._scan_callback(scan)

        # Variance of uniform data should be ~0
        self.assertLess(node.rplidar_range_variance, 0.01,
                       "Variance of uniform data should be near zero")
        node.destroy_node()

    def test_scan_callback_variance_varied(self):
        """Test variance calculation for varied ranges."""
        node = SensorHealthMonitor()

        # Create scan with varied ranges
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]

        node._scan_callback(scan)

        # Calculate expected variance
        import numpy as np
        expected_variance = float(np.var([0.5, 1.0, 1.5, 2.0, 2.5, 3.0]))

        self.assertAlmostEqual(node.rplidar_range_variance, expected_variance, places=3,
                              msg="Variance should match numpy calculation")
        node.destroy_node()

    def test_scan_callback_variance_with_invalid(self):
        """Test variance calculation with invalid ranges (NaN, inf)."""
        node = SensorHealthMonitor()

        # Create scan with valid and invalid ranges
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [1.0, float('nan'), 2.0, float('inf'), 3.0, float('-inf')]

        node._scan_callback(scan)

        # Should only calculate variance from valid ranges [1.0, 2.0, 3.0]
        import numpy as np
        expected_variance = float(np.var([1.0, 2.0, 3.0]))

        self.assertAlmostEqual(node.rplidar_range_variance, expected_variance, places=3,
                              msg="Should ignore NaN and inf values")
        node.destroy_node()


class TestTimeoutDetection(unittest.TestCase):
    """Test sensor timeout detection."""

    def setUp(self):
        """Initialize ROS2 for testing."""
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        """Cleanup after tests."""
        pass

    def test_rplidar_timeout_no_data(self):
        """Test RPLIDAR timeout when no data received."""
        node = SensorHealthMonitor()

        connected, warnings = node._check_rplidar_health()

        self.assertFalse(connected, "Should be disconnected with no data")
        self.assertGreater(len(warnings), 0, "Should generate warning")
        self.assertIn("No data received", warnings[0], "Warning should mention no data")
        node.destroy_node()

    def test_imu_timeout_no_data(self):
        """Test IMU timeout when no data received."""
        node = SensorHealthMonitor()

        connected, warnings = node._check_imu_health()

        self.assertFalse(connected, "Should be disconnected with no data")
        self.assertGreater(len(warnings), 0, "Should generate warning")
        self.assertIn("No data received", warnings[0], "Warning should mention no data")
        node.destroy_node()

    def test_rplidar_connected_recent_data(self):
        """Test RPLIDAR connected when receiving recent data."""
        node = SensorHealthMonitor()

        # Send recent scan data
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [1.0] * 100
        node._scan_callback(scan)

        # Wait a bit but not enough to timeout
        import time
        time.sleep(0.1)

        connected, warnings = node._check_rplidar_health()

        self.assertTrue(connected, "Should be connected with recent data")
        node.destroy_node()

    def test_imu_connected_recent_data(self):
        """Test IMU connected when receiving recent data."""
        node = SensorHealthMonitor()

        # Send recent IMU data
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.linear_acceleration.z = 9.81
        node._imu_callback(imu)

        # Wait a bit but not enough to timeout
        import time
        time.sleep(0.1)

        connected, warnings = node._check_imu_health()

        self.assertTrue(connected, "Should be connected with recent data")
        node.destroy_node()


class TestWarningGeneration(unittest.TestCase):
    """Test warning message generation."""

    def setUp(self):
        """Initialize ROS2 for testing."""
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        """Cleanup after tests."""
        pass

    def test_rplidar_low_point_count_warning(self):
        """Test warning generation for low point count."""
        node = SensorHealthMonitor()
        node.min_points = 100

        # Send scan with low point count
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [1.0] * 50  # Only 50 points
        node._scan_callback(scan)

        connected, warnings = node._check_rplidar_health()

        # Should have warning about low point count
        point_count_warnings = [w for w in warnings if "Low point count" in w]
        self.assertGreater(len(point_count_warnings), 0, "Should warn about low point count")
        self.assertIn("50", point_count_warnings[0], "Warning should include actual count")
        node.destroy_node()

    def test_rplidar_high_variance_warning(self):
        """Test warning generation for high range variance."""
        node = SensorHealthMonitor()
        node.max_range_variance = 0.5

        # Send scan with high variance
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [0.1, 5.0, 0.2, 4.8, 0.3, 4.5]  # High variance
        node._scan_callback(scan)

        connected, warnings = node._check_rplidar_health()

        # Should have warning about high variance
        variance_warnings = [w for w in warnings if "High range variance" in w]
        self.assertGreater(len(variance_warnings), 0, "Should warn about high variance")
        node.destroy_node()

    def test_imu_high_gyro_bias_warning(self):
        """Test warning generation for high gyro bias."""
        node = SensorHealthMonitor()
        node.max_gyro_bias = 0.02

        # Send IMU data with high gyro bias
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.angular_velocity.z = 0.05  # High bias
        imu.linear_acceleration.z = 9.81
        node._imu_callback(imu)

        connected, warnings = node._check_imu_health()

        # Should have warning about gyro bias
        gyro_warnings = [w for w in warnings if "High gyro bias" in w]
        self.assertGreater(len(gyro_warnings), 0, "Should warn about high gyro bias")
        self.assertIn("0.05", gyro_warnings[0], "Warning should include actual bias")
        node.destroy_node()

    def test_imu_abnormal_acceleration_warning(self):
        """Test warning generation for abnormal acceleration magnitude."""
        node = SensorHealthMonitor()
        node.max_accel_noise = 0.5

        # Send IMU data with abnormal acceleration
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.x = 5.0  # Abnormal
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        node._imu_callback(imu)

        connected, warnings = node._check_imu_health()

        # Should have warning about acceleration
        accel_warnings = [w for w in warnings if "Abnormal acceleration" in w]
        self.assertGreater(len(accel_warnings), 0, "Should warn about abnormal acceleration")
        node.destroy_node()

    def test_multiple_warnings(self):
        """Test multiple warnings generated simultaneously."""
        node = SensorHealthMonitor()

        # Send scan with multiple issues
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [0.1, 5.0] * 25  # Low count AND high variance
        node._scan_callback(scan)

        connected, warnings = node._check_rplidar_health()

        # Should have multiple warnings
        self.assertGreater(len(warnings), 1, "Should generate multiple warnings")
        node.destroy_node()


class TestGyroBiasEstimation(unittest.TestCase):
    """Test gyroscope bias estimation."""

    def setUp(self):
        """Initialize ROS2 for testing."""
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        """Cleanup after tests."""
        pass

    def test_gyro_bias_zero(self):
        """Test gyro bias estimation when stationary."""
        node = SensorHealthMonitor()

        # Send IMU data with zero gyro
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.z = 9.81
        node._imu_callback(imu)

        self.assertEqual(node.imu_gyro_bias_z, 0.0, "Bias should be zero")
        node.destroy_node()

    def test_gyro_bias_positive(self):
        """Test gyro bias estimation with positive bias."""
        node = SensorHealthMonitor()

        # Send IMU data with positive bias
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.angular_velocity.z = 0.01
        imu.linear_acceleration.z = 9.81
        node._imu_callback(imu)

        self.assertEqual(node.imu_gyro_bias_z, 0.01, "Bias should match gyro value")
        node.destroy_node()

    def test_gyro_bias_negative(self):
        """Test gyro bias estimation with negative bias (absolute value)."""
        node = SensorHealthMonitor()

        # Send IMU data with negative bias
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.angular_velocity.z = -0.015
        imu.linear_acceleration.z = 9.81
        node._imu_callback(imu)

        # Bias is stored as absolute value
        self.assertEqual(node.imu_gyro_bias_z, 0.015, "Bias should be absolute value")
        node.destroy_node()


class TestAccelerationMagnitude(unittest.TestCase):
    """Test acceleration magnitude calculation."""

    def setUp(self):
        """Initialize ROS2 for testing."""
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        """Cleanup after tests."""
        pass

    def test_accel_magnitude_gravity_z(self):
        """Test acceleration magnitude when gravity is on z-axis."""
        node = SensorHealthMonitor()

        # Send IMU data with gravity on z-axis
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81
        node._imu_callback(imu)

        self.assertAlmostEqual(node.imu_accel_magnitude, 9.81, places=2,
                              msg="Magnitude should be 9.81 m/s²")
        node.destroy_node()

    def test_accel_magnitude_gravity_x(self):
        """Test acceleration magnitude when gravity is on x-axis."""
        node = SensorHealthMonitor()

        # Send IMU data with gravity on x-axis (tilted sensor)
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.linear_acceleration.x = 9.81
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        node._imu_callback(imu)

        self.assertAlmostEqual(node.imu_accel_magnitude, 9.81, places=2,
                              msg="Magnitude should still be 9.81 m/s²")
        node.destroy_node()

    def test_accel_magnitude_3d(self):
        """Test acceleration magnitude with 3D components."""
        node = SensorHealthMonitor()

        # Send IMU data with 3D acceleration
        imu = Imu()
        imu.header.stamp = node.get_clock().now().to_msg()
        imu.linear_acceleration.x = 3.0
        imu.linear_acceleration.y = 4.0
        imu.linear_acceleration.z = 0.0
        node._imu_callback(imu)

        # sqrt(3^2 + 4^2 + 0^2) = 5.0
        self.assertAlmostEqual(node.imu_accel_magnitude, 5.0, places=2,
                              msg="Magnitude should be 5.0 m/s²")
        node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    unittest.main()
    rclpy.shutdown()
