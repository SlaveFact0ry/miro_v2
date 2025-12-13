#!/usr/bin/env python3
"""
Sensor Health Monitor Node for MIRO v2 Calibration System

Monitors RPLIDAR and IMU sensor health, tracking frequency, quality metrics,
and generating warnings for degraded performance or sensor disconnection.

Publishes:
    /calibration/sensor_status (miro_msgs/SensorStatus): Overall sensor health
    /calibration/rplidar_stats (miro_msgs/RPLIDARStats): Detailed RPLIDAR metrics
    /calibration/imu_stats (miro_msgs/IMUStats): Detailed IMU metrics

Subscribes:
    /scan (sensor_msgs/LaserScan): RPLIDAR scan data
    /imu/data (sensor_msgs/Imu): IMU sensor data

Parameters:
    rplidar_expected_hz (float): Expected RPLIDAR scan rate (default: 10.0)
    imu_expected_hz (float): Expected IMU data rate (default: 20.0)
    rate_tolerance_percent (float): Rate tolerance percentage (default: 20.0)
    sensor_timeout_sec (float): Sensor timeout threshold (default: 1.0)
    min_points (int): Minimum expected points per scan (default: 100)
    max_range_variance (float): Maximum acceptable range variance (default: 0.5)
    max_gyro_bias (float): Maximum acceptable gyro bias rad/s (default: 0.02)
    max_accel_noise (float): Maximum acceleration noise m/s² (default: 0.5)
    status_publish_rate_hz (float): Status publish rate (default: 1.0)
    frequency_window_sec (float): Frequency calculation window (default: 5.0)
"""

import math
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, Imu
from miro_msgs.msg import SensorStatus, RPLIDARStats, IMUStats


class SensorHealthMonitor(Node):
    """
    Monitor sensor health for RPLIDAR and IMU during calibration.

    Tracks frequency, quality metrics, and generates actionable warnings
    for sensor issues.
    """

    def __init__(self):
        super().__init__('sensor_health_monitor')

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        self.rplidar_expected_hz = self.get_parameter('rplidar_expected_hz').value
        self.imu_expected_hz = self.get_parameter('imu_expected_hz').value
        self.rate_tolerance_percent = self.get_parameter('rate_tolerance_percent').value
        self.sensor_timeout_sec = self.get_parameter('sensor_timeout_sec').value
        self.min_points = self.get_parameter('min_points').value
        self.max_range_variance = self.get_parameter('max_range_variance').value
        self.max_gyro_bias = self.get_parameter('max_gyro_bias').value
        self.max_accel_noise = self.get_parameter('max_accel_noise').value
        self.status_publish_rate_hz = self.get_parameter('status_publish_rate_hz').value
        self.frequency_window_sec = self.get_parameter('frequency_window_sec').value

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
            qos_profile
        )

        # Publishers
        self.sensor_status_pub = self.create_publisher(
            SensorStatus,
            '/calibration/sensor_status',
            qos_profile
        )

        self.rplidar_stats_pub = self.create_publisher(
            RPLIDARStats,
            '/calibration/rplidar_stats',
            qos_profile
        )

        self.imu_stats_pub = self.create_publisher(
            IMUStats,
            '/calibration/imu_stats',
            qos_profile
        )

        # Timer for status publishing
        self.status_timer = self.create_timer(
            1.0 / self.status_publish_rate_hz,
            self._status_timer_callback
        )

        # RPLIDAR state tracking
        self.rplidar_last_msg_time: Optional[Time] = None
        self.rplidar_timestamps = deque(maxlen=int(self.rplidar_expected_hz * self.frequency_window_sec))
        self.rplidar_point_counts = deque(maxlen=int(self.rplidar_expected_hz * self.frequency_window_sec))
        self.rplidar_latest_scan: Optional[LaserScan] = None
        self.rplidar_rate_hz: float = 0.0
        self.rplidar_point_count_avg: float = 0.0
        self.rplidar_range_variance: float = 0.0

        # IMU state tracking
        self.imu_last_msg_time: Optional[Time] = None
        self.imu_timestamps = deque(maxlen=int(self.imu_expected_hz * self.frequency_window_sec))
        self.imu_latest_data: Optional[Imu] = None
        self.imu_rate_hz: float = 0.0
        self.imu_gyro_bias_z: float = 0.0
        self.imu_accel_magnitude: float = 0.0

        # Warning tracking
        self.warnings: list[str] = []

        self.get_logger().info(
            f'Sensor Health Monitor initialized: '
            f'RPLIDAR={self.rplidar_expected_hz}Hz, IMU={self.imu_expected_hz}Hz, '
            f'status_rate={self.status_publish_rate_hz}Hz'
        )

    def _declare_parameters(self):
        """Declare all node parameters with default values."""
        self.declare_parameter('rplidar_expected_hz', 10.0)
        self.declare_parameter('imu_expected_hz', 20.0)
        self.declare_parameter('rate_tolerance_percent', 20.0)
        self.declare_parameter('sensor_timeout_sec', 1.0)
        self.declare_parameter('min_points', 100)
        self.declare_parameter('max_range_variance', 0.5)
        self.declare_parameter('max_gyro_bias', 0.02)
        self.declare_parameter('max_accel_noise', 0.5)
        self.declare_parameter('status_publish_rate_hz', 1.0)
        self.declare_parameter('frequency_window_sec', 5.0)

    def _scan_callback(self, msg: LaserScan):
        """
        Process RPLIDAR scan data.

        Args:
            msg: LaserScan message from RPLIDAR
        """
        current_time = Time.from_msg(msg.header.stamp)
        self.rplidar_last_msg_time = current_time
        self.rplidar_latest_scan = msg

        # Update timestamp window for frequency calculation
        self.rplidar_timestamps.append(current_time.nanoseconds / 1e9)

        # Calculate frequency from sliding window
        self.rplidar_rate_hz = self._calculate_frequency(self.rplidar_timestamps)

        # Track point count
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r)]
        point_count = len(valid_ranges)
        self.rplidar_point_counts.append(point_count)

        # Calculate average point count
        if len(self.rplidar_point_counts) > 0:
            self.rplidar_point_count_avg = sum(self.rplidar_point_counts) / len(self.rplidar_point_counts)

        # Calculate range variance using numpy for efficiency
        if len(valid_ranges) > 1:
            self.rplidar_range_variance = float(np.var(valid_ranges))
        else:
            self.rplidar_range_variance = 0.0

    def _imu_callback(self, msg: Imu):
        """
        Process IMU sensor data.

        Args:
            msg: Imu message from IMU sensor
        """
        current_time = Time.from_msg(msg.header.stamp)
        self.imu_last_msg_time = current_time
        self.imu_latest_data = msg

        # Update timestamp window for frequency calculation
        self.imu_timestamps.append(current_time.nanoseconds / 1e9)

        # Calculate frequency from sliding window
        self.imu_rate_hz = self._calculate_frequency(self.imu_timestamps)

        # Calculate gyro bias (z-axis when stationary)
        # This is a simple estimation - could be improved with stationary detection
        self.imu_gyro_bias_z = abs(msg.angular_velocity.z)

        # Calculate acceleration magnitude
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        self.imu_accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

    def _calculate_frequency(self, timestamps: deque) -> float:
        """
        Calculate frequency from timestamp sliding window.

        Args:
            timestamps: Deque of timestamps in seconds

        Returns:
            Calculated frequency in Hz, or 0.0 if insufficient samples
        """
        if len(timestamps) < 2:
            return 0.0

        # Calculate duration of window
        window_duration = timestamps[-1] - timestamps[0]

        if window_duration <= 0.0:
            return 0.0

        # Frequency = (number of messages - 1) / window_duration
        # We use (n-1) because n messages span (n-1) intervals
        frequency = (len(timestamps) - 1) / window_duration

        return frequency

    def _check_rplidar_health(self) -> tuple[bool, list[str]]:
        """
        Check RPLIDAR sensor health and generate warnings.

        Returns:
            Tuple of (connected, warnings_list)
        """
        warnings = []
        current_time = self.get_clock().now()

        # Check connection timeout
        if self.rplidar_last_msg_time is None:
            return False, ["RPLIDAR: No data received yet"]

        time_since_last = (current_time - self.rplidar_last_msg_time).nanoseconds / 1e9

        if time_since_last > self.sensor_timeout_sec:
            warnings.append(
                f"RPLIDAR: Sensor timeout ({time_since_last:.1f}s since last message, "
                f"expected <{self.sensor_timeout_sec}s)"
            )
            return False, warnings

        # Check scan rate
        expected_min = self.rplidar_expected_hz * (1.0 - self.rate_tolerance_percent / 100.0)
        expected_max = self.rplidar_expected_hz * (1.0 + self.rate_tolerance_percent / 100.0)

        if self.rplidar_rate_hz > 0.0 and (self.rplidar_rate_hz < expected_min or self.rplidar_rate_hz > expected_max):
            warnings.append(
                f"RPLIDAR: Scan rate out of range ({self.rplidar_rate_hz:.1f}Hz, "
                f"expected {self.rplidar_expected_hz}Hz ±{self.rate_tolerance_percent}%)"
            )

        # Check point count
        if self.rplidar_point_count_avg < self.min_points:
            warnings.append(
                f"RPLIDAR: Low point count ({self.rplidar_point_count_avg:.0f} points, "
                f"expected >{self.min_points} points). Check sensor alignment and obstacles."
            )

        # Check range variance
        if self.rplidar_range_variance > self.max_range_variance:
            warnings.append(
                f"RPLIDAR: High range variance ({self.rplidar_range_variance:.3f}, "
                f"expected <{self.max_range_variance}). Environment may be too cluttered."
            )

        return True, warnings

    def _check_imu_health(self) -> tuple[bool, list[str]]:
        """
        Check IMU sensor health and generate warnings.

        Returns:
            Tuple of (connected, warnings_list)
        """
        warnings = []
        current_time = self.get_clock().now()

        # Check connection timeout
        if self.imu_last_msg_time is None:
            return False, ["IMU: No data received yet"]

        time_since_last = (current_time - self.imu_last_msg_time).nanoseconds / 1e9

        if time_since_last > self.sensor_timeout_sec:
            warnings.append(
                f"IMU: Sensor timeout ({time_since_last:.1f}s since last message, "
                f"expected <{self.sensor_timeout_sec}s)"
            )
            return False, warnings

        # Check IMU rate
        expected_min = self.imu_expected_hz * (1.0 - self.rate_tolerance_percent / 100.0)
        expected_max = self.imu_expected_hz * (1.0 + self.rate_tolerance_percent / 100.0)

        if self.imu_rate_hz > 0.0 and (self.imu_rate_hz < expected_min or self.imu_rate_hz > expected_max):
            warnings.append(
                f"IMU: Data rate out of range ({self.imu_rate_hz:.1f}Hz, "
                f"expected {self.imu_expected_hz}Hz ±{self.rate_tolerance_percent}%)"
            )

        # Check gyro bias
        if self.imu_gyro_bias_z > self.max_gyro_bias:
            warnings.append(
                f"IMU: High gyro bias ({self.imu_gyro_bias_z:.4f} rad/s, "
                f"expected <{self.max_gyro_bias} rad/s). IMU may need recalibration."
            )

        # Check acceleration magnitude (should be close to gravity when stationary)
        expected_accel = 9.81  # m/s²
        accel_error = abs(self.imu_accel_magnitude - expected_accel)

        if accel_error > self.max_accel_noise:
            warnings.append(
                f"IMU: Abnormal acceleration magnitude ({self.imu_accel_magnitude:.2f} m/s², "
                f"expected ~{expected_accel:.2f} m/s² ±{self.max_accel_noise} m/s²). "
                f"Check IMU mounting and calibration."
            )

        return True, warnings

    def _publish_rplidar_stats(self, connected: bool):
        """
        Publish detailed RPLIDAR statistics.

        Args:
            connected: Whether RPLIDAR is connected
        """
        msg = RPLIDARStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'

        msg.connected = connected

        if self.rplidar_last_msg_time is not None:
            time_since_last = (self.get_clock().now() - self.rplidar_last_msg_time).nanoseconds / 1e9
            msg.time_since_last_msg = float(time_since_last)
        else:
            msg.time_since_last_msg = float('inf')

        msg.rate_hz = self.rplidar_rate_hz
        msg.expected_rate_hz = self.rplidar_expected_hz

        if self.rplidar_expected_hz > 0:
            msg.rate_error_percent = abs(self.rplidar_rate_hz - self.rplidar_expected_hz) / self.rplidar_expected_hz * 100.0
        else:
            msg.rate_error_percent = 0.0

        if self.rplidar_latest_scan is not None:
            valid_ranges = [r for r in self.rplidar_latest_scan.ranges if not math.isnan(r) and not math.isinf(r)]
            msg.point_count = len(valid_ranges)

            if len(valid_ranges) > 0:
                msg.range_min = float(min(valid_ranges))
                msg.range_max = float(max(valid_ranges))
                msg.range_mean = float(sum(valid_ranges) / len(valid_ranges))
            else:
                msg.range_min = 0.0
                msg.range_max = 0.0
                msg.range_mean = 0.0
        else:
            msg.point_count = 0
            msg.range_min = 0.0
            msg.range_max = 0.0
            msg.range_mean = 0.0

        msg.point_count_avg = self.rplidar_point_count_avg
        msg.min_point_count = self.min_points
        msg.range_variance = self.rplidar_range_variance
        msg.max_range_variance = self.max_range_variance

        # Determine overall health
        msg.healthy = (
            connected and
            msg.rate_error_percent < self.rate_tolerance_percent and
            msg.point_count_avg >= self.min_points and
            msg.range_variance <= self.max_range_variance
        )

        self.rplidar_stats_pub.publish(msg)

    def _publish_imu_stats(self, connected: bool):
        """
        Publish detailed IMU statistics.

        Args:
            connected: Whether IMU is connected
        """
        msg = IMUStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.connected = connected

        if self.imu_last_msg_time is not None:
            time_since_last = (self.get_clock().now() - self.imu_last_msg_time).nanoseconds / 1e9
            msg.time_since_last_msg = float(time_since_last)
        else:
            msg.time_since_last_msg = float('inf')

        msg.rate_hz = self.imu_rate_hz
        msg.expected_rate_hz = self.imu_expected_hz

        if self.imu_expected_hz > 0:
            msg.rate_error_percent = abs(self.imu_rate_hz - self.imu_expected_hz) / self.imu_expected_hz * 100.0
        else:
            msg.rate_error_percent = 0.0

        if self.imu_latest_data is not None:
            msg.gyro_x = self.imu_latest_data.angular_velocity.x
            msg.gyro_y = self.imu_latest_data.angular_velocity.y
            msg.gyro_z = self.imu_latest_data.angular_velocity.z

            msg.accel_x = self.imu_latest_data.linear_acceleration.x
            msg.accel_y = self.imu_latest_data.linear_acceleration.y
            msg.accel_z = self.imu_latest_data.linear_acceleration.z
        else:
            msg.gyro_x = 0.0
            msg.gyro_y = 0.0
            msg.gyro_z = 0.0
            msg.accel_x = 0.0
            msg.accel_y = 0.0
            msg.accel_z = 0.0

        msg.gyro_bias_z = self.imu_gyro_bias_z
        msg.max_gyro_bias = self.max_gyro_bias

        msg.accel_magnitude = self.imu_accel_magnitude
        msg.expected_accel_magnitude = 9.81
        msg.accel_error = abs(self.imu_accel_magnitude - 9.81)
        msg.max_accel_noise = self.max_accel_noise

        # Determine overall health
        msg.healthy = (
            connected and
            msg.rate_error_percent < self.rate_tolerance_percent and
            msg.gyro_bias_z <= self.max_gyro_bias and
            msg.accel_error <= self.max_accel_noise
        )

        self.imu_stats_pub.publish(msg)

    def _status_timer_callback(self):
        """
        Publish sensor health status at configured rate.

        Aggregates health checks for both sensors and publishes comprehensive status.
        """
        # Check sensor health
        rplidar_connected, rplidar_warnings = self._check_rplidar_health()
        imu_connected, imu_warnings = self._check_imu_health()

        # Aggregate warnings
        self.warnings = rplidar_warnings + imu_warnings

        # Publish overall status
        status_msg = SensorStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = 'base_link'

        status_msg.rplidar_connected = rplidar_connected
        status_msg.rplidar_rate_hz = self.rplidar_rate_hz
        status_msg.rplidar_point_count_avg = self.rplidar_point_count_avg
        status_msg.rplidar_range_variance = self.rplidar_range_variance

        status_msg.imu_connected = imu_connected
        status_msg.imu_rate_hz = self.imu_rate_hz
        status_msg.imu_gyro_bias_z = self.imu_gyro_bias_z
        status_msg.imu_accel_magnitude = self.imu_accel_magnitude

        status_msg.warnings = self.warnings

        self.sensor_status_pub.publish(status_msg)

        # Publish detailed stats
        self._publish_rplidar_stats(rplidar_connected)
        self._publish_imu_stats(imu_connected)

        # Log warnings if any
        if len(self.warnings) > 0:
            for warning in self.warnings:
                self.get_logger().warn(warning)


def main(args=None):
    """Main entry point for sensor health monitor node."""
    rclpy.init(args=args)

    node = SensorHealthMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
