#!/usr/bin/env python3
"""
IMU Data Verification Script for Autonomous Pool Cleaning Robot

This script monitors the /imu/data topic from the ESP32 micro-ROS node
and verifies data quality, frequency, and validity.

Verification checks:
- Publishing frequency (~20Hz expected from ESP32 firmware)
- Data validity (no NaN values)
- Orientation quaternion normalization
- Excessive tilt detection for safety
- Message statistics and diagnostics

Usage:
    ros2 run miro_system verify_imu.py
    ros2 run miro_system verify_imu.py --duration 30
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import argparse
from collections import deque


class IMUVerifier(Node):
    """Node to verify IMU data quality and frequency."""

    def __init__(self, duration=10.0):
        super().__init__('imu_verifier')

        self.duration = duration
        self.start_time = time.time()

        # Statistics
        self.msg_count = 0
        self.error_count = 0
        self.nan_count = 0
        self.timestamps = deque(maxlen=100)  # Last 100 timestamps for frequency calc

        # Data ranges for validation
        self.max_tilt_angle = math.radians(15.0)  # 15 degrees max tilt

        # Create subscription
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.get_logger().info('IMU Verifier started. Listening to /imu/data...')
        self.get_logger().info(f'Verification duration: {duration} seconds')

        # Timer for periodic statistics
        self.timer = self.create_timer(1.0, self.print_statistics)

    def imu_callback(self, msg):
        """Process incoming IMU message and verify data."""
        self.msg_count += 1
        current_time = time.time()
        self.timestamps.append(current_time)

        # Check for NaN values
        if self.check_nan_values(msg):
            self.nan_count += 1
            self.get_logger().warn('IMU message contains NaN values')
            return

        # Verify quaternion normalization
        if not self.verify_quaternion(msg.orientation):
            self.error_count += 1
            self.get_logger().warn('Quaternion is not normalized')

        # Check for excessive tilt
        if self.check_excessive_tilt(msg):
            self.get_logger().warn('WARNING: Excessive tilt detected!')

        # Check frame_id
        if msg.header.frame_id != 'imu_link':
            self.get_logger().warn(f'Unexpected frame_id: {msg.header.frame_id}, expected "imu_link"')

    def check_nan_values(self, msg):
        """Check if message contains NaN values."""
        values = [
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ]
        return any(math.isnan(v) for v in values)

    def verify_quaternion(self, quat):
        """Verify that quaternion is normalized (magnitude close to 1.0)."""
        magnitude = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        return abs(magnitude - 1.0) < 0.01  # Allow 1% tolerance

    def check_excessive_tilt(self, msg):
        """Check if robot is tilted beyond safe limits."""
        # Convert quaternion to roll and pitch
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        # Roll (rotation around X-axis)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (rotation around Y-axis)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Check if tilt exceeds limits
        if abs(roll) > self.max_tilt_angle or abs(pitch) > self.max_tilt_angle:
            self.get_logger().warn(
                f'Excessive tilt - Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°'
            )
            return True
        return False

    def calculate_frequency(self):
        """Calculate current publishing frequency."""
        if len(self.timestamps) < 2:
            return 0.0

        time_diff = self.timestamps[-1] - self.timestamps[0]
        if time_diff > 0:
            return (len(self.timestamps) - 1) / time_diff
        return 0.0

    def print_statistics(self):
        """Print periodic statistics."""
        elapsed = time.time() - self.start_time
        frequency = self.calculate_frequency()

        self.get_logger().info(
            f'Messages: {self.msg_count} | Frequency: {frequency:.2f} Hz | '
            f'Errors: {self.error_count} | NaN: {self.nan_count} | '
            f'Elapsed: {elapsed:.1f}s'
        )

        # Check if duration reached
        if elapsed >= self.duration:
            self.print_final_report()
            raise SystemExit

    def print_final_report(self):
        """Print final verification report."""
        elapsed = time.time() - self.start_time
        avg_frequency = self.msg_count / elapsed if elapsed > 0 else 0

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('IMU VERIFICATION REPORT')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Duration: {elapsed:.2f} seconds')
        self.get_logger().info(f'Total messages: {self.msg_count}')
        self.get_logger().info(f'Average frequency: {avg_frequency:.2f} Hz')
        self.get_logger().info(f'Expected frequency: ~20 Hz (ESP32 timer @ 50ms)')
        self.get_logger().info(f'Error count: {self.error_count}')
        self.get_logger().info(f'NaN count: {self.nan_count}')

        # Success criteria
        frequency_ok = 18.0 <= avg_frequency <= 22.0  # 20Hz ± 2Hz tolerance
        error_rate = (self.error_count / self.msg_count * 100) if self.msg_count > 0 else 0
        quality_ok = error_rate < 5.0  # Less than 5% error rate

        self.get_logger().info('\n' + '-'*60)
        if frequency_ok and quality_ok and self.nan_count == 0:
            self.get_logger().info('RESULT: ✓ IMU verification PASSED')
            self.get_logger().info('IMU is publishing valid data at expected frequency')
        else:
            self.get_logger().warn('RESULT: ✗ IMU verification FAILED')
            if not frequency_ok:
                self.get_logger().warn(f'  - Frequency out of range: {avg_frequency:.2f} Hz')
            if not quality_ok:
                self.get_logger().warn(f'  - High error rate: {error_rate:.2f}%')
            if self.nan_count > 0:
                self.get_logger().warn(f'  - NaN values detected: {self.nan_count} messages')
        self.get_logger().info('='*60 + '\n')


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Verify IMU data quality')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Verification duration in seconds (default: 10)')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        verifier = IMUVerifier(duration=parsed_args.duration)
        rclpy.spin(verifier)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
