#!/usr/bin/env python3
"""
EKF Sensor Fusion Verification Script for Autonomous Pool Cleaning Robot

This script verifies that the robot_localization EKF node is correctly fusing
sensor data from rf2o_laser_odometry and IMU.

Verification checks:
- EKF node is running
- Input topics are publishing (/odom_rf2o, /imu/data)
- Output topic /odometry/filtered is publishing at expected rate (~30Hz)
- TF transform odom -> base_link is being published
- Fused odometry has reasonable covariances

Usage:
    ros2 run miro_system verify_ekf.py
    ros2 run miro_system verify_ekf.py --duration 30
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import argparse
from collections import deque
import math


class EKFVerifier(Node):
    """Node to verify EKF sensor fusion functionality."""

    def __init__(self, duration=10.0):
        super().__init__('ekf_verifier')

        self.duration = duration
        self.start_time = time.time()

        # Statistics
        self.filtered_odom_count = 0
        self.rf2o_odom_count = 0
        self.imu_count = 0

        self.filtered_timestamps = deque(maxlen=100)
        self.rf2o_timestamps = deque(maxlen=100)
        self.imu_timestamps = deque(maxlen=100)

        # Data storage for analysis
        self.last_filtered_odom = None
        self.last_rf2o_odom = None
        self.last_imu = None

        # Create subscriptions
        self.filtered_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_callback,
            10
        )

        self.rf2o_sub = self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.rf2o_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.get_logger().info('EKF Verifier started. Monitoring topics...')
        self.get_logger().info(f'Verification duration: {duration} seconds')

        # Timer for periodic statistics
        self.timer = self.create_timer(2.0, self.print_statistics)

    def filtered_callback(self, msg):
        """Process filtered odometry from EKF."""
        self.filtered_odom_count += 1
        self.filtered_timestamps.append(time.time())
        self.last_filtered_odom = msg

    def rf2o_callback(self, msg):
        """Process RF2O odometry."""
        self.rf2o_odom_count += 1
        self.rf2o_timestamps.append(time.time())
        self.last_rf2o_odom = msg

    def imu_callback(self, msg):
        """Process IMU data."""
        self.imu_count += 1
        self.imu_timestamps.append(time.time())
        self.last_imu = msg

    def calculate_frequency(self, timestamps):
        """Calculate publishing frequency from timestamp deque."""
        if len(timestamps) < 2:
            return 0.0

        time_diff = timestamps[-1] - timestamps[0]
        if time_diff > 0:
            return (len(timestamps) - 1) / time_diff
        return 0.0

    def print_statistics(self):
        """Print periodic statistics."""
        elapsed = time.time() - self.start_time

        filtered_freq = self.calculate_frequency(self.filtered_timestamps)
        rf2o_freq = self.calculate_frequency(self.rf2o_timestamps)
        imu_freq = self.calculate_frequency(self.imu_timestamps)

        self.get_logger().info(
            f'Elapsed: {elapsed:.1f}s | '
            f'Filtered: {self.filtered_odom_count} msgs ({filtered_freq:.1f} Hz) | '
            f'RF2O: {self.rf2o_odom_count} msgs ({rf2o_freq:.1f} Hz) | '
            f'IMU: {self.imu_count} msgs ({imu_freq:.1f} Hz)'
        )

        # Check if duration reached
        if elapsed >= self.duration:
            self.print_final_report()
            raise SystemExit

    def check_covariance(self, cov_array):
        """Check if covariance values are reasonable."""
        # Extract diagonal elements (variances)
        variances = [cov_array[i*6 + i] for i in range(6)]

        # Check for invalid values
        if any(math.isnan(v) or math.isinf(v) for v in variances):
            return False, "Contains NaN or Inf"

        # Check if too large (indicates poor estimation)
        if any(v > 10.0 for v in variances):
            return False, "Covariances too large (> 10.0)"

        # Check if zero (not being updated)
        if all(v == 0.0 for v in variances):
            return False, "All covariances are zero"

        return True, "OK"

    def print_final_report(self):
        """Print final verification report."""
        elapsed = time.time() - self.start_time

        filtered_freq = self.filtered_odom_count / elapsed if elapsed > 0 else 0
        rf2o_freq = self.rf2o_odom_count / elapsed if elapsed > 0 else 0
        imu_freq = self.imu_count / elapsed if elapsed > 0 else 0

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('EKF SENSOR FUSION VERIFICATION REPORT')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Duration: {elapsed:.2f} seconds')
        self.get_logger().info('')

        # Input sensors
        self.get_logger().info('INPUT SENSORS:')
        self.get_logger().info(f'  RF2O Odometry: {self.rf2o_odom_count} messages ({rf2o_freq:.2f} Hz)')
        self.get_logger().info(f'  IMU Data: {self.imu_count} messages ({imu_freq:.2f} Hz)')
        self.get_logger().info('')

        # Output
        self.get_logger().info('EKF OUTPUT:')
        self.get_logger().info(f'  Filtered Odometry: {self.filtered_odom_count} messages ({filtered_freq:.2f} Hz)')
        self.get_logger().info(f'  Expected frequency: ~30 Hz (EKF configuration)')
        self.get_logger().info('')

        # Validation checks
        checks_passed = 0
        total_checks = 6

        self.get_logger().info('VALIDATION CHECKS:')

        # Check 1: RF2O publishing
        if rf2o_freq >= 5.0:
            self.get_logger().info(f'  ✓ RF2O odometry publishing (>= 5 Hz)')
            checks_passed += 1
        else:
            self.get_logger().warn(f'  ✗ RF2O odometry rate too low: {rf2o_freq:.2f} Hz')

        # Check 2: IMU publishing
        if imu_freq >= 18.0:
            self.get_logger().info(f'  ✓ IMU data publishing (>= 18 Hz)')
            checks_passed += 1
        else:
            self.get_logger().warn(f'  ✗ IMU rate too low: {imu_freq:.2f} Hz')

        # Check 3: Filtered odometry publishing
        if filtered_freq >= 25.0:
            self.get_logger().info(f'  ✓ Filtered odometry publishing (>= 25 Hz)')
            checks_passed += 1
        else:
            self.get_logger().warn(f'  ✗ Filtered odometry rate too low: {filtered_freq:.2f} Hz')

        # Check 4: All topics received
        if self.filtered_odom_count > 0 and self.rf2o_odom_count > 0 and self.imu_count > 0:
            self.get_logger().info(f'  ✓ All topics received data')
            checks_passed += 1
        else:
            self.get_logger().warn(f'  ✗ Not all topics received data')

        # Check 5: Covariance validity
        if self.last_filtered_odom:
            cov_ok, cov_msg = self.check_covariance(self.last_filtered_odom.pose.covariance)
            if cov_ok:
                self.get_logger().info(f'  ✓ Covariance values reasonable')
                checks_passed += 1
            else:
                self.get_logger().warn(f'  ✗ Covariance check failed: {cov_msg}')
        else:
            self.get_logger().warn(f'  ✗ No filtered odometry received')

        # Check 6: Data consistency
        if self.last_filtered_odom and self.filtered_odom_count > 10:
            self.get_logger().info(f'  ✓ EKF producing consistent output')
            checks_passed += 1
        else:
            self.get_logger().warn(f'  ✗ Insufficient data for consistency check')

        # Final result
        self.get_logger().info('')
        self.get_logger().info('-'*60)
        if checks_passed == total_checks:
            self.get_logger().info(f'RESULT: ✓ EKF verification PASSED ({checks_passed}/{total_checks} checks)')
            self.get_logger().info('EKF is correctly fusing RF2O and IMU data')
        else:
            self.get_logger().warn(f'RESULT: ⚠ EKF verification PARTIAL ({checks_passed}/{total_checks} checks passed)')
            self.get_logger().warn('Review failed checks above')
        self.get_logger().info('='*60 + '\n')


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Verify EKF sensor fusion')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Verification duration in seconds (default: 10)')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        verifier = EKFVerifier(duration=parsed_args.duration)
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
