#!/usr/bin/env python3
"""
TF Tree Verification Script for Autonomous Pool Cleaning Robot

This script verifies that the complete TF tree is correctly established
with all required transforms.

TF Tree Structure:
    map (SLAM/AMCL publishes this during mapping/cleaning)
    └── odom (published by SLAM Toolbox or AMCL)
        └── base_link (published by EKF)
            ├── imu_link (static)
            ├── laser (static)
            ├── sonar_left_link (static)
            ├── sonar_center_link (static)
            └── sonar_right_link (static)

Usage:
    ros2 run miro_system verify_tf_tree.py
    ros2 run miro_system verify_tf_tree.py --full  # Check map frame (requires SLAM/AMCL running)
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import argparse
import time


class TFTreeVerifier(Node):
    """Node to verify TF tree structure."""

    def __init__(self, check_map_frame=False):
        super().__init__('tf_tree_verifier')

        self.check_map_frame = check_map_frame

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define expected transforms
        self.static_transforms = [
            ('base_link', 'imu_link', 'IMU sensor'),
            ('base_link', 'laser', 'RPLIDAR'),
            ('base_link', 'sonar_left_link', 'Left ultrasonic'),
            ('base_link', 'sonar_center_link', 'Center ultrasonic'),
            ('base_link', 'sonar_right_link', 'Right ultrasonic'),
        ]

        self.dynamic_transforms = [
            ('odom', 'base_link', 'EKF odometry'),
        ]

        self.map_transform = [
            ('map', 'odom', 'SLAM/AMCL localization'),
        ]

        self.get_logger().info('TF Tree Verifier started')
        self.get_logger().info(f'Checking map frame: {check_map_frame}')

        # Wait for TF buffer to fill
        self.get_logger().info('Waiting 2 seconds for TF buffer to fill...')
        time.sleep(2.0)

    def check_transform(self, parent_frame, child_frame, description):
        """Check if a transform exists and return its status."""
        try:
            # Try to lookup the transform
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Check if transform is recent (within last 5 seconds)
            current_time = self.get_clock().now()
            transform_time = rclpy.time.Time.from_msg(transform.header.stamp)
            age = (current_time - transform_time).nanoseconds / 1e9

            if age > 5.0:
                return False, f"Transform is old ({age:.1f}s ago)"

            # Extract translation
            trans = transform.transform.translation
            rot = transform.transform.rotation

            return True, f"[{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}] " \
                        f"[{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]"

        except TransformException as ex:
            return False, str(ex)

    def verify_tf_tree(self):
        """Verify all expected transforms."""
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('TF TREE VERIFICATION REPORT')
        self.get_logger().info('='*70)

        total_checks = 0
        passed_checks = 0

        # Check static transforms
        self.get_logger().info('\nSTATIC TRANSFORMS (base_link -> sensors):')
        for parent, child, desc in self.static_transforms:
            total_checks += 1
            success, info = self.check_transform(parent, child, desc)

            if success:
                self.get_logger().info(f'  ✓ {parent} -> {child} ({desc})')
                self.get_logger().info(f'    Translation: [{info.split("]")[0].split("[")[1]}]')
                passed_checks += 1
            else:
                self.get_logger().warn(f'  ✗ {parent} -> {child} ({desc})')
                self.get_logger().warn(f'    Error: {info}')

        # Check dynamic transforms
        self.get_logger().info('\nDYNAMIC TRANSFORMS (requires EKF running):')
        for parent, child, desc in self.dynamic_transforms:
            total_checks += 1
            success, info = self.check_transform(parent, child, desc)

            if success:
                self.get_logger().info(f'  ✓ {parent} -> {child} ({desc})')
                passed_checks += 1
            else:
                self.get_logger().warn(f'  ✗ {parent} -> {child} ({desc})')
                self.get_logger().warn(f'    Error: {info}')

        # Check map frame (only if requested)
        if self.check_map_frame:
            self.get_logger().info('\nMAP FRAME (requires SLAM/AMCL running):')
            for parent, child, desc in self.map_transform:
                total_checks += 1
                success, info = self.check_transform(parent, child, desc)

                if success:
                    self.get_logger().info(f'  ✓ {parent} -> {child} ({desc})')
                    passed_checks += 1
                else:
                    self.get_logger().warn(f'  ✗ {parent} -> {child} ({desc})')
                    self.get_logger().warn(f'    Error: {info}')

        # Check critical paths
        self.get_logger().info('\nCRITICAL TF PATHS:')

        # Path 1: odom -> laser
        total_checks += 1
        success, info = self.check_transform('odom', 'laser', 'Odometry to LIDAR')
        if success:
            self.get_logger().info(f'  ✓ odom -> laser path verified')
            passed_checks += 1
        else:
            self.get_logger().warn(f'  ✗ odom -> laser path FAILED')
            self.get_logger().warn(f'    Error: {info}')

        # Path 2: odom -> imu_link
        total_checks += 1
        success, info = self.check_transform('odom', 'imu_link', 'Odometry to IMU')
        if success:
            self.get_logger().info(f'  ✓ odom -> imu_link path verified')
            passed_checks += 1
        else:
            self.get_logger().warn(f'  ✗ odom -> imu_link path FAILED')
            self.get_logger().warn(f'    Error: {info}')

        # Path 3: map -> base_link (if checking map)
        if self.check_map_frame:
            total_checks += 1
            success, info = self.check_transform('map', 'base_link', 'Map to robot')
            if success:
                self.get_logger().info(f'  ✓ map -> base_link path verified')
                passed_checks += 1
            else:
                self.get_logger().warn(f'  ✗ map -> base_link path FAILED')
                self.get_logger().warn(f'    Error: {info}')

        # Summary
        self.get_logger().info('\n' + '-'*70)
        if passed_checks == total_checks:
            self.get_logger().info(f'RESULT: ✓ TF tree verification PASSED ({passed_checks}/{total_checks} checks)')
            if not self.check_map_frame:
                self.get_logger().info('Note: Map frame not checked (use --full flag when SLAM/AMCL is running)')
        else:
            self.get_logger().warn(f'RESULT: ⚠ TF tree verification PARTIAL ({passed_checks}/{total_checks} checks passed)')

        # Recommendations
        if passed_checks < total_checks:
            self.get_logger().info('\nRECOMMENDATIONS:')
            if passed_checks < len(self.static_transforms):
                self.get_logger().info('  - Launch static_transforms.launch.py to publish static transforms')
            if any('odom' in t[0] for t in self.dynamic_transforms) and \
               passed_checks < len(self.static_transforms) + len(self.dynamic_transforms):
                self.get_logger().info('  - Launch ekf.launch.py to publish odom -> base_link transform')
            if self.check_map_frame and passed_checks < total_checks:
                self.get_logger().info('  - Launch SLAM Toolbox or AMCL to publish map -> odom transform')

        self.get_logger().info('='*70 + '\n')

        return passed_checks == total_checks


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Verify TF tree structure')
    parser.add_argument('--full', action='store_true',
                        help='Check map frame (requires SLAM/AMCL running)')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        verifier = TFTreeVerifier(check_map_frame=parsed_args.full)
        success = verifier.verify_tf_tree()
        exit_code = 0 if success else 1
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    exit(exit_code)


if __name__ == '__main__':
    main()
