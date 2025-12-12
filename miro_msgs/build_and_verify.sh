#!/bin/bash

# Build and verification script for miro_msgs package
# Run this script on the Raspberry Pi after sourcing ROS2 environment

set -e

echo "========================================="
echo "Building miro_msgs package..."
echo "========================================="

# Build the package
colcon build --packages-select miro_msgs

echo ""
echo "========================================="
echo "Sourcing workspace..."
echo "========================================="

# Source the workspace
source install/setup.bash

echo ""
echo "========================================="
echo "Verifying message definitions..."
echo "========================================="
echo ""

# List all miro_msgs interfaces
echo "Available miro_msgs interfaces:"
ros2 interface list | grep miro_msgs
echo ""

# Show each message definition
echo "========================================="
echo "MissionState.msg:"
echo "========================================="
ros2 interface show miro_msgs/msg/MissionState
echo ""

echo "========================================="
echo "SafetyStatus.msg:"
echo "========================================="
ros2 interface show miro_msgs/msg/SafetyStatus
echo ""

echo "========================================="
echo "CoverageStatus.msg:"
echo "========================================="
ros2 interface show miro_msgs/msg/CoverageStatus
echo ""

echo "========================================="
echo "ValidationReport.msg:"
echo "========================================="
ros2 interface show miro_msgs/msg/ValidationReport
echo ""

echo "========================================="
echo "PathProgress.msg:"
echo "========================================="
ros2 interface show miro_msgs/msg/PathProgress
echo ""

echo "========================================="
echo "Verification complete!"
echo "========================================="
echo "All message definitions are accessible."
