#!/bin/bash
# Install ROS2 joy and teleop_twist_joy packages
# Run this on the Raspberry Pi

set -e

echo "=== Installing ROS2 Joy Packages ==="
echo ""

# Check if running as root or with sudo
if [ "$EUID" -ne 0 ]; then
  echo "This script needs sudo privileges to install packages."
  echo "Please run with: sudo bash INSTALL_JOY_PACKAGES.sh"
  exit 1
fi

# Update package list
echo "Updating package list..."
apt update

# Install joy package (joystick driver)
echo ""
echo "Installing ros-humble-joy..."
apt install -y ros-humble-joy

# Install teleop_twist_joy package (joy to cmd_vel converter)
echo ""
echo "Installing ros-humble-teleop-twist-joy..."
apt install -y ros-humble-teleop-twist-joy

# Install joystick utilities for testing
echo ""
echo "Installing joystick testing utilities..."
apt install -y joystick jstest-gtk

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Verify installation:"
echo "  1. Check joystick device: ls -l /dev/input/js*"
echo "  2. Test joystick: jstest /dev/input/js0"
echo "  3. Run joy node: ros2 run joy joy_node"
echo ""
echo "Now you can run:"
echo "  ros2 launch miro_control teleop.launch.py"
echo "  ros2 launch miro_calibration calibration.launch.py"
