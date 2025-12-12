# Miro System Package

This package contains the core system launch files and configuration for the Miro autonomous pool cleaning robot.

## Contents

### Launch Files

- **miro.launch.py**: Main system launch file that starts all components (RPLIDAR, IMU, odometry, localization)
- **rplidar.launch.py**: Standalone RPLIDAR sensor launch file

### Configuration

- **config/ekf.yaml**: Extended Kalman Filter configuration for sensor fusion
- **config/udev/99-rplidar.rules**: Udev rules for RPLIDAR device naming

### Documentation

- **docs/rplidar-setup.md**: Complete RPLIDAR setup, configuration, and troubleshooting guide

## Quick Start

### Launch Full System

```bash
ros2 launch miro_system miro.launch.py
```

### Launch RPLIDAR Only

```bash
ros2 launch miro_system rplidar.launch.py
```

## Setup Instructions

For detailed RPLIDAR setup instructions, see [docs/rplidar-setup.md](docs/rplidar-setup.md).

### Quick RPLIDAR Setup

1. Install rplidar_ros:
   ```bash
   sudo apt install ros-humble-rplidar-ros
   ```

2. Install udev rules:
   ```bash
   sudo cp config/udev/99-rplidar.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

3. Add user to dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```

4. Build and launch:
   ```bash
   colcon build --packages-select miro_system
   source install/setup.bash
   ros2 launch miro_system rplidar.launch.py
   ```

## Dependencies

- ros-humble-rplidar-ros
- ros-humble-tf2-ros
- ros-humble-robot-localization
- ros-humble-rf2o-laser-odometry

## Hardware Requirements

- Raspberry Pi 4 (or compatible)
- RPLIDAR A1/A2/A3
- USB connection for RPLIDAR
- IMU sensor (for full system)
- ESP32 with micro-ROS (for full system)

## Topics Published

- `/scan` - LaserScan data from RPLIDAR (10Hz)
- `/odom` - Fused odometry from EKF
- `/odom_rf2o` - Laser odometry from RF2O
- `/tf` and `/tf_static` - Transform tree

## Frames

- `base_link` - Robot base frame
- `laser_frame` - RPLIDAR sensor frame (0.15m forward, 0.1m up, 10° pitch)
- `imu_link` - IMU sensor frame
- `odom` - Odometry frame

## Support

For issues and troubleshooting, see the detailed documentation in the `docs/` directory.
