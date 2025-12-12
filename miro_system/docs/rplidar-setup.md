# RPLIDAR Setup and Configuration Guide

## Overview

This document provides complete setup instructions for the RPLIDAR sensor integration on the Miro autonomous pool cleaning robot. The RPLIDAR provides 360-degree laser scan data at 10Hz for SLAM mapping and autonomous navigation.

## Hardware Requirements

- **RPLIDAR Model**: A1, A2, or A3
- **Connection**: USB port on Raspberry Pi
- **Power**: Powered via USB (no external power needed)
- **Mounting**: Forward-facing, 10-degree downward pitch

## Software Requirements

- ROS 2 Humble
- rplidar_ros package
- Ubuntu 22.04 (on Raspberry Pi)

---

## Installation Steps

### 1. Install RPLIDAR ROS 2 Package

```bash
# Install from apt repository (recommended)
sudo apt update
sudo apt install ros-humble-rplidar-ros

# OR build from source if needed
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git -b ros2
cd ~/ros2_ws
colcon build --packages-select rplidar_ros
```

### 2. Connect RPLIDAR Hardware

1. Connect RPLIDAR USB cable to Raspberry Pi USB port
2. Power on the RPLIDAR (should see laser spinning)
3. Verify device detection:

```bash
# Check if device appears
ls -l /dev/ttyUSB*

# Expected output: /dev/ttyUSB0 or similar
```

### 3. Install Udev Rules for Consistent Device Naming

The udev rule creates a persistent `/dev/rplidar` symlink and sets proper permissions.

```bash
# Copy udev rule from package
sudo cp ~/ros2_ws/src/miro_v2/miro_system/config/udev/99-rplidar.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify symlink created
ls -l /dev/rplidar

# Expected output: /dev/rplidar -> ttyUSB0
```

### 4. Add User to dialout Group (for serial port access)

```bash
# Add current user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect
# OR reboot the system
```

### 5. Build miro_system Package

```bash
cd ~/ros2_ws
colcon build --packages-select miro_system
source install/setup.bash
```

---

## Launch and Testing

### Launch RPLIDAR Standalone

```bash
# Source ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch RPLIDAR node
ros2 launch miro_system rplidar.launch.py
```

**Expected output:**
```
[rplidar_node-1] RPLIDAR S/N: XXXXXXXXXXXX
[rplidar_node-1] Firmware Ver: X.XX
[rplidar_node-1] Hardware Rev: X
[rplidar_node-1] RPLidar health status: OK
[rplidar_node-1] RPLIDAR is running in Standard scan mode
```

### Verify Scan Data Publishing

Open a new terminal and check the `/scan` topic:

```bash
# Check topic is publishing
ros2 topic list | grep scan

# Check scan frequency (should be ~10Hz)
ros2 topic hz /scan

# View scan data
ros2 topic echo /scan
```

### Visualize in RViz

```bash
# Launch RViz
rviz2

# In RViz:
# 1. Set Fixed Frame to "laser_frame"
# 2. Click "Add" -> "By topic" -> "/scan" -> "LaserScan"
# 3. You should see 360-degree scan data visualization
```

### Verify Scan Parameters

```bash
# Check scan message details
ros2 topic echo /scan --once

# Verify these parameters:
# - angle_min: ~0.0 rad
# - angle_max: ~6.28 rad (360 degrees)
# - range_min: 0.2 m
# - range_max: 6.0 m
# - scan_time: ~0.1 s (10Hz)
```

### Measure Scan Accuracy

Test accuracy by placing objects at known distances:

1. Place a flat surface (wall, board) at known distance (e.g., 1.0m, 2.0m, 4.0m)
2. Record RPLIDAR measurements using RViz or topic echo
3. Compare measured vs actual distance
4. Verify error is less than 2% (e.g., for 2.0m distance, error < 4cm)

```bash
# Example: Check specific scan angles
ros2 topic echo /scan/ranges --once
```

---

## Configuration Parameters

### Launch File Arguments

You can override default parameters when launching:

```bash
# Custom serial port
ros2 launch miro_system rplidar.launch.py serial_port:=/dev/ttyUSB0

# Custom frame ID
ros2 launch miro_system rplidar.launch.py frame_id:=custom_laser_frame

# Use Sensitivity scan mode (more detail, slower)
ros2 launch miro_system rplidar.launch.py scan_mode:=Sensitivity
```

### RPLIDAR Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/rplidar` | Serial device path |
| `serial_baudrate` | `115200` | Serial communication baud rate |
| `frame_id` | `laser_frame` | TF frame for scan data |
| `scan_mode` | `Standard` | Scan mode (Standard/Sensitivity) |
| `scan_frequency` | `10.0` | Target scan frequency in Hz |
| `angle_min` | `0.0` | Minimum scan angle (radians) |
| `angle_max` | `6.28318531` | Maximum scan angle (2π radians) |
| `range_min` | `0.2` | Minimum valid range (meters) |
| `range_max` | `6.0` | Maximum valid range (meters) |
| `inverted` | `False` | Mount orientation inverted |
| `angle_compensate` | `True` | Compensate for motor speed variations |
| `auto_reconnect` | `True` | Auto-reconnect on connection loss |

---

## Integration with Main System

The RPLIDAR is integrated into the main system launch file (`miro.launch.py`). To launch the complete system including RPLIDAR:

```bash
ros2 launch miro_system miro.launch.py
```

This will start:
- RPLIDAR driver
- TF transforms (base_link → laser_frame)
- RF2O laser odometry
- EKF sensor fusion
- Other system components

---

## Troubleshooting

### Issue: Device Not Found `/dev/rplidar`

**Symptoms:**
```
[ERROR] Failed to open serial port /dev/rplidar
```

**Solutions:**

1. **Check USB connection:**
   ```bash
   ls -l /dev/ttyUSB*
   # If no devices, check physical USB connection
   ```

2. **Check udev rules installed:**
   ```bash
   ls -l /etc/udev/rules.d/99-rplidar.rules
   # If not found, reinstall udev rules (see Installation Step 3)
   ```

3. **Reload udev rules:**
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Check device vendor/product ID:**
   ```bash
   lsusb
   # Look for CP2102 (10c4:ea60) or RPLIDAR (0483:5740)
   ```

5. **Temporarily use direct device:**
   ```bash
   ros2 launch miro_system rplidar.launch.py serial_port:=/dev/ttyUSB0
   ```

---

### Issue: Permission Denied

**Symptoms:**
```
[ERROR] Failed to open /dev/rplidar: Permission denied
```

**Solutions:**

1. **Add user to dialout group:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

2. **Temporary fix (testing only):**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

3. **Verify group membership:**
   ```bash
   groups
   # Should include 'dialout'
   ```

---

### Issue: No Scan Data Published

**Symptoms:**
- Node starts but `/scan` topic has no data
- RViz shows empty laser scan

**Solutions:**

1. **Check RPLIDAR motor spinning:**
   - Listen for motor sound
   - Look for visible laser spinning

2. **Check node output for errors:**
   ```bash
   ros2 launch miro_system rplidar.launch.py
   # Look for health status errors
   ```

3. **Test with rplidar standalone:**
   ```bash
   ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/rplidar
   ```

4. **Check RPLIDAR health:**
   - Unplug and replug USB
   - Power cycle RPLIDAR
   - Check for physical obstructions

---

### Issue: Low Scan Frequency

**Symptoms:**
```bash
ros2 topic hz /scan
# Shows < 8Hz (expected ~10Hz)
```

**Solutions:**

1. **Check CPU usage:**
   ```bash
   top
   # High CPU usage can slow down scanning
   ```

2. **Use Standard scan mode (not Sensitivity):**
   ```bash
   ros2 launch miro_system rplidar.launch.py scan_mode:=Standard
   ```

3. **Check for USB power issues:**
   - Use powered USB hub if needed
   - Check USB cable quality

---

### Issue: Inaccurate Range Measurements

**Symptoms:**
- Range measurements don't match known distances
- Error > 2% at 0-6m range

**Solutions:**

1. **Test on non-reflective surfaces:**
   - Water surfaces cause reflections
   - Use matte/non-reflective test surfaces first

2. **Enable angle compensation:**
   ```bash
   # Should be enabled by default
   # Verify in launch file: angle_compensate: True
   ```

3. **Check for physical obstructions:**
   - Clean RPLIDAR lens
   - Check for dust or water droplets

4. **Verify mounting stability:**
   - Ensure RPLIDAR is firmly mounted
   - Check for vibrations

---

### Issue: Auto-Reconnect Not Working

**Symptoms:**
- RPLIDAR disconnects and doesn't reconnect
- Node crashes on USB disconnect

**Solutions:**

1. **Verify auto_reconnect enabled:**
   ```bash
   # Check launch file parameter: auto_reconnect: True
   ```

2. **Check system logs:**
   ```bash
   dmesg | grep ttyUSB
   # Look for USB disconnection messages
   ```

3. **Use systemd auto-restart (for production):**
   ```ini
   # In systemd service file
   Restart=always
   RestartSec=5
   ```

---

### Issue: TF Frame Errors

**Symptoms:**
```
[WARN] Could not transform from laser_frame to base_link
```

**Solutions:**

1. **Check TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   # Open frames.pdf to see TF tree
   ```

2. **Verify static transform published:**
   ```bash
   ros2 topic echo /tf_static
   # Should show base_link → laser_frame
   ```

3. **Check frame_id parameter:**
   ```bash
   ros2 param get /rplidar_node frame_id
   # Should match TF transform (default: laser_frame)
   ```

---

## Testing in Pool Environment

### Water Surface Considerations

Water surfaces can cause laser reflections and false readings:

1. **Test at different water conditions:**
   - Calm water (best conditions)
   - Slight ripples
   - Wavy conditions

2. **Adjust mounting angle if needed:**
   - Current: 10° downward pitch
   - May need adjustment to avoid water reflections

3. **Set appropriate range_min:**
   - Current: 0.2m
   - Filters out very close reflections

### Verification Checklist

Before deploying in pool:

- [ ] RPLIDAR publishes /scan at 10Hz consistently
- [ ] 360-degree coverage verified in RViz
- [ ] Range accuracy < 2% error at 1m, 2m, 4m test distances
- [ ] No TF transform errors
- [ ] Auto-reconnect works after USB unplug/replug
- [ ] Node starts on system boot (if configured)
- [ ] Test with pool wall at various distances
- [ ] Test obstacle detection (pool toys, floats, etc.)
- [ ] Verify behavior with water surface reflections

---

## Performance Specifications

### Expected Performance

| Metric | Target | Verification Method |
|--------|--------|---------------------|
| Scan frequency | ≥10Hz | `ros2 topic hz /scan` |
| Scan range | 0.2m - 6.0m | Topic echo, physical measurement |
| Angular resolution | ~1° | Check angle_increment in scan message |
| Accuracy | <2% error | Compare known vs measured distances |
| Coverage | 360° | RViz visualization |
| Latency | <100ms | Check scan_time in message |

### System Integration

- **Topics published:** `/scan` (sensor_msgs/LaserScan)
- **TF frames:** `base_link` → `laser_frame`
- **Update rate:** 10Hz minimum
- **Integration points:**
  - RF2O laser odometry (consumes /scan)
  - SLAM mapping (slam_toolbox)
  - Navigation (Nav2 costmap)

---

## Additional Resources

- [RPLIDAR ROS 2 Documentation](https://github.com/Slamtec/rplidar_ros)
- [sensor_msgs/LaserScan Message Definition](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/LaserScan.html)
- [RPLIDAR SDK Manual](https://www.slamtec.com/en/Support)
- [TF2 Tutorials](http://wiki.ros.org/tf2/Tutorials)

---

## Quick Reference Commands

```bash
# Launch RPLIDAR
ros2 launch miro_system rplidar.launch.py

# Check scan topic
ros2 topic hz /scan

# View scan data
ros2 topic echo /scan

# Check TF frames
ros2 run tf2_tools view_frames

# Check parameters
ros2 param list /rplidar_node

# Visualize in RViz
rviz2

# Check device
ls -l /dev/rplidar

# Check permissions
groups | grep dialout
```

---

**Document Version:** 1.0
**Last Updated:** 2025-12-12
**Author:** Miro Development Team
