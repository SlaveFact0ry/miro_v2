# micro-ROS Integration Guide (Raspberry Pi)

## Overview

This document describes how to set up and verify the micro-ROS agent on Raspberry Pi to communicate with the ESP32 micro-ROS node. The agent acts as a bridge between the ESP32's micro-ROS DDS client and the ROS2 network.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Raspberry Pi                            │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    ROS2 Network (DDS)                     │  │
│  │  ┌────────────┐  ┌─────────────┐  ┌─────────────────┐   │  │
│  │  │ EKF Fusion │  │ Navigation  │  │ Safety Monitor  │   │  │
│  │  └─────┬──────┘  └──────┬──────┘  └────────┬────────┘   │  │
│  │        │                 │                  │            │  │
│  │        └─────────────────┴──────────────────┘            │  │
│  │                          │                               │  │
│  │                   ┌──────▼──────┐                        │  │
│  │                   │ /imu/data   │                        │  │
│  │                   │ /cmd_vel    │                        │  │
│  │                   └──────▲──────┘                        │  │
│  └──────────────────────────┼─────────────────────────────┘  │
│           ┌─────────────────▼──────────────────┐              │
│           │    micro-ROS Agent (DDS Bridge)    │              │
│           └─────────────────▲──────────────────┘              │
└─────────────────────────────┼─────────────────────────────────┘
                              │ USB Serial (/dev/ttyACM0)
                              │ or WiFi UDP
┌─────────────────────────────┼─────────────────────────────────┐
│                         ESP32 DevKit                           │
│           ┌─────────────────▼──────────────────┐              │
│           │   micro-ROS Client (XRCE-DDS)      │              │
│           │         miro_mcu_node              │              │
│           └─────────────────┬──────────────────┘              │
│                    ┌────────┴────────┐                        │
│                    │                 │                        │
│              ┌─────▼─────┐    ┌─────▼──────┐                 │
│              │ MPU9250   │    │ HC-SR04 x3 │                 │
│              │    IMU    │    │  Ultrasonic│                 │
│              └───────────┘    └────────────┘                 │
└────────────────────────────────────────────────────────────────┘
```

## Installing micro-ROS Agent

### Prerequisites

- ROS2 Humble installed on Raspberry Pi
- Ubuntu 22.04 or Raspberry Pi OS (64-bit recommended)

### Installation

The micro-ROS agent is available as a pre-built ROS2 package:

```bash
# Update package index
sudo apt update

# Install micro-ROS agent
sudo apt install ros-humble-micro-ros-agent

# Source ROS2 environment
source /opt/ros/humble/setup.bash
```

### Verify Installation

```bash
# Check if agent executable exists
which micro_ros_agent

# Expected output: /opt/ros/humble/lib/micro_ros_agent/micro_ros_agent
```

## Setting Up Serial Connection

### Configure Serial Permissions

The user running ROS2 needs permission to access serial devices:

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Verify group membership
groups

# Logout and login for changes to take effect
# OR run: newgrp dialout (temporary for current session)
```

### Identify ESP32 Serial Device

When ESP32 is connected via USB, it typically appears as `/dev/ttyACM0`:

```bash
# List all serial devices
ls -l /dev/ttyACM* /dev/ttyUSB*

# Expected output:
# crw-rw---- 1 root dialout ... /dev/ttyACM0

# Check device info
udevadm info -a -n /dev/ttyACM0 | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}'
```

**Common ESP32 USB-to-Serial chips**:
- CP2102: Appears as `/dev/ttyUSB0`
- CH340: Appears as `/dev/ttyUSB0`
- Native USB CDC: Appears as `/dev/ttyACM0` (most ESP32 DevKit boards)

## Launching micro-ROS Agent

### Using Launch File (Recommended)

The `microros_agent.launch.py` file provides a convenient way to start the agent:

```bash
# Serial connection (default)
ros2 launch miro_system microros_agent.launch.py

# Serial with custom device
ros2 launch miro_system microros_agent.launch.py device:=/dev/ttyACM1

# UDP WiFi connection
ros2 launch miro_system microros_agent.launch.py transport:=udp4 port:=8888

# Serial with verbose logging
ros2 launch miro_system microros_agent.launch.py verbosity:=6
```

### Manual Command (Alternative)

You can also run the agent directly:

```bash
# Serial connection
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# UDP connection
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# With verbose output
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6
```

### Expected Output

When the agent connects successfully, you should see:

```
[micro_ros_agent]: Starting micro-ROS Agent with Serial transport on device: /dev/ttyACM0 @ 115200 baud
[1234567890.123456789] [micro_ros_agent]: Starting Micro XRCE-DDS Agent
[1234567890.234567890] [micro_ros_agent]: OK
[1234567890.345678901] [micro_ros_agent]: Running Micro XRCE-DDS Agent...
[1234567890.456789012] [micro_ros_agent]: Micro XRCE-DDS Agent running
```

When ESP32 connects, you'll see:

```
[1234567891.123456789] [micro_ros_agent]: Session established
```

## Verifying IMU Data Reception

### Check Topic Availability

```bash
# List all topics
ros2 topic list

# Expected to see:
# /imu/data
# /ultrasonic/left
# /ultrasonic/center
# /ultrasonic/right
# /cmd_vel (if subscribed)
```

### Check Topic Information

```bash
# Get topic details
ros2 topic info /imu/data

# Expected output:
# Type: sensor_msgs/msg/Imu
# Publisher count: 1
# Subscription count: 0 (or more if other nodes are subscribing)
```

### Monitor Message Frequency

```bash
# Check publishing rate
ros2 topic hz /imu/data

# Expected output:
# average rate: 20.000
#   min: 0.045s max: 0.055s std dev: 0.00234s window: 20
```

The expected frequency is ~20Hz (50ms timer in ESP32 firmware).

### View Message Content

```bash
# Echo a single message
ros2 topic echo /imu/data --once

# Expected output:
# header:
#   stamp:
#     sec: 1234567890
#     nanosec: 123456789
#   frame_id: imu_link
# orientation:
#   x: 0.0
#   y: 0.0
#   z: 0.0
#   w: 1.0
# angular_velocity:
#   x: 0.012  # rad/s
#   y: -0.003
#   z: 0.001
# linear_acceleration:
#   x: 0.05  # m/s²
#   y: 0.02
#   z: 9.81
```

### Run Verification Script

Use the dedicated verification script to test data quality:

```bash
# Run for 30 seconds
ros2 run miro_system verify_imu.py --duration 30

# Expected output:
# [imu_verifier]: IMU Verifier started. Listening to /imu/data...
# [imu_verifier]: Messages: 200 | Frequency: 20.05 Hz | Errors: 0 | NaN: 0
# ...
# [imu_verifier]: RESULT: ✓ IMU verification PASSED
```

## Verifying TF Transforms

The IMU data should be integrated into the robot's TF tree:

```bash
# Launch static transforms
ros2 launch miro_system static_transforms.launch.py

# View TF tree
ros2 run tf2_tools view_frames

# This generates frames.pdf showing the TF tree
# Expected: base_link -> imu_link transform exists

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link imu_link

# Expected output:
# At time 0.0
# - Translation: [0.000, 0.000, 0.050]
# - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

## Testing Latency and Performance

### Measure End-to-End Latency

Create a simple test to measure latency from ESP32 to ROS2:

```bash
# Monitor timestamp differences
ros2 topic echo /imu/data --field header.stamp | head -n 20

# Compare ESP32 timestamp with ROS2 receive time
# Latency should be < 10ms for serial, < 50ms for WiFi
```

### Monitor System Load

```bash
# Check micro-ROS agent CPU usage
top -p $(pgrep micro_ros_agent)

# Expected: ~2-5% CPU on Raspberry Pi 4
```

### Stress Test

Run all sensors simultaneously:

```bash
# Terminal 1: Launch micro-ROS agent
ros2 launch miro_system microros_agent.launch.py

# Terminal 2: Monitor all topics
ros2 topic hz /imu/data /ultrasonic/left /ultrasonic/center /ultrasonic/right

# Terminal 3: Verify data quality
ros2 run miro_system verify_imu.py --duration 60
```

## Troubleshooting

### Agent fails to start: "Permission denied"

**Cause**: User not in `dialout` group

**Solution**:
```bash
sudo usermod -a -G dialout $USER
# Logout and login, or run: newgrp dialout
```

### Agent starts but ESP32 doesn't connect

**Symptoms**: Agent shows "Waiting for client..." indefinitely

**Possible causes and solutions**:

1. **Wrong baudrate**:
   - Verify ESP32 firmware uses 115200
   - Check agent launch arguments: `-b 115200`

2. **Wrong device**:
   ```bash
   # List all devices
   ls -l /dev/ttyACM* /dev/ttyUSB*

   # Try each device
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
   ```

3. **ESP32 not running micro-ROS firmware**:
   - Reflash ESP32 with correct firmware
   - Check ESP32 serial monitor for error messages

4. **Serial port already in use**:
   ```bash
   # Check if another process is using the port
   lsof | grep ttyACM0

   # Kill the process if needed
   sudo killall -9 screen  # or minicom, or other serial program
   ```

### Topic /imu/data doesn't appear

**Symptoms**: `ros2 topic list` doesn't show `/imu/data`

**Solutions**:

1. **Verify ESP32 connection**:
   - Agent should show "Session established"
   - If not, check ESP32 serial monitor for errors

2. **Check topic creation on ESP32**:
   - Ensure `rclc_publisher_init_default()` succeeded
   - Verify no memory allocation errors in ESP32

3. **Restart both agent and ESP32**:
   ```bash
   # Kill agent
   pkill micro_ros_agent

   # Reset ESP32 (press EN button or power cycle)

   # Restart agent
   ros2 launch miro_system microros_agent.launch.py
   ```

### IMU data has NaN values

**Symptoms**: `verify_imu.py` reports NaN values

**Solutions**:

1. **Check I2C connection**:
   - Verify wiring (SDA, SCL, VCC, GND)
   - Check I2C address (0x68 vs 0x69)

2. **IMU initialization failed**:
   - Check ESP32 serial monitor for MPU9250 init errors
   - Verify `mpu.begin()` returns 0 (success)

3. **Power supply issue**:
   - Ensure stable 3.3V supply
   - Add decoupling capacitor near IMU

### Low message frequency

**Symptoms**: `ros2 topic hz` shows < 18 Hz

**Solutions**:

1. **Check ESP32 timer**:
   - Verify `RCL_MS_TO_NS(50)` in firmware (20Hz)
   - Ensure no blocking delays in timer callback

2. **Serial buffer overflow**:
   - Reduce logging/printing in ESP32 firmware
   - Increase baudrate (e.g., 230400 or 460800)

3. **Agent dropping messages**:
   - Check agent logs for warnings
   - Reduce verbosity level: `-v 3`

### Connection drops intermittently

**Symptoms**: Agent shows repeated "Session closed" / "Session established"

**Solutions**:

1. **USB cable quality**:
   - Use high-quality, short USB cable
   - Avoid USB hubs if possible

2. **Power supply**:
   - Ensure stable power to ESP32
   - Avoid powering ESP32 through RPI's USB (use external power)

3. **Enable respawn in launch file**:
   - Launch file already includes `respawn=True`
   - Agent will automatically restart on failure

## Auto-start on Boot

To automatically launch micro-ROS agent when Raspberry Pi boots:

### Using systemd Service

1. **Create service file**:
   ```bash
   sudo nano /etc/systemd/system/microros-agent.service
   ```

2. **Add service configuration**:
   ```ini
   [Unit]
   Description=micro-ROS Agent for MIRO Robot
   After=network.target

   [Service]
   Type=simple
   User=ubuntu  # Change to your username
   WorkingDirectory=/home/ubuntu
   ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 launch miro_system microros_agent.launch.py'
   Restart=always
   RestartSec=5

   [Install]
   WantedBy=multi-user.target
   ```

3. **Enable and start service**:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable microros-agent.service
   sudo systemctl start microros-agent.service

   # Check status
   sudo systemctl status microros-agent.service
   ```

## Integration with System Launch

To include micro-ROS agent in the main system launch:

Edit `/Users/jung-yechan/.claude/miro_v2/miro_system/launch/miro.launch.py`:

```python
# Add micro-ROS agent to launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

microros_agent = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('miro_system'),
            'launch',
            'microros_agent.launch.py'
        ])
    ])
)

return LaunchDescription([
    # ... other nodes ...
    microros_agent,
])
```

## Performance Benchmarks

Expected performance on Raspberry Pi 4 (4GB):

| Metric | Expected Value |
|--------|---------------|
| IMU frequency | 20 Hz ± 1 Hz |
| Message latency (serial) | < 10 ms |
| Message latency (WiFi) | < 50 ms |
| Agent CPU usage | 2-5% |
| Agent memory usage | ~20 MB |
| Connection reliability | > 99.9% uptime |

## Next Steps

After verifying micro-ROS integration:
1. Configure robot_localization EKF to fuse IMU with odometry (Task 004)
2. Set up SLAM with IMU-enhanced odometry (Task 013)
3. Integrate IMU tilt monitoring in safety supervisor (Task 073)

## References

- [micro-ROS Agent Documentation](https://micro.ros.org/docs/tutorials/core/first_application_linux/)
- [XRCE-DDS Protocol](https://www.omg.org/spec/DDS-XRCE/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
