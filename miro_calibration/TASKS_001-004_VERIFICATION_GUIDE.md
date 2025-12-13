# MIRO v2 Calibration System - Tasks 001-004 Verification Guide

**Date**: 2025-12-13
**Tasks Covered**: Speed Preset Manager, Feedback Monitor, Sensor Health Monitor, Data Logger
**Target Platform**: Raspberry Pi 4 with ROS2 Humble

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Build Instructions](#build-instructions)
3. [Task 001: Speed Preset Manager](#task-001-speed-preset-manager)
4. [Task 002: Feedback Monitor](#task-002-feedback-monitor)
5. [Task 003: Sensor Health Monitor](#task-003-sensor-health-monitor)
6. [Task 004: Data Logger](#task-004-data-logger)
7. [Integrated System Test](#integrated-system-test)
8. [Troubleshooting](#troubleshooting)
9. [Performance Verification](#performance-verification)

---

## Prerequisites

### Hardware Requirements
- ✅ Raspberry Pi 4 (4GB RAM recommended)
- ✅ RPLIDAR A1 connected via USB
- ✅ ESP32 with MPU9250 IMU (micro-ROS)
- ✅ Joystick (PS4/Xbox controller or compatible)
- ✅ At least 2GB free disk space

### Software Requirements
- ✅ ROS2 Humble installed
- ✅ Existing sensor drivers running (Tasks 002-005 from autonomous system)
- ✅ Python packages: pytest, numpy, rclpy

### Check Existing System

```bash
# Verify ROS2 installation
ros2 --version
# Expected: ros2 cli version: 0.18.x

# Check available disk space (need >2GB)
df -h ~
# Expected: At least 2GB free

# Verify Python packages
python3 -c "import numpy; import rclpy; print('OK')"
# Expected: OK
```

---

## Build Instructions

### Step 1: Navigate to Workspace

```bash
cd ~/miro_v2
source /opt/ros/humble/setup.bash
```

### Step 2: Build Messages and Services (miro_msgs)

```bash
# Build miro_msgs package first (contains new message types)
colcon build --packages-select miro_msgs --symlink-install

# Source the workspace to make new messages available
source install/setup.bash

# Verify new messages are available
ros2 interface list | grep miro_msgs
```

**Expected Output**:
```
miro_msgs/msg/IMUStats
miro_msgs/msg/RecordingStatus
miro_msgs/msg/RPLIDARStats
miro_msgs/msg/SensorStatus
miro_msgs/msg/SpeedMode
miro_msgs/msg/VelocityFeedback
miro_msgs/msg/VelocityStats
miro_msgs/srv/GetRecordingStatus
miro_msgs/srv/StartRecording
miro_msgs/srv/StopRecording
```

### Step 3: Build Calibration Package (miro_calibration)

```bash
# Build miro_calibration package
colcon build --packages-select miro_calibration --symlink-install

# Source again to include new nodes
source install/setup.bash

# Verify nodes are available
ros2 pkg executables miro_calibration
```

**Expected Output**:
```
miro_calibration data_logger
miro_calibration feedback_monitor
miro_calibration sensor_health_monitor
miro_calibration speed_preset_manager
```

### Step 4: Run Unit Tests

```bash
# Run all calibration tests
pytest src/miro_calibration/test/ -v

# Or run individual test files
pytest src/miro_calibration/test/test_speed_preset_manager.py -v
pytest src/miro_calibration/test/test_feedback_monitor.py -v
pytest src/miro_calibration/test/test_sensor_health_monitor.py -v
pytest src/miro_calibration/test/test_data_logger.py -v
```

**Expected**: All tests should pass (may have warnings, that's OK)

---

## Task 001: Speed Preset Manager

### Purpose
Test joystick button control with three fixed speed presets (SLOW/MEDIUM/FAST).

### Prerequisites for This Test
- Joystick connected via Bluetooth or USB
- joy_node can detect joystick

### Step 1: Check Joystick Connection

```bash
# Check if joystick device exists
ls -l /dev/input/js*
# Expected: /dev/input/js0 (or js1)

# Test joystick input directly
ros2 run joy joy_node &
ros2 topic echo /joy --once
# Press any button on joystick
# Expected: Should see Joy message with button/axes data
```

**If joystick not detected**:
- Check Bluetooth pairing (for wireless)
- Try different USB port (for wired)
- Install joystick utilities: `sudo apt install joystick jstest-gtk`
- Test with: `jstest /dev/input/js0`

### Step 2: Launch Speed Preset Manager

```bash
# Kill the manual joy_node if still running
killall joy_node

# Launch calibration system (includes speed_preset_manager)
ros2 launch miro_calibration calibration.launch.py
```

**Expected Console Output**:
```
[speed_preset_manager]: Speed Preset Manager initialized
[speed_preset_manager]: Default preset: MEDIUM (0.2 m/s, 0.4 rad/s)
[speed_preset_manager]: Button mapping: B0=SLOW, B1=MEDIUM, B2=FAST
```

### Step 3: Monitor Speed Mode Changes

Open a **new terminal**:

```bash
source ~/miro_v2/install/setup.bash
ros2 topic echo /calibration/speed_mode
```

### Step 4: Test Button Presses

Press joystick buttons (PS4 controller mapping):
- **X button (B0)**: Should switch to SLOW
- **Circle button (B1)**: Should switch to MEDIUM
- **Triangle button (B2)**: Should switch to FAST

**Expected Output** (after pressing Circle/B1):
```yaml
mode: 1
linear_speed: 0.2
angular_speed: 0.4
timestamp:
  sec: 1702461845
  nanosec: 123456789
---
```

### Step 5: Verify Debouncing

Rapidly press the same button multiple times (within 300ms).

**Expected**: Only ONE mode change message published, not multiple

### Verification Checklist

- [ ] Joystick connected and detected
- [ ] Speed mode topic publishes on button press
- [ ] SLOW mode (B0): linear=0.1, angular=0.2
- [ ] MEDIUM mode (B1): linear=0.2, angular=0.4
- [ ] FAST mode (B2): linear=0.3, angular=0.6
- [ ] Debouncing works (no double triggers)
- [ ] Default mode is MEDIUM on startup

---

## Task 002: Feedback Monitor

### Purpose
Test real-time velocity feedback display (commanded vs actual).

### Prerequisites for This Test
- EKF node running (publishes `/odometry/filtered`)
- Robot able to receive `/cmd_vel` commands

### Step 1: Verify Prerequisites

```bash
# Check if odometry is publishing
ros2 topic hz /odometry/filtered
# Expected: ~30Hz

# Check if cmd_vel topic exists
ros2 topic info /cmd_vel
# Expected: Topic info with publishers/subscribers
```

### Step 2: Monitor Velocity Feedback

Open a **new terminal**:

```bash
source ~/miro_v2/install/setup.bash
ros2 topic echo /calibration/velocity_feedback
```

### Step 3: Send Test Commands

Open **another terminal**:

```bash
source ~/miro_v2/install/setup.bash

# Send forward velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Wait 2 seconds for robot to respond, then check feedback
```

**Expected Output** (velocity_feedback topic):
```yaml
cmd_linear_x: 0.2
cmd_angular_z: 0.0
actual_linear_x: 0.195  # Close to commanded
actual_angular_z: 0.001  # Close to zero
linear_error_abs: 0.005
linear_error_percent: 2.5
angular_error_abs: 0.001
angular_error_percent: 0.5
status: 0  # GREEN (<5% error)
status_message: 'Velocity tracking: GOOD'
```

### Step 4: Test Error Thresholds

```bash
# Test with robot stationary (should show error)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --once

# If robot can't reach 0.3 m/s, error should increase
# status: 1 (YELLOW) if 5-10% error
# status: 2 (RED) if >10% error
```

### Step 5: Monitor Velocity Stats

```bash
ros2 topic echo /calibration/velocity_stats --once
```

**Expected**: Statistics over 10-second window (mean, max, min errors)

### Verification Checklist

- [ ] Feedback monitor publishes at 10Hz
- [ ] Commanded velocity displayed correctly
- [ ] Actual velocity from odometry displayed
- [ ] Error percentage calculated correctly
- [ ] Status changes based on error thresholds:
  - [ ] GREEN (<5% error)
  - [ ] YELLOW (5-10% error)
  - [ ] RED (>10% error)
- [ ] Velocity stats topic publishes

---

## Task 003: Sensor Health Monitor

### Purpose
Test real-time sensor health monitoring for RPLIDAR and IMU.

### Prerequisites for This Test
- RPLIDAR publishing to `/scan` at ~10Hz
- IMU publishing to `/imu/data` at ~20Hz

### Step 1: Verify Sensor Topics

```bash
# Check RPLIDAR
ros2 topic hz /scan
# Expected: ~10Hz

# Check IMU
ros2 topic hz /imu/data
# Expected: ~20Hz

# Check scan data quality
ros2 topic echo /scan --once | grep "ranges:" -A 5
# Expected: Array of range values
```

### Step 2: Monitor Sensor Status

```bash
source ~/miro_v2/install/setup.bash
ros2 topic echo /calibration/sensor_status
```

**Expected Output** (with healthy sensors):
```yaml
all_sensors_ok: true
warnings: []

# RPLIDAR status
rplidar_connected: true
rplidar_rate_hz: 10.2
rplidar_point_count_avg: 287
rplidar_range_variance: 0.12

# IMU status
imu_connected: true
imu_rate_hz: 19.8
imu_gyro_bias_z: 0.008  # rad/s
imu_accel_magnitude: 9.78  # m/s² (close to 9.81)
```

### Step 3: Monitor Detailed Stats

```bash
# RPLIDAR detailed stats
ros2 topic echo /calibration/rplidar_stats --once

# IMU detailed stats
ros2 topic echo /calibration/imu_stats --once
```

### Step 4: Test RPLIDAR Timeout Detection

```bash
# In one terminal, monitor sensor status
ros2 topic echo /calibration/sensor_status

# In another terminal, stop RPLIDAR
ros2 lifecycle set /rplidar_node shutdown
# OR unplug RPLIDAR USB cable
```

**Expected Output** (after 1-2 seconds):
```yaml
all_sensors_ok: false
warnings:
  - "RPLIDAR: Sensor timeout (1.2s since last message, expected <1.0s)"
rplidar_connected: false
rplidar_rate_hz: 0.0
```

**Recovery**: Restart RPLIDAR
```bash
ros2 launch miro_system rplidar.launch.py
# OR re-plug USB cable
```

### Step 5: Test IMU Timeout Detection

```bash
# Stop micro-ROS agent
killall micro_ros_agent
```

**Expected Output** (after 1-2 seconds):
```yaml
all_sensors_ok: false
warnings:
  - "IMU: Sensor timeout (1.1s since last message, expected <1.0s)"
imu_connected: false
imu_rate_hz: 0.0
```

**Recovery**: Restart micro-ROS agent
```bash
ros2 launch miro_system microros_agent.launch.py
```

### Step 6: Test Warning Generation

```bash
# Obstruct RPLIDAR with your hand (should reduce point count)
# Monitor warnings
ros2 topic echo /calibration/sensor_status | grep "warnings:" -A 3
```

**Expected Warning** (if point count drops):
```
warnings:
  - "RPLIDAR: Low point count (50 points, expected >100 points). Check sensor alignment and obstacles."
```

### Verification Checklist

- [ ] Sensor status publishes at 1Hz
- [ ] RPLIDAR rate detected (~10Hz)
- [ ] IMU rate detected (~20Hz)
- [ ] Point count average calculated
- [ ] Range variance calculated
- [ ] Gyro bias estimated
- [ ] Acceleration magnitude calculated (~9.81 m/s²)
- [ ] RPLIDAR timeout detected (<1 second)
- [ ] IMU timeout detected (<1 second)
- [ ] Warnings generated for low point count
- [ ] Warnings cleared when conditions return to normal
- [ ] all_sensors_ok flag accurate

---

## Task 004: Data Logger

### Purpose
Test automated rosbag2 recording with session management.

### Prerequisites for This Test
- All previous tasks running (sensors, feedback, etc.)
- At least 1GB free disk space

### Step 1: Check Available Disk Space

```bash
df -h ~
# Expected: At least 1GB free in home directory
```

### Step 2: Start Recording

```bash
# Start recording with custom session name
ros2 service call /calibration/start_recording miro_msgs/srv/StartRecording "{session_name: 'test_verification'}"
```

**Expected Response**:
```yaml
success: true
session_id: 'session_20251213_145030'
storage_path: '/home/pi/miro_calibration_data/session_20251213_145030'
message: 'Recording started successfully. Recording 13 topics.'
```

### Step 3: Monitor Recording Status

```bash
# Check recording status topic
ros2 topic echo /calibration/recording_status
```

**Expected Output**:
```yaml
is_recording: true
session_id: 'session_20251213_145030'
start_time:
  sec: 1702461830
  nanosec: 0
topics_recorded:
  - /tf
  - /tf_static
  - /odometry/filtered
  - /cmd_vel
  - /scan
  - /imu/data
  - /calibration/speed_mode
  - /calibration/velocity_feedback
  - /calibration/velocity_stats
  - /calibration/sensor_status
  - /calibration/rplidar_stats
  - /calibration/imu_stats
  - /calibration/recording_status
storage_path: '/home/pi/miro_calibration_data/session_20251213_145030'
file_size_mb: 12.5  # Increases over time
```

### Step 4: Generate Test Data

While recording is active, perform some actions:

```bash
# Change speed mode (press joystick buttons)
# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

# Move robot if possible
# Let it record for 30-60 seconds
```

### Step 5: Stop Recording

```bash
ros2 service call /calibration/stop_recording miro_msgs/srv/StopRecording
```

**Expected Response**:
```yaml
success: true
session_id: 'session_20251213_145030'
duration_sec: 45.3
final_size_mb: 18.7
message: 'Recording stopped successfully.'
```

### Step 6: Verify Recorded Files

```bash
# Navigate to calibration data directory
cd ~/miro_calibration_data

# List sessions
ls -lh
# Expected: session_YYYYMMDD_HHMMSS/ directory

# Check session contents
cd session_*/
ls -la
```

**Expected Directory Structure**:
```
session_20251213_145030/
├── rosbag2_20251213_145030/
│   ├── metadata.yaml
│   └── rosbag2_20251213_145030_0.db3
├── session_metadata.json
└── tests/
```

### Step 7: Verify Session Metadata

```bash
cat session_metadata.json
```

**Expected Content**:
```json
{
  "session_id": "session_20251213_145030",
  "start_timestamp": "2025-12-13T14:50:30.123456",
  "end_timestamp": "2025-12-13T14:51:15.456789",
  "duration_sec": 45.3,
  "topics_recorded": [
    "/tf",
    "/tf_static",
    ...
  ],
  "file_size_mb": 18.7,
  "node_version": "1.0.0",
  "robot_id": "miro_v2_001",
  "operator_name": "default"
}
```

### Step 8: Playback Recorded Data

```bash
# Play back the rosbag
ros2 bag play rosbag2_20251213_145030/

# In another terminal, verify topics are being published
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/data
```

**Expected**: All recorded topics should play back at original rates

### Step 9: Test Storage Cleanup

```bash
# Check current session count
ls ~/miro_calibration_data | wc -l

# Create multiple test sessions to trigger cleanup
for i in {1..25}; do
  ros2 service call /calibration/start_recording miro_msgs/srv/StartRecording "{session_name: 'cleanup_test_$i'}"
  sleep 2
  ros2 service call /calibration/stop_recording miro_msgs/srv/StopRecording
done

# Verify only 20 sessions remain (oldest deleted)
ls ~/miro_calibration_data | wc -l
# Expected: 20 (or fewer)
```

### Step 10: Test Error Handling

```bash
# Test recording while already recording (should fail gracefully)
ros2 service call /calibration/start_recording miro_msgs/srv/StartRecording "{session_name: 'test_double'}"
# Expected: success: false, message: 'Recording already in progress'

# Fill disk space warning (if disk < 1GB)
# Data logger should warn but allow recording
```

### Verification Checklist

- [ ] Start recording service works
- [ ] Stop recording service works
- [ ] Session directories created with timestamp
- [ ] Rosbag files generated (metadata.yaml + .db3)
- [ ] Session metadata JSON created
- [ ] All 13 configured topics recorded
- [ ] Recording status topic publishes at 1Hz
- [ ] File size increases during recording
- [ ] Recorded bags can be played back
- [ ] Storage cleanup works (max 20 sessions)
- [ ] Storage cleanup works (max 10GB)
- [ ] Error handling for double recording
- [ ] Disk space warning (<1GB)

---

## Integrated System Test

### Purpose
Test all four tasks working together in a realistic calibration scenario.

### Full Workflow Test

**Step 1: Launch Everything**
```bash
# Terminal 1: Launch calibration system
ros2 launch miro_calibration calibration.launch.py
```

**Step 2: Start Recording**
```bash
# Terminal 2: Start data logger
ros2 service call /calibration/start_recording miro_msgs/srv/StartRecording "{session_name: 'integrated_test'}"
```

**Step 3: Monitor All Status**
```bash
# Terminal 3: Monitor all calibration topics
ros2 topic echo /calibration/sensor_status &
ros2 topic echo /calibration/velocity_feedback &
ros2 topic echo /calibration/speed_mode &
ros2 topic echo /calibration/recording_status &
```

**Step 4: Perform Test Operations**
- Press joystick buttons to change speed modes (B0, B1, B2)
- Drive robot using joystick (if safe to do so)
- Observe real-time feedback
- Let it run for 2-3 minutes

**Step 5: Verify All Systems**
- Speed mode changes on button press ✓
- Velocity feedback updates at 10Hz ✓
- Sensor health shows all OK ✓
- Recording status shows active recording ✓

**Step 6: Stop and Review**
```bash
# Stop recording
ros2 service call /calibration/stop_recording miro_msgs/srv/StopRecording

# Review session data
cd ~/miro_calibration_data/session_*/
cat session_metadata.json
ros2 bag info rosbag2_*/
```

### Integration Checklist

- [ ] All four nodes launch without errors
- [ ] Speed preset manager responds to joystick
- [ ] Feedback monitor tracks velocity
- [ ] Sensor monitor shows healthy status
- [ ] Data logger records all activity
- [ ] No topic errors or warnings
- [ ] Recorded data plays back correctly
- [ ] System runs for >5 minutes without crashes

---

## Troubleshooting

### Common Issues and Solutions

#### Issue: "Message type not found"

**Symptom**: Error about miro_msgs/msg/SpeedMode or similar

**Solution**:
```bash
# Rebuild miro_msgs
cd ~/miro_v2
colcon build --packages-select miro_msgs --symlink-install
source install/setup.bash

# Verify messages are available
ros2 interface list | grep miro_msgs
```

#### Issue: Joystick not detected

**Symptom**: No /joy topic or joy_node errors

**Solution**:
```bash
# Check device
ls -l /dev/input/js*

# If not found, check USB/Bluetooth
# For wireless: pair in Bluetooth settings
# For wired: try different USB port

# Test joystick directly
jstest /dev/input/js0

# If still issues, reinstall joy
sudo apt install ros-humble-joy
```

#### Issue: RPLIDAR timeout warnings

**Symptom**: Sensor status shows RPLIDAR disconnected

**Solution**:
```bash
# Check USB connection
ls -l /dev/rplidar

# Restart RPLIDAR driver
ros2 launch miro_system rplidar.launch.py

# Check scan topic
ros2 topic hz /scan
# Should be ~10Hz
```

#### Issue: IMU timeout warnings

**Symptom**: Sensor status shows IMU disconnected

**Solution**:
```bash
# Check ESP32 connection
ls -l /dev/ttyACM0

# Restart micro-ROS agent
ros2 launch miro_system microros_agent.launch.py

# Check IMU topic
ros2 topic hz /imu/data
# Should be ~20Hz
```

#### Issue: Recording fails to start

**Symptom**: StartRecording service returns success: false

**Solution**:
```bash
# Check disk space
df -h ~
# Need at least 1GB free

# Check directory permissions
ls -ld ~/miro_calibration_data
# Should be writable by current user

# Create directory manually if needed
mkdir -p ~/miro_calibration_data

# Check if already recording
ros2 service call /calibration/get_recording_status miro_msgs/srv/GetRecordingStatus
# If is_recording: true, stop first
ros2 service call /calibration/stop_recording miro_msgs/srv/StopRecording
```

#### Issue: High CPU usage

**Symptom**: System sluggish, CPU >80%

**Solution**:
```bash
# Check which node is consuming CPU
top
# Press 'P' to sort by CPU

# Reduce RViz if running (resource intensive)
# Reduce topic rates if needed
# Check for topic storms (uncontrolled publishing)
```

#### Issue: Topics not being recorded

**Symptom**: Some topics missing from rosbag playback

**Solution**:
```bash
# Check topic availability before recording
ros2 topic list | grep -E "(scan|imu|odometry)"

# Verify topic types match configuration
ros2 topic info /scan

# Check data_logger.yaml configuration
cat ~/miro_v2/src/miro_calibration/config/data_logger.yaml

# Restart sensors if topics missing
ros2 launch miro_system rplidar.launch.py
ros2 launch miro_system microros_agent.launch.py
ros2 launch miro_system ekf.launch.py
```

---

## Performance Verification

### CPU Usage Targets

| Node | Target CPU | Measurement Command |
|------|-----------|---------------------|
| speed_preset_manager | <3% | `top -p $(pgrep -f speed_preset)` |
| feedback_monitor | <5% | `top -p $(pgrep -f feedback_monitor)` |
| sensor_health_monitor | <5% | `top -p $(pgrep -f sensor_health)` |
| data_logger | <8% | `top -p $(pgrep -f data_logger)` |
| **Total** | **<25%** | `htop` (press F5 for tree view) |

### Memory Usage Targets

| Node | Target Memory | Measurement Command |
|------|--------------|---------------------|
| speed_preset_manager | <50MB | `ps aux \| grep speed_preset` |
| feedback_monitor | <80MB | `ps aux \| grep feedback_monitor` |
| sensor_health_monitor | <100MB | `ps aux \| grep sensor_health` |
| data_logger (active) | <200MB | `ps aux \| grep data_logger` |
| **Total** | **<500MB** | `free -h` |

### Topic Rate Verification

```bash
# Verify all calibration topics are publishing at expected rates
ros2 topic hz /calibration/speed_mode        # Event-based (on button press)
ros2 topic hz /calibration/velocity_feedback # 10 Hz
ros2 topic hz /calibration/velocity_stats    # 1 Hz
ros2 topic hz /calibration/sensor_status     # 1 Hz
ros2 topic hz /calibration/rplidar_stats     # 1 Hz
ros2 topic hz /calibration/imu_stats         # 1 Hz
ros2 topic hz /calibration/recording_status  # 1 Hz
```

### Performance Test Script

Create a monitoring script:

```bash
#!/bin/bash
# performance_check.sh

echo "=== MIRO Calibration Performance Check ==="
echo ""

echo "CPU Usage:"
top -bn1 | grep "miro_calibration" | head -10

echo ""
echo "Memory Usage:"
ps aux | grep "miro_calibration" | grep -v grep

echo ""
echo "Topic Rates:"
timeout 5 ros2 topic hz /calibration/velocity_feedback
timeout 5 ros2 topic hz /calibration/sensor_status

echo ""
echo "Disk Space:"
df -h ~/miro_calibration_data

echo ""
echo "Session Count:"
ls ~/miro_calibration_data 2>/dev/null | wc -l
```

Run with:
```bash
chmod +x performance_check.sh
./performance_check.sh
```

---

## Completion Checklist

### Task 001: Speed Preset Manager
- [ ] Joystick connected and detected
- [ ] Speed mode changes on button press
- [ ] All three presets work (SLOW/MEDIUM/FAST)
- [ ] Debouncing prevents double triggers
- [ ] Default mode is MEDIUM on startup
- [ ] CPU usage <3%

### Task 002: Feedback Monitor
- [ ] Velocity feedback publishes at 10Hz
- [ ] Commanded vs actual velocity displayed
- [ ] Error percentage calculated correctly
- [ ] Status color-coded (GREEN/YELLOW/RED)
- [ ] Velocity stats accumulated
- [ ] CPU usage <5%

### Task 003: Sensor Health Monitor
- [ ] Sensor status publishes at 1Hz
- [ ] RPLIDAR rate detected (~10Hz)
- [ ] IMU rate detected (~20Hz)
- [ ] Timeout detection works (<1 second)
- [ ] Warnings generated and cleared appropriately
- [ ] CPU usage <5%

### Task 004: Data Logger
- [ ] Recording starts/stops via services
- [ ] Session directories created with timestamps
- [ ] All 13 topics recorded
- [ ] Session metadata JSON generated
- [ ] Recorded bags playback correctly
- [ ] Storage cleanup works (20 sessions, 10GB)
- [ ] CPU usage <8% during recording
- [ ] Memory usage <200MB during recording

### Integration
- [ ] All nodes launch together without errors
- [ ] System runs for >5 minutes without crashes
- [ ] Total CPU usage <25%
- [ ] Total memory usage <500MB
- [ ] No topic errors or warnings
- [ ] Recorded data is complete and playable

---

## Success Criteria

✅ **PASS**: All checklist items completed successfully
⚠️ **PARTIAL**: 80%+ checklist items completed (document failures)
❌ **FAIL**: <80% checklist items completed (requires debugging)

---

## Next Steps After Verification

Once Tasks 001-004 are verified:

1. **Document Results**: Note any deviations from expected behavior
2. **Proceed to Task 005**: Safety Supervisor Integration
3. **Integrate with Teleop**: Connect speed presets to actual robot control
4. **Begin Calibration Tests**: Use data logger for actual calibration procedures

---

## Support and Resources

**Completion Reports**:
- `TASK_001_COMPLETION.md` - Speed Preset Manager
- `TASK_002_COMPLETION.md` - Feedback Monitor
- `TASK_003_COMPLETION.md` - Sensor Health Monitor
- `design/calibration-system/tasks/TASK_004_COMPLETION.md` - Data Logger

**Test Files**:
- `test/test_speed_preset_manager.py`
- `test/test_feedback_monitor.py`
- `test/test_sensor_health_monitor.py`
- `test/test_data_logger.py`

**Configuration Files**:
- `config/speed_presets.yaml`
- `config/feedback_monitor.yaml`
- `config/sensor_monitor.yaml`
- `config/data_logger.yaml`

---

**Version**: 1.0
**Last Updated**: 2025-12-13
**Verified On**: ROS2 Humble, Raspberry Pi 4
**Author**: MIRO v2 Development Team
