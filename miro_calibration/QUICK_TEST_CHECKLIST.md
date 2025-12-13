# MIRO v2 Calibration - Quick Test Checklist

**Purpose**: Fast verification of Tasks 001-004 on Raspberry Pi
**Time Required**: ~15 minutes

---

## Pre-Test Setup (5 minutes)

```bash
# 1. Navigate to workspace
cd ~/miro_v2
source /opt/ros/humble/setup.bash

# 2. Build packages
colcon build --packages-select miro_msgs miro_calibration --symlink-install
source install/setup.bash

# 3. Verify build
ros2 pkg executables miro_calibration
# Expected: 4 nodes listed

# 4. Check prerequisites
ros2 topic list | grep -E "(scan|imu|odometry)"
# Expected: /scan, /imu/data, /odometry/filtered

# 5. Check joystick
ls /dev/input/js*
# Expected: /dev/input/js0
```

**Status**: ☐ Setup Complete

---

## Task 001: Speed Preset Manager (2 minutes)

```bash
# Launch system
ros2 launch miro_calibration calibration.launch.py

# In new terminal, monitor speed mode
ros2 topic echo /calibration/speed_mode
```

**Test Actions**:
1. Press X button (B0) → ☐ SLOW mode (0.1 m/s)
2. Press Circle button (B1) → ☐ MEDIUM mode (0.2 m/s)
3. Press Triangle button (B2) → ☐ FAST mode (0.3 m/s)
4. Rapid press same button → ☐ Only one change (debouncing works)

**Status**: ☐ PASS / ☐ FAIL

---

## Task 002: Feedback Monitor (2 minutes)

```bash
# Monitor velocity feedback
ros2 topic echo /calibration/velocity_feedback

# Send test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
```

**Verify**:
- ☐ cmd_linear_x: 0.2
- ☐ actual_linear_x: ~0.2 (close)
- ☐ linear_error_percent: <10%
- ☐ status: 0 (GREEN) or 1 (YELLOW)
- ☐ Topic rate: ~10Hz (`ros2 topic hz /calibration/velocity_feedback`)

**Status**: ☐ PASS / ☐ FAIL

---

## Task 003: Sensor Health Monitor (3 minutes)

```bash
# Monitor sensor status
ros2 topic echo /calibration/sensor_status
```

**Verify Healthy State**:
- ☐ all_sensors_ok: true
- ☐ rplidar_connected: true
- ☐ rplidar_rate_hz: ~10.0
- ☐ imu_connected: true
- ☐ imu_rate_hz: ~20.0
- ☐ warnings: [] (empty)

**Test Timeout Detection**:
```bash
# Unplug RPLIDAR USB
# Wait 2 seconds, check sensor_status
```
- ☐ rplidar_connected: false (within 1 second)
- ☐ Warning message generated

```bash
# Re-plug RPLIDAR
# Wait 2 seconds, check sensor_status
```
- ☐ rplidar_connected: true (recovery works)

**Status**: ☐ PASS / ☐ FAIL

---

## Task 004: Data Logger (5 minutes)

### Test 1: Basic Recording

```bash
# Start recording
ros2 service call /calibration/start_recording miro_msgs/srv/StartRecording "{session_name: 'quick_test'}"
```

**Verify Response**:
- ☐ success: true
- ☐ session_id: session_YYYYMMDD_HHMMSS
- ☐ storage_path: /home/pi/miro_calibration_data/...

```bash
# Monitor status
ros2 topic echo /calibration/recording_status --once
```

**Verify Status**:
- ☐ is_recording: true
- ☐ 13 topics in topics_recorded list
- ☐ file_size_mb > 0 (increasing)

```bash
# Wait 30 seconds, then stop
ros2 service call /calibration/stop_recording miro_msgs/srv/StopRecording
```

**Verify Response**:
- ☐ success: true
- ☐ duration_sec: ~30
- ☐ final_size_mb > 0

### Test 2: Verify Files

```bash
cd ~/miro_calibration_data/session_*/
ls -la
```

**Verify Directory**:
- ☐ rosbag2_*/ directory exists
- ☐ session_metadata.json exists
- ☐ tests/ directory exists

```bash
cat session_metadata.json
```

**Verify Metadata**:
- ☐ session_id matches
- ☐ 13 topics listed
- ☐ duration_sec ~30
- ☐ file_size_mb > 0

### Test 3: Playback

```bash
ros2 bag play rosbag2_*/
# In another terminal:
ros2 topic hz /scan
ros2 topic hz /imu/data
```

**Verify Playback**:
- ☐ /scan plays at ~10Hz
- ☐ /imu/data plays at ~20Hz
- ☐ No playback errors

**Status**: ☐ PASS / ☐ FAIL

---

## Performance Check (2 minutes)

```bash
# Check CPU usage
top -bn1 | grep "calibration" | head -5

# Check memory
free -h

# Check topic rates
ros2 topic hz /calibration/velocity_feedback  # ~10Hz
ros2 topic hz /calibration/sensor_status      # ~1Hz
```

**Verify**:
- ☐ Total CPU usage <25%
- ☐ Free memory >500MB
- ☐ All topic rates correct
- ☐ No error messages in logs

**Status**: ☐ PASS / ☐ FAIL

---

## Integration Test (3 minutes)

**Scenario**: Record a complete calibration session

```bash
# 1. Start recording
ros2 service call /calibration/start_recording miro_msgs/srv/StartRecording "{session_name: 'integration_test'}"

# 2. Perform actions (all at once):
# - Press joystick buttons (change speed modes)
# - Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# - Move robot with joystick (if safe)
# - Let run for 60 seconds

# 3. Monitor all topics simultaneously
ros2 topic echo /calibration/speed_mode &
ros2 topic echo /calibration/velocity_feedback &
ros2 topic echo /calibration/sensor_status &
ros2 topic echo /calibration/recording_status &

# Wait 60 seconds...

# 4. Stop recording
ros2 service call /calibration/stop_recording miro_msgs/srv/StopRecording
```

**Verify**:
- ☐ All 4 nodes running without errors
- ☐ Speed changes recorded
- ☐ Velocity feedback recorded
- ☐ Sensor status recorded
- ☐ No warnings or errors during test
- ☐ Recording completed successfully
- ☐ Playback shows all activities

**Status**: ☐ PASS / ☐ FAIL

---

## Final Results

| Task | Status | Notes |
|------|--------|-------|
| 001: Speed Preset Manager | ☐ PASS / ☐ FAIL | |
| 002: Feedback Monitor | ☐ PASS / ☐ FAIL | |
| 003: Sensor Health Monitor | ☐ PASS / ☐ FAIL | |
| 004: Data Logger | ☐ PASS / ☐ FAIL | |
| Performance | ☐ PASS / ☐ FAIL | |
| Integration | ☐ PASS / ☐ FAIL | |

**Overall**: ☐ PASS (all pass) / ☐ PARTIAL (4-5 pass) / ☐ FAIL (<4 pass)

---

## Quick Troubleshooting

| Issue | Quick Fix |
|-------|-----------|
| Message not found | `colcon build --packages-select miro_msgs && source install/setup.bash` |
| Joystick not detected | Check `ls /dev/input/js*`, re-pair Bluetooth |
| RPLIDAR timeout | `ros2 launch miro_system rplidar.launch.py` |
| IMU timeout | `ros2 launch miro_system microros_agent.launch.py` |
| Recording fails | Check `df -h ~` (need >1GB free) |
| High CPU | Check `top`, reduce RViz if running |

---

## Next Steps

- ☐ Document any failures in detailed verification guide
- ☐ Proceed to Task 005 (Safety Supervisor)
- ☐ OR fix issues and re-test
- ☐ Update completion reports with hardware test results

---

**Test Date**: _____________
**Tester**: _____________
**Platform**: Raspberry Pi 4 / ROS2 Humble
**Overall Result**: ☐ PASS / ☐ PARTIAL / ☐ FAIL
