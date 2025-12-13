# Task 003: Sensor Health Monitor - Verification Guide

## Quick Verification Checklist

Use this checklist when deploying to Raspberry Pi to verify the sensor health monitor implementation.

## 1. Build and Compile

```bash
cd ~/miro_v2
colcon build --packages-select miro_msgs miro_calibration --symlink-install
```

**Expected Output**:
- ✅ Both packages build successfully
- ✅ No compilation errors
- ✅ Three new message types generated:
  - miro_msgs/msg/SensorStatus
  - miro_msgs/msg/RPLIDARStats
  - miro_msgs/msg/IMUStats

**Verify**:
```bash
source install/setup.bash
ros2 interface show miro_msgs/msg/SensorStatus
ros2 interface show miro_msgs/msg/RPLIDARStats
ros2 interface show miro_msgs/msg/IMUStats
```

## 2. Unit Tests

```bash
cd ~/miro_v2
source install/setup.bash
pytest src/miro_calibration/test/test_sensor_health_monitor.py -v
```

**Expected Output**:
- ✅ All frequency calculation tests pass
- ✅ All variance calculation tests pass
- ✅ All timeout detection tests pass
- ✅ All warning generation tests pass
- ✅ All gyro bias estimation tests pass
- ✅ All acceleration magnitude tests pass

**Expected**: 25+ tests passed, 0 failures

## 3. Integration Tests

```bash
cd ~/miro_v2
source install/setup.bash
pytest src/miro_calibration/test/test_sensor_health_monitor_integration.py -v
```

**Expected Output**:
- ✅ Node initialization test passes
- ✅ RPLIDAR monitoring test passes
- ✅ IMU monitoring test passes
- ✅ Timeout detection test passes
- ✅ Warning generation tests pass
- ✅ Healthy sensor test passes

**Expected**: 8 tests passed, 0 failures

## 4. Standalone Node Test

### 4.1 Launch Node Standalone

```bash
ros2 run miro_calibration sensor_health_monitor
```

**Expected Output**:
```
[INFO] [sensor_health_monitor]: Sensor Health Monitor initialized: RPLIDAR=10.0Hz, IMU=20.0Hz, status_rate=1.0Hz
[WARN] [sensor_health_monitor]: RPLIDAR: No data received yet
[WARN] [sensor_health_monitor]: IMU: No data received yet
```

**Verify Topics**:
```bash
# In another terminal
ros2 topic list | grep calibration
```

**Expected**:
```
/calibration/sensor_status
/calibration/rplidar_stats
/calibration/imu_stats
```

### 4.2 Check Topic Rates

```bash
ros2 topic hz /calibration/sensor_status
ros2 topic hz /calibration/rplidar_stats
ros2 topic hz /calibration/imu_stats
```

**Expected**: All topics publishing at ~1.0 Hz

### 4.3 Verify Message Content (No Sensors)

```bash
ros2 topic echo /calibration/sensor_status --once
```

**Expected**:
```yaml
header: ...
rplidar_connected: false
rplidar_rate_hz: 0.0
rplidar_point_count_avg: 0.0
rplidar_range_variance: 0.0
imu_connected: false
imu_rate_hz: 0.0
imu_gyro_bias_z: 0.0
imu_accel_magnitude: 0.0
warnings:
- 'RPLIDAR: No data received yet'
- 'IMU: No data received yet'
```

## 5. Full Calibration System Test

### 5.1 Launch Calibration System

```bash
ros2 launch miro_calibration calibration.launch.py
```

**Expected Output**:
```
[INFO] [joy_node]: Joystick device opened: /dev/input/js0
[INFO] [speed_preset_manager]: Speed Preset Manager initialized with default: MEDIUM (0.2 m/s, 0.4 rad/s)
[INFO] [sensor_health_monitor]: Sensor Health Monitor initialized: RPLIDAR=10.0Hz, IMU=20.0Hz, status_rate=1.0Hz
```

**Verify All Nodes Running**:
```bash
ros2 node list
```

**Expected**:
```
/joy_node
/speed_preset_manager
/sensor_health_monitor
```

## 6. RPLIDAR Integration Test

### 6.1 Start RPLIDAR Driver

```bash
# In another terminal
ros2 launch rplidar_ros rplidar.launch.py
```

### 6.2 Verify RPLIDAR Detection

Wait 5-10 seconds for frequency calculation window to fill, then:

```bash
ros2 topic echo /calibration/rplidar_stats --once
```

**Expected**:
```yaml
connected: true
rate_hz: 9.5-10.5  # Should be ~10 Hz ±20%
point_count: 100-400
point_count_avg: 100-400
rate_error_percent: <20.0
healthy: true
```

### 6.3 Check for Warnings

```bash
ros2 topic echo /calibration/sensor_status --once
```

**Expected**:
- `rplidar_connected: true`
- No RPLIDAR warnings in warnings array (if environment is good)

## 7. IMU Integration Test

### 7.1 Start IMU Driver

```bash
# Assuming IMU driver is running
# Check /imu/data topic is available
ros2 topic list | grep imu
```

### 7.2 Verify IMU Detection

Wait 5-10 seconds for frequency calculation window to fill, then:

```bash
ros2 topic echo /calibration/imu_stats --once
```

**Expected**:
```yaml
connected: true
rate_hz: 16.0-24.0  # Should be ~20 Hz ±20%
gyro_bias_z: <0.02  # Should be small when stationary
accel_magnitude: 9.3-10.3  # Should be ~9.81 m/s²
rate_error_percent: <20.0
healthy: true
```

### 7.3 Check for Warnings

```bash
ros2 topic echo /calibration/sensor_status --once
```

**Expected**:
- `imu_connected: true`
- No IMU warnings in warnings array (if IMU is healthy)

## 8. Fault Injection Tests

### 8.1 Test RPLIDAR Timeout

1. Start calibration system with RPLIDAR running
2. Stop RPLIDAR driver: `Ctrl+C` on rplidar launch terminal
3. Wait 2 seconds
4. Check status:

```bash
ros2 topic echo /calibration/sensor_status --once
```

**Expected**:
- `rplidar_connected: false`
- Warning: "RPLIDAR: Sensor timeout (X.Xs since last message, expected <1.0s)"

### 8.2 Test IMU Timeout

1. Start calibration system with IMU running
2. Stop IMU driver
3. Wait 2 seconds
4. Check status:

```bash
ros2 topic echo /calibration/sensor_status --once
```

**Expected**:
- `imu_connected: false`
- Warning: "IMU: Sensor timeout (X.Xs since last message, expected <1.0s)"

### 8.3 Test Low Point Count

1. Cover RPLIDAR sensor partially with hand/object
2. Wait 5 seconds
3. Check status:

```bash
ros2 topic echo /calibration/sensor_status --once
```

**Expected** (if point count drops below 100):
- Warning: "RPLIDAR: Low point count (XX points, expected >100 points). Check sensor alignment and obstacles."

## 9. Performance Verification

### 9.1 Check CPU Usage

```bash
top -p $(pgrep -f sensor_health_monitor)
```

**Expected**:
- CPU usage <5% on Raspberry Pi 4
- Memory usage <50 MB

### 9.2 Check Message Latency

```bash
ros2 topic delay /calibration/sensor_status
```

**Expected**:
- Latency <100ms

## 10. Configuration Parameter Test

### 10.1 Test Custom Thresholds

Edit `/config/sensor_monitor.yaml`:
```yaml
sensor_health_monitor:
  ros__parameters:
    min_points: 50  # Lower threshold for testing
    sensor_timeout_sec: 2.0  # Longer timeout
```

Rebuild and relaunch:
```bash
colcon build --packages-select miro_calibration --symlink-install
ros2 launch miro_calibration calibration.launch.py
```

**Expected**:
- Warnings appear with new thresholds
- Timeout detection uses 2.0 seconds

### 10.2 Verify Parameter Loading

```bash
ros2 param list /sensor_health_monitor
```

**Expected Parameters**:
```
rplidar_expected_hz
imu_expected_hz
rate_tolerance_percent
sensor_timeout_sec
min_points
max_range_variance
max_gyro_bias
max_accel_noise
status_publish_rate_hz
frequency_window_sec
```

```bash
ros2 param get /sensor_health_monitor min_points
```

**Expected**: `Integer value is: 50`

## 11. Warning Message Verification

Check that warnings are actionable and contain required information:

### Expected Warning Formats:

**RPLIDAR Timeout**:
```
RPLIDAR: Sensor timeout (1.2s since last message, expected <1.0s)
```

**RPLIDAR Low Point Count**:
```
RPLIDAR: Low point count (75 points, expected >100 points). Check sensor alignment and obstacles.
```

**RPLIDAR High Variance**:
```
RPLIDAR: High range variance (0.723, expected <0.5). Environment may be too cluttered.
```

**RPLIDAR Rate**:
```
RPLIDAR: Scan rate out of range (7.5Hz, expected 10.0Hz ±20%)
```

**IMU Timeout**:
```
IMU: Sensor timeout (1.1s since last message, expected <1.0s)
```

**IMU High Gyro Bias**:
```
IMU: High gyro bias (0.0350 rad/s, expected <0.02 rad/s). IMU may need recalibration.
```

**IMU Abnormal Acceleration**:
```
IMU: Abnormal acceleration magnitude (12.34 m/s², expected ~9.81 m/s² ±0.5 m/s²). Check IMU mounting and calibration.
```

**IMU Rate**:
```
IMU: Data rate out of range (15.2Hz, expected 20.0Hz ±20%)
```

## 12. Final Integration Checklist

- [ ] All packages build successfully
- [ ] Unit tests pass (25+ tests)
- [ ] Integration tests pass (8 tests)
- [ ] Node launches without errors
- [ ] Topics publish at correct rates (1 Hz)
- [ ] RPLIDAR monitoring works with live data
- [ ] IMU monitoring works with live data
- [ ] Timeout detection works (<1 second)
- [ ] Warnings are generated correctly
- [ ] Warning messages are actionable
- [ ] CPU usage <5%
- [ ] Configuration parameters load correctly
- [ ] Node integrates with calibration.launch.py
- [ ] Both sensors report healthy when working
- [ ] Sensor disconnection detected within 1 second

## Common Issues and Solutions

### Issue: "No module named 'numpy'"
**Solution**: Install numpy on Raspberry Pi
```bash
pip3 install numpy
```

### Issue: RPLIDAR rate showing 0.0 Hz
**Solution**: Wait 5-10 seconds for sliding window to fill with samples

### Issue: High CPU usage
**Solution**: Check that numpy is installed for vectorized operations

### Issue: Topics not publishing
**Solution**: Verify node is running and source setup.bash was called

### Issue: Always showing "No data received yet"
**Solution**:
1. Check sensor drivers are running
2. Verify topic names match (/scan, /imu/data)
3. Check QoS compatibility

## Success Criteria

✅ All tests pass
✅ CPU usage <5% on RPi4
✅ Status published at 1 Hz
✅ RPLIDAR monitoring accurate
✅ IMU monitoring accurate
✅ Timeout detection <1 second
✅ Warnings actionable and clear
✅ No memory leaks (run for 10+ minutes)

---

**Date**: December 13, 2024
**Task**: 003-sensor-health-monitor-node
**Status**: Ready for Raspberry Pi Verification
