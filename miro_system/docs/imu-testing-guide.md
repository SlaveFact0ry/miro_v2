# IMU Testing and Verification Guide

## Overview

This document provides step-by-step procedures to test and verify the IMU integration for the autonomous pool cleaning robot.

## Prerequisites

- ESP32 firmware flashed and running
- Raspberry Pi with ROS2 Humble installed
- ESP32 connected to Raspberry Pi via USB
- miro_system package built and sourced

## Test Procedures

### Test 1: Hardware Connection Verification

**Objective**: Verify ESP32 and MPU9250 are properly connected and powered.

**Procedure**:

1. Connect ESP32 to PC (not RPI) via USB
2. Open serial monitor:
   ```bash
   pio device monitor -b 115200
   ```
3. Look for initialization messages:
   ```
   MPU9250 initialization...
   MPU9250 initialized successfully
   I2C address: 0x68
   ```

**Expected Result**: MPU9250 initializes without errors.

**Troubleshooting**:
- If "I2C error": Check MPU9250 wiring (SDA, SCL, VCC, GND)
- If "Address not found": Try I2C address 0x69 (AD0 pin state)
- If random values: Check power supply stability (3.3V)

---

### Test 2: micro-ROS Connection Test

**Objective**: Verify micro-ROS agent can connect to ESP32.

**Procedure**:

1. Connect ESP32 to Raspberry Pi via USB
2. Verify device:
   ```bash
   ls -l /dev/ttyACM0
   ```
3. Launch micro-ROS agent:
   ```bash
   ros2 launch miro_system microros_agent.launch.py
   ```
4. Look for "Session established" in output

**Expected Result**:
```
[micro_ros_agent]: Starting micro-ROS Agent...
[micro_ros_agent]: Session established
```

**Troubleshooting**:
- "Permission denied": Run `sudo usermod -a -G dialout $USER`, logout/login
- "No such file": Check device with `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`
- "Waiting for client...": Reset ESP32 (EN button), check baudrate (115200)

---

### Test 3: Topic Publication Test

**Objective**: Verify /imu/data topic is being published.

**Procedure**:

1. With micro-ROS agent running, open new terminal
2. Source ROS2:
   ```bash
   source ~/miro_ws/install/setup.bash
   ```
3. List topics:
   ```bash
   ros2 topic list
   ```
4. Check /imu/data exists:
   ```bash
   ros2 topic list | grep imu
   ```

**Expected Result**:
```
/imu/data
```

**Troubleshooting**:
- If topic missing: Check agent connection, restart ESP32
- If other topics missing: Check ESP32 firmware initialization

---

### Test 4: Message Frequency Test

**Objective**: Verify IMU publishes at expected rate (~20Hz).

**Procedure**:

1. Check topic frequency:
   ```bash
   ros2 topic hz /imu/data
   ```
2. Let it run for at least 10 seconds
3. Observe average rate

**Expected Result**:
```
average rate: 20.000
    min: 0.045s max: 0.055s std dev: 0.00234s window: 100
```

**Acceptance Criteria**:
- Average rate: 18-22 Hz (±2 Hz tolerance)
- Standard deviation: < 0.01s
- No "no new messages" warnings

**Troubleshooting**:
- If < 18 Hz: Check ESP32 CPU load, reduce logging
- If > 22 Hz: Check timer configuration in firmware (should be 50ms)
- If irregular: Check serial connection quality, try different USB port

---

### Test 5: Message Content Test

**Objective**: Verify IMU data is valid and within expected ranges.

**Procedure**:

1. Echo a single message:
   ```bash
   ros2 topic echo /imu/data --once
   ```
2. Check each field:
   - `frame_id`: Should be "imu_link"
   - `orientation`: Should be [0, 0, 0, 1] (identity quaternion)
   - `angular_velocity`: Should be small values (~0.0 ±0.1 rad/s when stationary)
   - `linear_acceleration.z`: Should be ~9.81 m/s² when upright

**Expected Result**:
```yaml
header:
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
angular_velocity:  # Should be ~0 when stationary
  x: 0.012
  y: -0.003
  z: 0.001
linear_acceleration:  # Z should be ~9.81 when upright
  x: 0.05
  y: 0.02
  z: 9.81
```

**Acceptance Criteria**:
- No NaN values
- Angular velocity < 0.1 rad/s when stationary
- Linear acceleration magnitude ~9.81 m/s² (gravity)
- Frame ID matches "imu_link"

**Troubleshooting**:
- If NaN values: Check I2C connection, MPU9250 initialization
- If wrong frame_id: Update ESP32 firmware
- If accel != 9.81: Normal variation ±0.5 m/s² acceptable

---

### Test 6: Physical Movement Test

**Objective**: Verify IMU responds to physical movement.

**Procedure**:

1. Echo IMU data continuously:
   ```bash
   ros2 topic echo /imu/data
   ```
2. While monitoring output, perform these actions:
   - **Tilt left**: angular_velocity.x should increase
   - **Tilt forward**: angular_velocity.y should increase
   - **Rotate CCW**: angular_velocity.z should increase
   - **Move up/down**: linear_acceleration.z should change

**Expected Result**:
- Angular velocities respond to rotation
- Linear acceleration responds to tilt and movement
- Values return to baseline when stationary

**Acceptance Criteria**:
- Angular velocity changes > 0.5 rad/s during rotation
- Linear acceleration changes > 2 m/s² during tilt
- Data responsive within 100ms (2 messages)

**Troubleshooting**:
- No response: Check MPU9250 mounting, firmware configuration
- Reversed axes: Check MPU9250 orientation on board
- Excessive noise: Enable DLPF in firmware

---

### Test 7: Static Transform Test

**Objective**: Verify TF transform from base_link to imu_link.

**Procedure**:

1. Launch static transforms:
   ```bash
   ros2 launch miro_system static_transforms.launch.py
   ```
2. Echo transform:
   ```bash
   ros2 run tf2_ros tf2_echo base_link imu_link
   ```

**Expected Result**:
```
At time 0.0
- Translation: [0.000, 0.000, 0.050]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

**Acceptance Criteria**:
- Translation.z = 0.05 (5cm above base)
- Rotation = identity (no rotation)
- Transform published continuously

**Troubleshooting**:
- "Transform timeout": Check static_transforms.launch.py is running
- Wrong values: Update static_transforms.launch.py

---

### Test 8: Automated Verification Test

**Objective**: Run comprehensive automated verification.

**Procedure**:

1. Run verification script:
   ```bash
   ros2 run miro_system verify_imu.py --duration 30
   ```
2. Wait for completion (30 seconds)
3. Check final report

**Expected Result**:
```
============================================================
IMU VERIFICATION REPORT
============================================================
Duration: 30.02 seconds
Total messages: 600
Average frequency: 19.99 Hz
Expected frequency: ~20 Hz (ESP32 timer @ 50ms)
Error count: 0
NaN count: 0

------------------------------------------------------------
RESULT: ✓ IMU verification PASSED
IMU is publishing valid data at expected frequency
============================================================
```

**Acceptance Criteria**:
- Total messages: ~600 (20 Hz × 30s)
- Average frequency: 18-22 Hz
- Error count: 0
- NaN count: 0
- Verification PASSED

**Troubleshooting**:
- If frequency out of range: Check USB connection, restart agent
- If errors > 0: Check data quality, I2C connection
- If NaN values: Check MPU9250 initialization, power supply

---

### Test 9: Long-Duration Stability Test

**Objective**: Verify IMU operates reliably over extended period.

**Procedure**:

1. Launch micro-ROS agent
2. Run continuous monitoring:
   ```bash
   ros2 topic hz /imu/data --window 1000
   ```
3. Let run for at least 10 minutes
4. Monitor for:
   - Frequency drops
   - Connection losses
   - Warning messages

**Expected Result**:
- Consistent ~20 Hz throughout test
- No connection losses
- No error messages

**Acceptance Criteria**:
- Frequency variation < 5% over 10 minutes
- Zero disconnections
- Uptime: 100%

**Troubleshooting**:
- If disconnections: Check USB cable quality, power supply
- If frequency drops: Check ESP32 thermal throttling
- If irregular patterns: Check for I2C bus contention

---

### Test 10: Tilt Safety Test

**Objective**: Verify safety system detects excessive tilt.

**Procedure**:

1. Launch verification script with tilt monitoring:
   ```bash
   ros2 run miro_system verify_imu.py --duration 60
   ```
2. During test, tilt robot > 15 degrees
3. Check for warning messages

**Expected Result**:
```
[WARN] [imu_verifier]: WARNING: Excessive tilt detected!
[WARN] [imu_verifier]: Excessive tilt - Roll: 16.23°, Pitch: 2.45°
```

**Acceptance Criteria**:
- Warning triggers when tilt > 15°
- Warning clears when tilt < 15°
- Roll and pitch calculated correctly

**Troubleshooting**:
- No warning: Check quaternion-to-euler conversion in verify_imu.py
- Wrong angles: Verify MPU9250 axis alignment

---

## Test Summary Checklist

Before declaring IMU integration complete, verify:

- [ ] ESP32 connects to Raspberry Pi reliably
- [ ] micro-ROS agent establishes session
- [ ] /imu/data topic publishes at ~20 Hz
- [ ] IMU data is valid (no NaN values)
- [ ] IMU responds to physical movement
- [ ] TF transform base_link → imu_link exists
- [ ] Automated verification script passes
- [ ] 10-minute stability test passes
- [ ] Tilt safety detection works correctly
- [ ] Documentation is complete and accurate

## Regression Testing

Run these tests after:
- ESP32 firmware updates
- ROS2 package updates
- Hardware modifications
- System configuration changes

**Quick regression test** (5 minutes):
```bash
# Terminal 1
ros2 launch miro_system microros_agent.launch.py

# Terminal 2
ros2 run miro_system verify_imu.py --duration 60

# Terminal 3
ros2 topic hz /imu/data
```

## Performance Benchmarks

Record these metrics for future comparison:

| Metric | Expected | Measured | Status |
|--------|----------|----------|--------|
| Frequency | 20 Hz | ______ Hz | ☐ Pass ☐ Fail |
| Latency | < 10 ms | ______ ms | ☐ Pass ☐ Fail |
| Uptime | > 99.9% | ______ % | ☐ Pass ☐ Fail |
| Error rate | < 0.1% | ______ % | ☐ Pass ☐ Fail |
| Agent CPU | < 5% | ______ % | ☐ Pass ☐ Fail |

## Integration Test with EKF

After completing IMU tests, verify integration with EKF (Task 004):

```bash
# Terminal 1: Launch all sensors
ros2 launch miro_system miro.launch.py

# Terminal 2: Check EKF is receiving IMU data
ros2 topic echo /diagnostics | grep imu

# Terminal 3: Verify fused odometry
ros2 topic echo /odometry/filtered --once
```

The fused odometry should incorporate IMU angular velocity and linear acceleration.

## Troubleshooting Common Issues

### Issue: High CPU usage on Raspberry Pi

**Symptoms**: Agent uses > 10% CPU

**Solutions**:
1. Reduce micro-ROS agent verbosity: `verbosity:=3`
2. Disable unnecessary logging in ESP32 firmware
3. Use hardware serial instead of USB-CDC if available

### Issue: IMU drift over time

**Symptoms**: Orientation drifts even when stationary

**Solutions**:
1. Run gyroscope calibration: `mpu.calibrateGyro()` in firmware
2. Enable DLPF: `mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ)`
3. Wait for sensor warm-up (30-60 seconds after power-on)

### Issue: Intermittent NaN values

**Symptoms**: Occasional NaN in IMU data

**Solutions**:
1. Check I2C pull-up resistors (4.7kΩ recommended)
2. Shorten I2C wire length (< 20cm)
3. Add decoupling capacitor (0.1µF) near MPU9250 VCC
4. Check for EMI from motors (add shielding)

## Contact and Support

For issues not covered in this guide:
1. Check ESP32 serial monitor for error messages
2. Check micro-ROS agent log for warnings
3. Review `docs/esp32-microros-setup.md` and `docs/microros-integration.md`
4. Check ROS2 topics and TF tree: `rqt_graph`, `view_frames`

## Conclusion

Successful completion of all tests confirms:
- IMU hardware is functioning correctly
- micro-ROS communication is reliable
- Data quality meets requirements (20Hz, <2° accuracy after fusion)
- System is ready for EKF integration (Task 004)

Next steps:
1. Configure robot_localization EKF (Task 004)
2. Test SLAM with IMU-enhanced odometry (Task 013)
3. Implement safety supervisor with tilt monitoring (Task 073)
