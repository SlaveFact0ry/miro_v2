# IMU Integration Overview

## Architecture Summary

The IMU subsystem for the autonomous pool cleaning robot uses a **micro-ROS architecture** where the IMU sensor is connected to an ESP32 microcontroller, which then communicates with the Raspberry Pi via micro-ROS.

### Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    Physical Hardware                            │
├─────────────────────────────────────────────────────────────────┤
│  MPU9250 IMU ──I2C──> ESP32 DevKit ──USB Serial──> Raspberry Pi│
└─────────────────────────────────────────────────────────────────┘
         │                      │                          │
         │                      │                          │
         v                      v                          v
┌───────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│ Raw Sensor    │    │ micro-ROS Client │    │ micro-ROS Agent     │
│ - Accel: m/s² │───>│ - miro_mcu_node  │───>│ - DDS Bridge        │
│ - Gyro: rad/s │    │ - Publishes:     │    │ - Serial/UDP        │
│ - 20 Hz       │    │   /imu/data      │    └─────────┬───────────┘
└───────────────┘    └──────────────────┘              │
                                                        v
                                               ┌────────────────────┐
                                               │ ROS2 Topics        │
                                               │ - /imu/data        │
                                               └────────┬───────────┘
                                                        │
                     ┌──────────────────────────────────┴──────┐
                     │                                         │
                     v                                         v
            ┌────────────────┐                        ┌────────────────┐
            │ EKF Sensor     │                        │ Safety         │
            │ Fusion         │                        │ Supervisor     │
            │ (Task 004)     │                        │ (Task 073)     │
            └────────────────┘                        └────────────────┘
```

## Why micro-ROS?

### Traditional Approach Problems
- **Direct IMU to RPI**: Requires ROS2 drivers, kernel modules, I2C conflicts
- **Limited I/O**: RPI GPIO pins are limited and shared with other sensors
- **Real-time issues**: Linux on RPI is not real-time, can miss sensor readings

### micro-ROS Benefits
- **Dedicated MCU**: ESP32 handles sensors with hard real-time guarantees
- **Modular**: Sensors isolated on ESP32, easier to debug and replace
- **Flexible**: ESP32 can run multiple sensors (IMU + ultrasonic in our case)
- **Reliable**: ESP32 micro-ROS auto-reconnects on connection loss
- **Scalable**: Easy to add more sensors to ESP32 without affecting RPI

## Components

### 1. ESP32 Firmware (micro-ROS Client)

**File**: `<esp32_project>/src/main.cpp`

**Responsibilities**:
- Initialize MPU9250 IMU sensor via I2C
- Read accelerometer and gyroscope data at 20Hz
- Publish `sensor_msgs/Imu` messages to `/imu/data` topic
- Handle micro-ROS client connection and reconnection

**Key Features**:
- 50ms timer for consistent 20Hz publishing
- Gyroscope calibration on startup
- No orientation fusion (publishes raw data only)

**Setup Guide**: `docs/esp32-microros-setup.md`

### 2. micro-ROS Agent (Raspberry Pi)

**File**: `miro_system/launch/microros_agent.launch.py`

**Responsibilities**:
- Bridge micro-ROS (XRCE-DDS) on ESP32 to ROS2 (DDS) on RPI
- Handle serial USB or UDP WiFi communication
- Auto-reconnect on connection loss

**Launch**:
```bash
ros2 launch miro_system microros_agent.launch.py
```

**Setup Guide**: `docs/microros-integration.md`

### 3. IMU Configuration

**File**: `miro_system/config/imu.yaml`

**Contents**:
- Expected topic parameters (`/imu/data`, `imu_link`, 20Hz)
- Sensor covariance matrices for EKF fusion
- Safety monitoring thresholds (max tilt angles)
- TF transform parameters (base_link → imu_link)

### 4. Static TF Transforms

**File**: `miro_system/launch/static_transforms.launch.py`

**Transforms**:
- `base_link` → `imu_link`: IMU position on robot (0, 0, 0.05m)
- `base_link` → `laser`: RPLIDAR position
- `base_link` → `sonar_*_link`: Ultrasonic sensor positions

### 5. IMU Verification Script

**File**: `miro_system/scripts/verify_imu.py`

**Functionality**:
- Monitor `/imu/data` topic
- Check publishing frequency (~20Hz)
- Verify data validity (no NaN values)
- Detect excessive tilt
- Generate verification report

**Usage**:
```bash
ros2 run miro_system verify_imu.py --duration 30
```

## Quick Start Guide

### Step 1: Flash ESP32 Firmware

```bash
# On development PC (not RPI)
cd <esp32_project>
pio run --target upload
```

Verify firmware is running:
```bash
pio device monitor -b 115200
# Should see: "MPU9250 initialized successfully"
```

### Step 2: Connect ESP32 to Raspberry Pi

1. Connect ESP32 to RPI via USB cable
2. Verify device enumeration:
   ```bash
   ls -l /dev/ttyACM0
   ```
3. Add user to dialout group (first time only):
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login
   ```

### Step 3: Launch micro-ROS Agent

```bash
cd ~/miro_ws
source install/setup.bash
ros2 launch miro_system microros_agent.launch.py
```

Expected output:
```
[micro_ros_agent]: Starting micro-ROS Agent with Serial transport on device: /dev/ttyACM0 @ 115200 baud
[micro_ros_agent]: Session established
```

### Step 4: Verify IMU Data

Open a new terminal:

```bash
# Check topic exists
ros2 topic list | grep imu
# Output: /imu/data

# Check frequency
ros2 topic hz /imu/data
# Output: average rate: 20.000 Hz

# View data
ros2 topic echo /imu/data --once
```

### Step 5: Launch Static Transforms

```bash
ros2 launch miro_system static_transforms.launch.py
```

Verify TF:
```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

### Step 6: Run Verification Script

```bash
ros2 run miro_system verify_imu.py --duration 30
```

Expected result:
```
[imu_verifier]: RESULT: ✓ IMU verification PASSED
```

## Integration with Other Systems

### EKF Sensor Fusion (Task 004)

The robot_localization EKF node subscribes to `/imu/data` and fuses it with wheel odometry:

**EKF Input**:
- `/imu/data`: Angular velocity and linear acceleration
- `/odom`: Wheel odometry (position and velocity)

**EKF Output**:
- `/odometry/filtered`: Fused odometry with IMU correction
- `odom` → `base_link` transform

**Note**: Since ESP32 doesn't provide orientation quaternion, EKF calculates it from angular velocity.

### Safety Supervisor (Task 073)

The safety supervisor monitors `/imu/data` for excessive tilt:

**Monitored Parameters**:
- Roll angle: Warn if > 10°, E-stop if > 15°
- Pitch angle: Warn if > 10°, E-stop if > 15°
- IMU timeout: Fault if no data for > 0.5s (10 missed messages)

**Actions**:
- **Warning**: Log message, reduce speed
- **E-stop**: Publish `/cmd_vel` = 0, trigger emergency stop

### SLAM Toolbox (Task 013)

SLAM uses the EKF-fused odometry (`/odometry/filtered`) which includes IMU data:

**Benefits**:
- **Loop closure**: IMU-enhanced odometry reduces drift
- **Dynamic environments**: IMU detects robot rotation during collisions
- **Water surfaces**: IMU compensates for wheel slip on water

## Data Specifications

### Published Message: /imu/data

**Type**: `sensor_msgs/Imu`

**Frame**: `imu_link`

**Frequency**: ~20 Hz (50ms timer on ESP32)

**Content**:
```yaml
header:
  stamp: {sec: X, nanosec: Y}  # Timestamp from micro-ROS agent
  frame_id: "imu_link"

# Orientation: NOT PROVIDED (identity quaternion)
# EKF will calculate orientation from angular_velocity
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance: [-1, 0, 0, 0, -1, 0, 0, 0, -1]  # Unknown

# Angular velocity: FROM GYROSCOPE
angular_velocity:
  x: 0.012  # rad/s (roll rate)
  y: -0.003  # rad/s (pitch rate)
  z: 0.001  # rad/s (yaw rate)
angular_velocity_covariance: [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]

# Linear acceleration: FROM ACCELEROMETER
linear_acceleration:
  x: 0.05  # m/s² (forward)
  y: 0.02  # m/s² (left)
  z: 9.81  # m/s² (up, includes gravity)
linear_acceleration_covariance: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
```

## Troubleshooting

### Problem: Agent can't connect to ESP32

**Symptoms**: Agent shows "Waiting for client..." indefinitely

**Solutions**:
1. Check USB cable (must be data cable, not charge-only)
2. Verify device: `ls -l /dev/ttyACM0`
3. Check permissions: `groups` (should include `dialout`)
4. Reset ESP32 (press EN button)
5. Check baudrate matches (115200 on both sides)

### Problem: /imu/data not appearing

**Symptoms**: `ros2 topic list` doesn't show `/imu/data`

**Solutions**:
1. Verify agent connection: Look for "Session established" in agent log
2. Check ESP32 serial monitor for errors
3. Verify MPU9250 I2C connection (SDA, SCL, VCC, GND)
4. Restart both agent and ESP32

### Problem: Low frequency or dropped messages

**Symptoms**: `ros2 topic hz` shows < 18 Hz

**Solutions**:
1. Check USB cable quality (avoid USB hubs)
2. Reduce serial logging in ESP32 firmware
3. Verify ESP32 isn't overloaded (reduce other tasks)
4. Check agent CPU usage on RPI

### Problem: IMU data has high noise

**Symptoms**: Accel/gyro values fluctuate excessively

**Solutions**:
1. Enable DLPF in ESP32 firmware: `mpu.setDlpfBandwidth(...)`
2. Run gyroscope calibration: `mpu.calibrateGyro()`
3. Check power supply stability (add decoupling capacitor)
4. Shield I2C wires from motor noise

## Performance Metrics

### Expected Performance

| Metric | Target | Typical |
|--------|--------|---------|
| Publishing frequency | 20 Hz | 19.8-20.2 Hz |
| Message latency (serial) | < 10 ms | ~5 ms |
| Data loss rate | < 0.1% | ~0.01% |
| Agent CPU usage (RPI4) | < 5% | ~2-3% |
| Connection uptime | > 99.9% | ~99.99% |

### Verification Results

Run the verification script to measure actual performance:

```bash
ros2 run miro_system verify_imu.py --duration 60
```

## Future Enhancements

### 1. Add Orientation Fusion on ESP32

Currently, ESP32 publishes **raw IMU data** only (accel + gyro). Orientation is calculated on RPI by EKF.

**Option**: Add Madgwick/Mahony filter to ESP32 firmware to publish fused orientation quaternion.

**Benefits**:
- Reduced RPI CPU load
- Lower latency for orientation data
- Better heading accuracy

**Trade-off**: Increased ESP32 complexity

### 2. Enable Magnetometer for Heading

MPU9250 includes AK8963 magnetometer, but current firmware doesn't use it.

**Benefits**:
- Absolute heading reference (no yaw drift)
- Better localization in large pools

**Requirements**:
- Magnetometer calibration (figure-8 pattern)
- Handle magnetic disturbances (motors, metal)

### 3. Add WiFi Transport

Current setup uses USB serial. Can switch to WiFi UDP for wireless operation.

**Benefits**:
- No USB cable between ESP32 and RPI
- Easier robot maintenance

**Trade-offs**:
- Higher latency (~50ms vs ~5ms)
- Network dependency
- Lower reliability

## Related Documentation

- **ESP32 Setup**: `docs/esp32-microros-setup.md`
- **RPI Integration**: `docs/microros-integration.md`
- **EKF Configuration**: `config/ekf.yaml` (Task 004)
- **Safety Parameters**: `config/imu.yaml` (Safety section)

## References

- [micro-ROS Documentation](https://micro.ros.org/)
- [MPU9250 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/)
- [sensor_msgs/Imu Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)
- [REP-103: Standard Units of Measure](https://www.ros.org/reps/rep-0103.html)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
