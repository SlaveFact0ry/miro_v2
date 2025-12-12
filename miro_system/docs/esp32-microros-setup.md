# ESP32 micro-ROS Setup Guide

## Overview

This document describes the ESP32 micro-ROS firmware setup for the autonomous pool cleaning robot. The ESP32 DevKit runs micro-ROS and interfaces with:
- **MPU9250 IMU** (I2C) - Publishes orientation and motion data
- **3x HC-SR04 Ultrasonic Sensors** - Publishes range data
- **Motor Controller** (future) - Subscribes to velocity commands

## Hardware Configuration

### Components
- **MCU**: ESP32 DevKit (30-pin)
- **IMU**: MPU9250 9-axis IMU (I2C address: 0x68)
- **Ultrasonic Sensors**: 3x HC-SR04 (Left, Center, Right)
- **Connection**: USB Serial to Raspberry Pi (appears as /dev/ttyACM0)

### Pin Connections

#### MPU9250 IMU (I2C)
```
MPU9250 VCC  -> ESP32 3.3V
MPU9250 GND  -> ESP32 GND
MPU9250 SCL  -> ESP32 GPIO 22 (Default I2C SCL)
MPU9250 SDA  -> ESP32 GPIO 21 (Default I2C SDA)
```

#### HC-SR04 Ultrasonic Sensors
```
Left Sensor:
  TRIG -> GPIO 5
  ECHO -> GPIO 18

Center Sensor:
  TRIG -> GPIO 19
  ECHO -> GPIO 23

Right Sensor:
  TRIG -> GPIO 32
  ECHO -> GPIO 33
```

## Software Architecture

### micro-ROS Node Structure

**Node Name**: `miro_mcu_node`

**Published Topics**:
- `/imu/data` (sensor_msgs/Imu) - IMU data at 20Hz
- `/ultrasonic/left` (sensor_msgs/Range) - Left sonar at ~6.6Hz
- `/ultrasonic/center` (sensor_msgs/Range) - Center sonar at ~6.6Hz
- `/ultrasonic/right` (sensor_msgs/Range) - Right sonar at ~6.6Hz

**Subscribed Topics**:
- `/cmd_vel` (geometry_msgs/Twist) - Motor velocity commands

### Firmware Timing

The ESP32 firmware uses a 50ms timer (20Hz) for the main loop:
- **Every cycle**: Read and publish IMU data
- **Round-robin**: Publish one ultrasonic sensor per cycle (3 cycles = 150ms for all sensors)

This approach minimizes delays caused by ultrasonic sensor measurements (30ms timeout each).

## IMU Data Publishing

### Message Format

The ESP32 publishes `sensor_msgs/Imu` messages with the following structure:

```cpp
sensor_msgs__msg__Imu imu_msg;

// Header
imu_msg.header.frame_id.data = "imu_link";

// Linear acceleration (m/s²)
imu_msg.linear_acceleration.x = mpu.getAccelX_mss();
imu_msg.linear_acceleration.y = mpu.getAccelY_mss();
imu_msg.linear_acceleration.z = mpu.getAccelZ_mss();

// Angular velocity (rad/s)
imu_msg.angular_velocity.x = mpu.getGyroX_rads();
imu_msg.angular_velocity.y = mpu.getGyroY_rads();
imu_msg.angular_velocity.z = mpu.getGyroZ_rads();

// Orientation - Set to identity quaternion if not calculated
imu_msg.orientation.x = 0.0;
imu_msg.orientation.y = 0.0;
imu_msg.orientation.z = 0.0;
imu_msg.orientation.w = 1.0;
```

### Current Firmware Limitations

The current firmware publishes **raw IMU data** without sensor fusion:
- ✓ Linear acceleration (from accelerometer)
- ✓ Angular velocity (from gyroscope)
- ✗ Orientation quaternion (set to identity, not calculated)

**Solution**: Orientation fusion is performed on the Raspberry Pi side using `imu_complementary_filter` or robot_localization EKF.

## PlatformIO Project Setup

### platformio.ini Configuration

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
    bolderflight/Bolder Flight Systems MPU9250@^1.0.1

build_flags =
    -DMICRO_ROS_TRANSPORT_ARDUINO_SERIAL
    -std=gnu++17
```

### Required Libraries

1. **micro_ros_platformio**: Provides micro-ROS integration for PlatformIO
2. **Bolder Flight Systems MPU9250**: IMU driver library

Install via PlatformIO Library Manager or add to `lib_deps`.

## Flashing Instructions

### Prerequisites

- PlatformIO Core or PlatformIO IDE (VSCode extension)
- ESP32 USB drivers installed
- USB cable (data capable, not charge-only)

### Steps

1. **Connect ESP32 to PC via USB**

2. **Check serial port**:
   ```bash
   # Linux/macOS
   ls /dev/tty*

   # Windows
   # Check Device Manager -> Ports (COM & LPT)
   ```

3. **Upload firmware**:
   ```bash
   cd <project_directory>
   pio run --target upload
   ```

4. **Monitor serial output** (optional):
   ```bash
   pio device monitor -b 115200
   ```

## IMU Calibration

### Gyroscope Calibration

The gyroscope needs to be calibrated to remove bias (drift when stationary).

**Procedure**:
1. Place robot on a flat, stable surface
2. Keep robot completely still
3. Run calibration in firmware:
   ```cpp
   mpu.begin();
   mpu.calibrateGyro();  // Takes ~5 seconds
   ```
4. Calibration values are stored internally in the MPU9250 library

### Accelerometer Calibration

The accelerometer should be calibrated for accurate tilt measurement.

**Procedure** (6-point calibration):
1. Place robot in each of 6 orientations (±X, ±Y, ±Z up)
2. Record accelerometer values in each position
3. Calculate bias and scale factors
4. Apply corrections in firmware

**Note**: For pool cleaning robot, basic factory calibration is usually sufficient since we primarily use IMU for tilt detection.

### Magnetometer Calibration

The MPU9250 includes a magnetometer, but it requires calibration for accurate heading.

**Procedure** (Figure-8 calibration):
1. Move robot in a figure-8 pattern
2. Rotate through all orientations
3. Record min/max values for each axis
4. Calculate hard-iron (offset) and soft-iron (scale) corrections

**Note**: Current firmware does not use magnetometer. If heading accuracy is required, implement magnetometer reading and calibration.

## Connecting ESP32 to Raspberry Pi

### USB Serial Connection (Recommended)

1. **Connect ESP32 to Raspberry Pi via USB cable**

2. **Verify device enumeration**:
   ```bash
   ls -l /dev/ttyACM*
   # Expected: /dev/ttyACM0 (ESP32 usually appears as ACM, not USB)
   ```

3. **Add user to dialout group** (for serial access):
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login for changes to take effect
   ```

4. **Launch micro-ROS agent on Raspberry Pi**:
   ```bash
   ros2 launch miro_system microros_agent.launch.py device:=/dev/ttyACM0
   ```

### WiFi UDP Connection (Alternative)

If you prefer wireless connection:

1. **Configure WiFi in ESP32 firmware**:
   ```cpp
   #include <WiFi.h>

   const char* ssid = "YourSSID";
   const char* password = "YourPassword";

   WiFi.begin(ssid, password);
   set_microros_wifi_transports("192.168.1.100", 8888);  // RPI IP
   ```

2. **Launch micro-ROS agent on Raspberry Pi**:
   ```bash
   ros2 launch miro_system microros_agent.launch.py transport:=udp4 port:=8888
   ```

**Trade-offs**:
- Serial: More reliable, no network dependency, lower latency
- WiFi: Cable-free, but adds latency and potential disconnections

## Verifying IMU Communication

After launching the micro-ROS agent, verify IMU data:

1. **Check if /imu/data topic exists**:
   ```bash
   ros2 topic list | grep imu
   # Expected output: /imu/data
   ```

2. **Check message frequency**:
   ```bash
   ros2 topic hz /imu/data
   # Expected: average rate: ~20.000 Hz
   ```

3. **View IMU data**:
   ```bash
   ros2 topic echo /imu/data --once
   ```

4. **Run verification script**:
   ```bash
   ros2 run miro_system verify_imu.py --duration 30
   ```

## Troubleshooting

### ESP32 not detected

**Symptoms**: `/dev/ttyACM0` doesn't appear after connecting ESP32

**Solutions**:
- Check USB cable (must be data cable, not charge-only)
- Install CH340/CP2102 USB-to-Serial drivers (depending on ESP32 variant)
- Try different USB port
- Check `dmesg | tail` for connection errors

### micro-ROS agent connection failed

**Symptoms**: Agent starts but shows "Waiting for micro-ROS agent..."

**Solutions**:
- Verify baudrate matches (115200 on both sides)
- Check serial permissions (`groups` command should show `dialout`)
- Ensure no other program is using the serial port
- Reset ESP32 (press EN button)

### IMU data not publishing

**Symptoms**: Topic `/imu/data` doesn't appear

**Solutions**:
- Check I2C connections (SDA, SCL, VCC, GND)
- Verify MPU9250 I2C address (0x68 or 0x69)
- Add pull-up resistors to I2C lines (4.7kΩ) if using long wires
- Check ESP32 serial monitor for MPU9250 initialization errors

### IMU data has high noise

**Symptoms**: Accelerometer/gyro values fluctuate excessively

**Solutions**:
- Enable DLPF (Digital Low-Pass Filter) in firmware:
  ```cpp
  mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  ```
- Perform gyroscope calibration
- Check power supply (noise on 3.3V rail affects IMU)
- Add decoupling capacitor (0.1µF) near IMU VCC pin

### Low message frequency

**Symptoms**: `/imu/data` publishing slower than 20Hz

**Solutions**:
- Check ESP32 CPU load (reduce logging if heavy)
- Increase timer frequency in firmware
- Verify micro-ROS agent isn't dropping messages (check agent log)

## Firmware Customization

### Adjusting IMU Publishing Rate

Current rate: 20Hz (50ms timer). To change:

```cpp
// In setup()
rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
                                                         ^^
                                                 Change this value

// Examples:
// 50 Hz -> RCL_MS_TO_NS(20)
// 100 Hz -> RCL_MS_TO_NS(10)
```

**Note**: Higher rates increase CPU load and may impact ultrasonic sensor processing.

### Adding Orientation Fusion on ESP32

To calculate orientation quaternion on ESP32 (instead of Raspberry Pi):

1. **Add Madgwick or Mahony filter library**:
   ```ini
   lib_deps =
       ...
       https://github.com/arduino-libraries/MadgwickAHRS
   ```

2. **Implement sensor fusion**:
   ```cpp
   #include <MadgwickAHRS.h>

   Madgwick filter;

   void timer_callback() {
       mpu.readSensor();

       // Update filter
       filter.updateIMU(
           mpu.getGyroX_rads(), mpu.getGyroY_rads(), mpu.getGyroZ_rads(),
           mpu.getAccelX_mss(), mpu.getAccelY_mss(), mpu.getAccelZ_mss()
       );

       // Get quaternion
       imu_msg.orientation.w = filter.q0;
       imu_msg.orientation.x = filter.q1;
       imu_msg.orientation.y = filter.q2;
       imu_msg.orientation.z = filter.q3;
   }
   ```

**Trade-off**: Offloading sensor fusion to ESP32 reduces Raspberry Pi CPU load but increases ESP32 complexity.

## Next Steps

After ESP32 setup is complete:
1. Verify IMU data reception on Raspberry Pi (see `microros-integration.md`)
2. Configure robot_localization EKF to fuse IMU with odometry (Task 004)
3. Integrate IMU tilt monitoring in safety supervisor (Task 073)

## References

- [micro-ROS Documentation](https://micro.ros.org/)
- [MPU9250 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/)
- [ESP32 PlatformIO Setup](https://docs.platformio.org/en/latest/platforms/espressif32.html)
- [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino)
