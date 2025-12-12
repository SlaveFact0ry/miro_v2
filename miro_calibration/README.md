# MIRO Calibration System

Manual calibration and hardware tuning system for the MIRO v2 autonomous pool cleaning robot.

## Overview

The calibration system provides tools for manual hardware testing, parameter tuning, and validation before autonomous operation. It enables operators to verify sensor accuracy, test motor response, and calibrate odometry parameters using joystick control with fixed speed presets.

## Features

### Implemented (Task 001)

- **Speed Preset Manager**: Three fixed speed presets (slow/medium/fast) switchable via joystick buttons
  - Slow: 0.1 m/s linear, 0.2 rad/s angular
  - Medium: 0.2 m/s linear, 0.4 rad/s angular (default)
  - Fast: 0.3 m/s linear, 0.6 rad/s angular

### Planned

- Real-time velocity feedback monitoring
- Sensor health monitoring
- Automated calibration tests (straight-line, rotation, sensor quality)
- Data logging and analysis
- Configuration file management
- Calibration report generation

## Installation

### Build the Package

```bash
cd ~/miro_v2
colcon build --packages-select miro_msgs miro_calibration
source install/setup.bash
```

## Usage

### Launch Calibration System

```bash
# Launch with joystick driver
ros2 launch miro_calibration calibration.launch.py

# Launch without joystick driver (if already running)
ros2 launch miro_calibration calibration.launch.py launch_joy:=false
```

### Joystick Controls

**PS4 Controller Button Mapping**:
- **X Button (Button 0)**: Slow preset (0.1 m/s)
- **Circle Button (Button 1)**: Medium preset (0.2 m/s)
- **Triangle Button (Button 2)**: Fast preset (0.3 m/s)

### Monitor Speed Mode

```bash
# Watch current speed mode
ros2 topic echo /calibration/speed_mode

# Check message rate
ros2 topic hz /calibration/speed_mode
```

## Configuration

Speed presets can be customized in `config/speed_presets.yaml`:

```yaml
speed_preset_manager:
  ros__parameters:
    # Button mappings
    button_slow: 0
    button_medium: 1
    button_fast: 2

    # Speed values
    speed_slow: 0.1
    speed_medium: 0.2
    speed_fast: 0.3

    # Angular speeds
    angular_slow: 0.2
    angular_medium: 0.4
    angular_fast: 0.6

    # Default preset
    default_preset: "medium"

    # Debounce time
    debounce_time: 0.3
```

## Topics

### Published

- `/calibration/speed_mode` (`miro_msgs/SpeedMode`): Current speed preset

### Subscribed

- `/joy` (`sensor_msgs/Joy`): Joystick input

## Dependencies

- **ROS2 Humble**: Core ROS2 framework
- **joy**: Joystick driver package
- **miro_msgs**: Custom message definitions (SpeedMode)

## Hardware Requirements

- Joystick controller (PS4, Xbox, or compatible)
- Bluetooth or USB connection to Raspberry Pi

## Troubleshooting

### Joystick not detected

```bash
# Check joystick device
ls -l /dev/input/js*

# Test joystick manually
ros2 run joy joy_node

# In another terminal, check output
ros2 topic echo /joy
```

### Speed mode not changing

1. Verify joystick is publishing: `ros2 topic echo /joy`
2. Check button indices in controller (may differ from PS4)
3. Adjust button mapping in `speed_presets.yaml`
4. Check debounce time if buttons feel unresponsive

## Development Status

**Current**: Task 001 - Speed Preset Manager (✓ Complete)

**Next Tasks**:
- Task 002: Velocity Feedback Monitor
- Task 003: Sensor Health Monitor
- Task 004: Data Logger
- Task 005: Safety Supervisor Integration

See `design/calibration-system/tasks/` for complete task list.

## License

MIT License

## Authors

MIRO v2 Development Team
