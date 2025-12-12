# miro_msgs

Custom ROS2 message definitions for the Miro autonomous pool cleaning system.

## Overview

This package provides message types used for mission control, safety monitoring, coverage tracking, and path execution in the autonomous pool cleaning navigation system.

## Message Definitions

### MissionState.msg

Mission state tracking with enumeration constants.

**Fields:**
- `uint8 state` - Current mission state (see constants below)
- `builtin_interfaces/Time timestamp` - Timestamp of state change

**Constants:**
- `IDLE = 0` - System idle, waiting for commands
- `MAPPING = 1` - Actively mapping pool boundary
- `READY = 2` - Map validated, ready to start cleaning
- `CLEANING = 3` - Executing cleaning path
- `PAUSED = 4` - Mission paused (can resume)
- `COMPLETE = 5` - Cleaning mission completed
- `EMERGENCY_STOP = 6` - Emergency stop activated

### SafetyStatus.msg

Safety monitoring status with violation detection.

**Fields:**
- `bool tilt_ok` - Robot is within safe tilt angle threshold
- `bool battery_ok` - Battery level is sufficient for operation
- `bool comms_ok` - Communication with control system is stable
- `string violation_reason` - Human-readable reason for safety violation (empty if all OK)

### CoverageStatus.msg

Coverage tracking metrics during cleaning mission.

**Fields:**
- `float32 coverage_percentage` - Percentage of pool area covered (0-100)
- `int32 visited_cells` - Number of grid cells that have been visited
- `int32 total_cells` - Total number of grid cells in the map
- `builtin_interfaces/Duration mission_time` - Time elapsed since mission start

### ValidationReport.msg

Map validation results after boundary mapping phase.

**Fields:**
- `bool passed` - Overall validation result (true if map is valid)
- `bool boundary_closed` - Whether boundary forms a closed polygon
- `geometry_msgs/Point dimensions` - Pool dimensions (x=length, y=width, z=depth in meters)
- `float32 artifact_percentage` - Percentage of map occupied by artifacts/obstacles
- `string[] error_messages` - Detailed error messages (empty array if validation passed)

### PathProgress.msg

Real-time path execution progress tracking.

**Fields:**
- `int32 current_segment` - Index of current path segment being executed
- `int32 total_segments` - Total number of segments in the cleaning path
- `geometry_msgs/Pose last_pose` - Last recorded robot pose
- `float32 completion_percentage` - Overall path completion percentage (0-100)

## Dependencies

- `builtin_interfaces` - For Time and Duration types
- `geometry_msgs` - For Point and Pose types
- `rosidl_default_generators` - For message generation
- `rosidl_default_runtime` - For runtime message support

## Building

From the workspace root:

```bash
colcon build --packages-select miro_msgs
source install/setup.bash
```

## Verification

List all generated messages:

```bash
ros2 interface list | grep miro_msgs
```

Show a specific message definition:

```bash
ros2 interface show miro_msgs/msg/MissionState
ros2 interface show miro_msgs/msg/SafetyStatus
ros2 interface show miro_msgs/msg/CoverageStatus
ros2 interface show miro_msgs/msg/ValidationReport
ros2 interface show miro_msgs/msg/PathProgress
```

## Usage Example

```python
from miro_msgs.msg import MissionState, SafetyStatus

# Create and publish mission state
mission_state = MissionState()
mission_state.state = MissionState.CLEANING
mission_state.timestamp = self.get_clock().now().to_msg()

# Create safety status
safety = SafetyStatus()
safety.tilt_ok = True
safety.battery_ok = True
safety.comms_ok = True
safety.violation_reason = ""
```

## License

TODO: Add license information

## Version

0.1.0
