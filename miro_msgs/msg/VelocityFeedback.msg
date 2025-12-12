# VelocityFeedback.msg
# Real-time velocity comparison feedback for calibration monitoring
# Published at 10Hz by feedback_monitor node

# Header with timestamp
std_msgs/Header header

# Commanded velocities (from /cmd_vel)
float64 cmd_linear_x    # Commanded linear velocity (m/s)
float64 cmd_angular_z   # Commanded angular velocity (rad/s)

# Actual velocities (from /odometry/filtered)
float64 actual_linear_x    # Actual linear velocity (m/s)
float64 actual_angular_z   # Actual angular velocity (rad/s)

# Error metrics - Linear
float64 linear_error_abs       # Absolute linear error (m/s)
float64 linear_error_percent   # Linear error percentage (0-100)

# Error metrics - Angular
float64 angular_error_abs      # Absolute angular error (rad/s)
float64 angular_error_percent  # Angular error percentage (0-100)

# Status indicator
uint8 GREEN=0
uint8 YELLOW=1
uint8 RED=2
uint8 status              # Color-coded status based on error thresholds

# Human-readable status message
string status_message     # Formatted status string for display
