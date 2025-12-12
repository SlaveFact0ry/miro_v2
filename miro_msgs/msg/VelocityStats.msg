# VelocityStats.msg
# Accumulated velocity error statistics for calibration assessment
# Published at 1Hz by feedback_monitor node

# Header with timestamp
std_msgs/Header header

# Statistics window duration
float64 window_duration_sec   # Duration of statistics window (seconds)

# Sample count
uint32 sample_count          # Number of samples in current window

# Linear velocity error statistics
float64 linear_error_mean       # Mean absolute linear error (m/s)
float64 linear_error_std_dev    # Standard deviation of linear error (m/s)
float64 linear_error_min        # Minimum linear error in window (m/s)
float64 linear_error_max        # Maximum linear error in window (m/s)

# Angular velocity error statistics
float64 angular_error_mean      # Mean absolute angular error (rad/s)
float64 angular_error_std_dev   # Standard deviation of angular error (rad/s)
float64 angular_error_min       # Minimum angular error in window (rad/s)
float64 angular_error_max       # Maximum angular error in window (rad/s)

# Percentage error statistics
float64 linear_percent_mean     # Mean linear error percentage
float64 angular_percent_mean    # Mean angular error percentage
